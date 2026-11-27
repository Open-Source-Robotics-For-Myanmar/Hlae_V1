// Closed-loop differential drive for ESP32-WROOM using micro-ROS over Wi-Fi.
// Robot-specific settings are in robot_config.h.

#include "robot_config.h"
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

volatile int32_t leftTicks = 0;
volatile int32_t rightTicks = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

rcl_node_t node;
rcl_subscription_t cmdSubscriber;
rcl_publisher_t odomPublisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
geometry_msgs__msg__Twist cmdMsg;
nav_msgs__msg__Odometry odomMsg;

struct WheelPid {
  float kp, ki, kd;
  float integral;
  float previousError;
  float target;
  float measured;
  int pwm;
};

WheelPid leftPid = {LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, 0, 0, 0, 0, 0};
WheelPid rightPid = {RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, 0, 0, 0, 0, 0};
bool rosReady = false;
bool haveCommand = false;
unsigned long lastCommandMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastOdomMs = 0;
int32_t previousLeftTicks = 0;
int32_t previousRightTicks = 0;
float odomX = 0.0f;
float odomY = 0.0f;
float odomYaw = 0.0f;

#define RCCHECK(fn) do { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) fatalRosError(); } while (0)

void stopOutputs() {
  analogWrite(M1_IN1, 0); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0); analogWrite(M2_IN2, 0);
}

void fatalRosError() {
  stopOutputs();
  rosReady = false;
  while (true) delay(1000);
}

void IRAM_ATTR readLeftEncoder() {
  const int a = digitalRead(ENC1_A);
  const int b = digitalRead(ENC1_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  leftTicks += ((a == b) ? 1 : -1) * LEFT_ENCODER_SIGN;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR readRightEncoder() {
  const int a = digitalRead(ENC2_A);
  const int b = digitalRead(ENC2_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  rightTicks += ((a == b) ? 1 : -1) * RIGHT_ENCODER_SIGN;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void setMotor(int in1, int in2, int enablePin, int pwm, int motorSign) {
  pwm = constrain(pwm * motorSign, -PWM_LIMIT, PWM_LIMIT);
  digitalWrite(enablePin, HIGH);
  if (pwm > 0) {
    analogWrite(in1, pwm); analogWrite(in2, 0);
  } else if (pwm < 0) {
    analogWrite(in1, 0); analogWrite(in2, -pwm);
  } else {
    analogWrite(in1, 0); analogWrite(in2, 0);
  }
}

void setWheelTargets(float linear, float angular) {
  float left = linear - angular * WHEEL_BASE_M * 0.5f;
  float right = linear + angular * WHEEL_BASE_M * 0.5f;
  const float maxAbs = fmaxf(fabsf(left), fabsf(right));
  if (maxAbs > MAX_WHEEL_SPEED_MPS) {
    const float factor = MAX_WHEEL_SPEED_MPS / maxAbs;
    left *= factor;
    right *= factor;
  }
  leftPid.target = left;
  rightPid.target = right;
}

void cmdCallback(const void *msgIn) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgIn;
  setWheelTargets((float)msg->linear.x, (float)msg->angular.z);
  lastCommandMs = millis();
  haveCommand = true;
}

int updatePid(WheelPid &pid, float dt) {
  const float error = pid.target - pid.measured;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  const float derivative = (error - pid.previousError) / dt;
  pid.previousError = error;
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  output = constrain(output, (float)-PWM_LIMIT, (float)PWM_LIMIT);
  int pwm = (int)lroundf(output);
  if (fabsf(pid.target) > 0.001f && abs(pwm) > 0 && abs(pwm) < PWM_MIN_MOVING)
    pwm = (pwm > 0) ? PWM_MIN_MOVING : -PWM_MIN_MOVING;
  return pwm;
}

void updateControl(unsigned long now) {
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  const float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  if (!haveCommand || now - lastCommandMs > CMD_TIMEOUT_MS) {
    leftPid.target = rightPid.target = 0.0f;
  }

  int32_t leftNow, rightNow;
  portENTER_CRITICAL(&encoderMux);
  leftNow = leftTicks;
  rightNow = rightTicks;
  portEXIT_CRITICAL(&encoderMux);

  const int32_t leftDelta = leftNow - previousLeftTicks;
  const int32_t rightDelta = rightNow - previousRightTicks;
  previousLeftTicks = leftNow;
  previousRightTicks = rightNow;
  const float metersPerTick = (PI * WHEEL_DIAMETER_M) / ENCODER_COUNTS_PER_REV;
  leftPid.measured = leftDelta * metersPerTick / dt;
  rightPid.measured = rightDelta * metersPerTick / dt;

  leftPid.pwm = updatePid(leftPid, dt);
  rightPid.pwm = updatePid(rightPid, dt);
  setMotor(M1_IN1, M1_IN2, M1_ENABLE, leftPid.pwm, LEFT_MOTOR_SIGN);
  setMotor(M2_IN1, M2_IN2, M2_ENABLE, rightPid.pwm, RIGHT_MOTOR_SIGN);

  // Integrate measured wheel travel for planar differential-drive odometry.
  const float dl = leftDelta * metersPerTick;
  const float dr = rightDelta * metersPerTick;
  const float distance = (dl + dr) * 0.5f;
  const float dtheta = (dr - dl) / WHEEL_BASE_M;
  odomX += distance * cosf(odomYaw + dtheta * 0.5f);
  odomY += distance * sinf(odomYaw + dtheta * 0.5f);
  odomYaw = atan2f(sinf(odomYaw + dtheta), cosf(odomYaw + dtheta));
}

void publishOdometry(unsigned long now) {
  if (now - lastOdomMs < ODOM_PUBLISH_PERIOD_MS) return;
  lastOdomMs = now;

  const float linear = (leftPid.measured + rightPid.measured) * 0.5f;
  const float angular = (rightPid.measured - leftPid.measured) / WHEEL_BASE_M;
  const uint64_t epochMs = rmw_uros_epoch_millis();
  odomMsg.header.stamp.sec = (int32_t)(epochMs / 1000ULL);
  odomMsg.header.stamp.nanosec = (uint32_t)((epochMs % 1000ULL) * 1000000ULL);
  odomMsg.pose.pose.position.x = odomX;
  odomMsg.pose.pose.position.y = odomY;
  odomMsg.pose.pose.position.z = 0.0;
  odomMsg.pose.pose.orientation.x = 0.0;
  odomMsg.pose.pose.orientation.y = 0.0;
  odomMsg.pose.pose.orientation.z = sinf(odomYaw * 0.5f);
  odomMsg.pose.pose.orientation.w = cosf(odomYaw * 0.5f);
  odomMsg.twist.twist.linear.x = linear;
  odomMsg.twist.twist.angular.z = angular;
  (void)rcl_publish(&odomPublisher, &odomMsg, NULL);
}

void setup() {
  Serial.begin(115200);
  IPAddress agentIp(AGENT_IP_0, AGENT_IP_1, AGENT_IP_2, AGENT_IP_3);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agentIp, AGENT_PORT);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT); pinMode(M1_ENABLE, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT); pinMode(M2_ENABLE, OUTPUT);
  digitalWrite(M1_ENABLE, HIGH); digitalWrite(M2_ENABLE, HIGH);
  stopOutputs();
  attachInterrupt(digitalPinToInterrupt(ENC1_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readRightEncoder, CHANGE);

  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hlae_base_driver", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &cmdSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC));
  RCCHECK(rclc_publisher_init_default(
    &odomPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC));
  if (!rosidl_runtime_c__String__assign(&odomMsg.header.frame_id, ODOM_FRAME_ID)) fatalRosError();
  if (!rosidl_runtime_c__String__assign(&odomMsg.child_frame_id, BASE_FRAME_ID)) fatalRosError();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmdSubscriber, &cmdMsg, &cmdCallback, ON_NEW_DATA));

  rmw_uros_sync_session(1000);
  lastCommandMs = millis();
  lastControlMs = lastCommandMs;
  lastOdomMs = lastCommandMs;
  rosReady = true;
}

void loop() {
  if (!rosReady || WiFi.status() != WL_CONNECTED) {
    stopOutputs();
    delay(10);
    return;
  }
  (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  const unsigned long now = millis();
  updateControl(now);
  publishOdometry(now);
}
