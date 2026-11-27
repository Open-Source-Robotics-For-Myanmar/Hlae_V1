#pragma once

// Edit this file for your robot and Wi-Fi network. Keep the .ino unchanged.

// Wi-Fi network and the computer running the micro-ROS Agent.
// Agent IP must be reachable from the ESP32 on the same network/subnet.
#define WIFI_SSID "CHANGE_ME"
#define WIFI_PASSWORD "CHANGE_ME"

#define AGENT_IP_0 192
#define AGENT_IP_1 168
#define AGENT_IP_2 1
#define AGENT_IP_3 10
#define AGENT_PORT 8888

// Motor 1 = left wheel; Motor 2 = right wheel.
constexpr int ENC1_A = 18;
constexpr int ENC1_B = 19;
constexpr int M1_IN1 = 32;
constexpr int M1_IN2 = 33;
constexpr int M1_ENABLE = 4;
constexpr int ENC2_A = 25;
constexpr int ENC2_B = 26;
constexpr int M2_IN1 = 27;
constexpr int M2_IN2 = 14;
constexpr int M2_ENABLE = 13;

// Set these from measurements. Encoder counts/revolution must use the same
// count convention as the ISR (ENC_A CHANGE, quadrature direction on ENC_B).
constexpr float WHEEL_DIAMETER_M = 0.065f;
constexpr float WHEEL_BASE_M = 0.160f;
constexpr float ENCODER_COUNTS_PER_REV = 600.0f;

// Set a sign to -1 if that motor or encoder is reversed relative to forward.
constexpr int LEFT_MOTOR_SIGN = 1;
constexpr int RIGHT_MOTOR_SIGN = 1;
constexpr int LEFT_ENCODER_SIGN = 1;
constexpr int RIGHT_ENCODER_SIGN = 1;

// Closed-loop target and controller settings. Tune PID after basic wiring tests.
constexpr float MAX_WHEEL_SPEED_MPS = 0.30f;
constexpr int PWM_LIMIT = 255;
constexpr int PWM_MIN_MOVING = 0; // Increase only after measuring motor deadband.
constexpr float LEFT_PID_KP = 450.0f;
constexpr float LEFT_PID_KI = 80.0f;
constexpr float LEFT_PID_KD = 0.0f;
constexpr float RIGHT_PID_KP = 450.0f;
constexpr float RIGHT_PID_KI = 80.0f;
constexpr float RIGHT_PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 0.5f;

// Timing, topics, and frame IDs.
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long CMD_TIMEOUT_MS = 500;
constexpr unsigned long ODOM_PUBLISH_PERIOD_MS = 50;
#define CMD_VEL_TOPIC "/cmd_vel"
#define ODOM_TOPIC "/odom"
#define ODOM_FRAME_ID "odom"
#define BASE_FRAME_ID "base_link"
