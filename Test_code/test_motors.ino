// Two-motor direction and encoder test for ESP32-WROOM DevKit V1.
// Check the pin assignments below against your wiring before powering the motors.

// Motor 1 uses the pins from the supplied single-motor example.
const int ENC1_A = 18;
const int ENC1_B = 19;
const int M1_IN1 = 32;
const int M1_IN2 = 33;
const int M1_STBY = 4;

// Motor 2 uses a separate set of ESP32 GPIOs.
const int ENC2_A = 25;
const int ENC2_B = 26;
const int M2_IN1 = 27;
const int M2_IN2 = 14;
const int M2_STBY = 13;

const int TEST_SPEED = 150;       // 0..255 PWM magnitude
const unsigned long RUN_MS = 2000;
const unsigned long PAUSE_MS = 1000;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR readEncoder1A() {
  const int a = digitalRead(ENC1_A);
  const int b = digitalRead(ENC1_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder1Count += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR readEncoder2A() {
  const int a = digitalRead(ENC2_A);
  const int b = digitalRead(ENC2_B);
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder2Count += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void setMotor(int in1, int in2, int standby, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(standby, HIGH);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0); // Coast
  }
}

long readCount(int motor) {
  long count;
  portENTER_CRITICAL(&encoderMux);
  count = (motor == 1) ? encoder1Count : encoder2Count;
  portEXIT_CRITICAL(&encoderMux);
  return count;
}

void resetCount(int motor) {
  portENTER_CRITICAL(&encoderMux);
  if (motor == 1) encoder1Count = 0;
  else encoder2Count = 0;
  portEXIT_CRITICAL(&encoderMux);
}

void runMotorTest(int motor, int in1, int in2, int standby) {
  const char *name = (motor == 1) ? "Motor 1" : "Motor 2";

  for (int direction : {1, -1}) {
    resetCount(motor);
    Serial.printf("%s %s: start\n", name, direction > 0 ? "forward" : "backward");

    const unsigned long started = millis();
    unsigned long lastPrint = 0;
    setMotor(in1, in2, standby, direction * TEST_SPEED);
    while (millis() - started < RUN_MS) {
      if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("%s %s encoder: %ld\n", name,
                      direction > 0 ? "forward" : "backward", readCount(motor));
      }
      delay(1);
    }

    setMotor(in1, in2, standby, 0);
    Serial.printf("%s %s: final encoder count = %ld\n", name,
                  direction > 0 ? "forward" : "backward", readCount(motor));
    delay(PAUSE_MS);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_STBY, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_STBY, OUTPUT);
  digitalWrite(M1_STBY, HIGH);
  digitalWrite(M2_STBY, HIGH);
  setMotor(M1_IN1, M1_IN2, M1_STBY, 0);
  setMotor(M2_IN1, M2_IN2, M2_STBY, 0);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoder2A, CHANGE);
  Serial.println("Two-motor encoder test starting");
}

void loop() {
  runMotorTest(1, M1_IN1, M1_IN2, M1_STBY);
  runMotorTest(2, M2_IN1, M2_IN2, M2_STBY);
  Serial.println("Test cycle complete; repeating in 3 seconds");
  delay(3000);
}
