// Arduino Uno bench-only motor test for two dual-input H-bridge drivers.
// Keep the wheels lifted and start with a small command such as M,20.
//
// Serial commands at 115200 baud:
//   M,100   forward at PWM 100
//   M,-100  reverse at PWM 100
//   M,0     stop

#include <Arduino.h>

constexpr uint8_t LEFT_PWM = 5U;
constexpr uint8_t LEFT_IN1 = 6U;
constexpr uint8_t LEFT_IN2 = 8U;
constexpr uint8_t RIGHT_PWM = 9U;
constexpr uint8_t RIGHT_IN1 = 10U;
constexpr uint8_t RIGHT_IN2 = 12U;

constexpr bool LEFT_REVERSED = false;
constexpr bool RIGHT_REVERSED = true;
constexpr int MAX_PWM = 100;
constexpr uint16_t COMMAND_TIMEOUT_MS = 500U;

constexpr uint16_t COMMAND_BUFFER_SIZE = 16U;
char command_buffer[COMMAND_BUFFER_SIZE];
uint8_t command_length = 0U;
uint32_t last_command_ms = 0UL;
bool command_active = false;

void driveOneMotor(
    int signed_pwm,
    uint8_t pwm_pin,
    uint8_t in1,
    uint8_t in2,
    bool reversed) {
  if (reversed) {
    signed_pwm = -signed_pwm;
  }
  const int pwm = abs(signed_pwm);

  if (signed_pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (signed_pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm_pin, pwm);
}

void driveMotors(int signed_pwm) {
  driveOneMotor(
      signed_pwm, LEFT_PWM, LEFT_IN1, LEFT_IN2, LEFT_REVERSED);
  driveOneMotor(
      signed_pwm, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, RIGHT_REVERSED);
}

void stopMotors() {
  driveMotors(0);
}

bool parseSignedPwm(const char* command, int& signed_pwm) {
  if (command[0] != 'M' || command[1] != ',') {
    return false;
  }

  char* end = NULL;
  const long parsed = strtol(command + 2, &end, 10);
  if (end == command + 2 || *end != '\0') {
    return false;
  }
  signed_pwm = constrain(parsed, -MAX_PWM, MAX_PWM);
  return true;
}

void processSerial() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      return;
    }
    const char character = static_cast<char>(incoming);
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      command_buffer[command_length] = '\0';
      int signed_pwm = 0;
      if (parseSignedPwm(command_buffer, signed_pwm)) {
        driveMotors(signed_pwm);
        command_active = signed_pwm != 0;
        last_command_ms = millis();
        Serial.println("OK");
      } else {
        stopMotors();
        command_active = false;
        Serial.println("ERR");
      }
      command_length = 0U;
      continue;
    }
    if (command_length + 1U >= COMMAND_BUFFER_SIZE) {
      command_length = 0U;
      stopMotors();
      command_active = false;
      Serial.println("ERR");
      continue;
    }
    command_buffer[command_length++] = character;
  }
}

void setup() {
  digitalWrite(LEFT_PWM, LOW);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_PWM, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  processSerial();
  if (command_active &&
      static_cast<uint32_t>(millis() - last_command_ms) >=
          COMMAND_TIMEOUT_MS) {
    stopMotors();
    command_active = false;
    Serial.println("TIMEOUT");
  }
}
