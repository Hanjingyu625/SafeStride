// Arduino Uno bench-only test for one driver feeding both motors.
// Keep both wheels lifted and start with a small command such as M,20.
//
// Serial commands at 115200 baud:
//   M,100   forward at PWM 100
//   M,-100  reverse at PWM 100
//   M,0     stop

#include <Arduino.h>

constexpr uint8_t MOTOR_PWM = 5U;
constexpr uint8_t MOTOR_IN1 = 6U;
constexpr uint8_t MOTOR_IN2 = 8U;
constexpr bool MOTOR_REVERSED = false;
constexpr int MAX_PWM = 100;
constexpr uint16_t COMMAND_TIMEOUT_MS = 500U;

constexpr uint16_t COMMAND_BUFFER_SIZE = 16U;
char command_buffer[COMMAND_BUFFER_SIZE];
uint8_t command_length = 0U;
uint32_t last_command_ms = 0UL;
bool command_active = false;

void driveMotors(int signed_pwm) {
  if (MOTOR_REVERSED) {
    signed_pwm = -signed_pwm;
  }
  const int pwm = abs(signed_pwm);

  if (signed_pwm > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (signed_pwm < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }
  analogWrite(MOTOR_PWM, pwm);
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
  digitalWrite(MOTOR_PWM, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
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
