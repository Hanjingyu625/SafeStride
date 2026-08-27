#include <Arduino.h>

constexpr uint8_t HALL_PIN = 2U;
constexpr uint8_t PRESSURE_LEFT_PIN = A0;
constexpr uint8_t PRESSURE_RIGHT_PIN = A1;
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;
constexpr uint32_t HALL_MIN_PULSE_INTERVAL_US = 500UL;
constexpr uint32_t HALL_PULSES_PER_REV = 6UL;
constexpr float PRESSURE_THRESHOLD = 25.0F;

volatile uint32_t g_pulses = 0UL;
volatile uint32_t g_last_pulse_us = 0UL;
volatile uint32_t g_period_us = 0UL;
uint32_t g_last_print_ms = 0UL;

void hallIsr() {
  const uint32_t now_us = micros();
  const uint32_t elapsed = now_us - g_last_pulse_us;
  if (g_last_pulse_us != 0UL && elapsed < HALL_MIN_PULSE_INTERVAL_US) {
    return;
  }
  if (g_last_pulse_us != 0UL) {
    g_period_us = elapsed;
  }
  g_last_pulse_us = now_us;
  ++g_pulses;
}

void setup() {
  digitalWrite(MOTOR_PWM_PIN, LOW);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallIsr, FALLING);
  Serial.begin(115200UL);
  Serial.println(F("pulses,period_us,rpm,pressure_left,pressure_right,deadman"));
}

void loop() {
  if (millis() - g_last_print_ms < 100U) {
    return;
  }
  g_last_print_ms = millis();
  uint32_t pulses;
  uint32_t period_us;
  noInterrupts();
  pulses = g_pulses;
  period_us = g_period_us;
  interrupts();
  const float rpm = period_us > 0UL
      ? 60000000.0F /
          (static_cast<float>(period_us) * HALL_PULSES_PER_REV)
      : 0.0F;
  const int left = analogRead(PRESSURE_LEFT_PIN);
  const int right = analogRead(PRESSURE_RIGHT_PIN);
  Serial.print(pulses);
  Serial.print(',');
  Serial.print(period_us);
  Serial.print(',');
  Serial.print(rpm);
  Serial.print(',');
  Serial.print(left);
  Serial.print(',');
  Serial.print(right);
  Serial.print(',');
  Serial.println(
      left >= PRESSURE_THRESHOLD && right >= PRESSURE_THRESHOLD ? 1 : 0);
}
