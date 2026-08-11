#include <Arduino.h>
#include <Wire.h>

constexpr bool LEG_OUTPUT_ENABLED = false;
constexpr uint32_t HOST_TIMEOUT_MS = 250U;
constexpr uint32_t SENSOR_PERIOD_MS = 20U;

enum class LegState : uint8_t {
  STOWED, DEPLOYING, DEPLOYED, RETRACTING, SAFE_STOP, FAULT
};

LegState state = LegState::SAFE_STOP;
uint32_t last_host_command_ms = 0U;
uint32_t last_sensor_ms = 0U;

void disableLegImmediately() {
  // TODO: drive the verified hardware-enable signal inactive.
}

void sampleSensors() {
  // TODO: non-blocking TOF-10120, MPU-9250 and BNO055 reads.
  // Report validity/calibration; never substitute invalid values with zero.
}

void processHostProtocol() {
  // TODO: framed CRC/session protocol. Text commands are not accepted.
}

void setup() {
  disableLegImmediately();
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  processHostProtocol();
  const uint32_t now = millis();
  if (now - last_sensor_ms >= SENSOR_PERIOD_MS) {
    last_sensor_ms = now;
    sampleSensors();
  }
  if (!LEG_OUTPUT_ENABLED || now - last_host_command_ms > HOST_TIMEOUT_MS) {
    disableLegImmediately();
    state = LegState::SAFE_STOP;
  }
}
