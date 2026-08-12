#include <Arduino.h>
#include <Wire.h>

#include "tof10120_sensor.h"

constexpr bool LEG_OUTPUT_ENABLED = false;
constexpr uint32_t HOST_TIMEOUT_MS = 250U;

enum class LegState : uint8_t {
  STOWED, DEPLOYING, DEPLOYED, RETRACTING, SAFE_STOP, FAULT
};

LegState state = LegState::SAFE_STOP;
uint32_t last_host_command_ms = 0U;
Tof10120Sensor tof;

void disableLegImmediately() {
  // TODO: drive the verified hardware-enable signal inactive.
}

void sampleSensors() {
  tof.update(millis());
  // TODO: add non-blocking MPU-9250 and BNO055 reads. Report validity and
  // calibration; never substitute invalid values with zero.
}

void processHostProtocol() {
  // TODO: framed CRC/session protocol. Text commands are not accepted.
}

void setup() {
  disableLegImmediately();
  Wire.begin();
  Serial.begin(115200);
  tof.begin(millis());
}

void loop() {
  processHostProtocol();
  const uint32_t now = millis();
  sampleSensors();
  if (!LEG_OUTPUT_ENABLED || now - last_host_command_ms > HOST_TIMEOUT_MS) {
    disableLegImmediately();
    state = LegState::SAFE_STOP;
  }
}
