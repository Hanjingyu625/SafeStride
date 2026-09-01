#include <assert.h>
#include <stdio.h>

#include "../firmware/safestride_mcu/safestride_mcu.ino"

namespace {

uint32_t g_test_millis = 0UL;
int g_pressure_adc = 200;
int g_hall_adc = 512;

}  // namespace

HardwareSerial Serial;

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return LOW; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t pin) {
  return pin == safestride_config::HALL_ANALOG_PIN
      ? g_hall_adc
      : g_pressure_adc;
}
int digitalPinToInterrupt(uint8_t) { return 0; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return g_test_millis; }
uint32_t micros() { return g_test_millis * 1000UL; }
void delayMicroseconds(unsigned int) {}

void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t) { return 1U; }
size_t HardwareSerial::write(const uint8_t*, size_t length) {
  return length;
}
size_t HardwareSerial::print(const char*) { return 1U; }
size_t HardwareSerial::println(const char*) { return 1U; }

int main() {
  namespace proto = safestride_protocol;

  g_pressure.begin(0UL);
  g_boot_id = 0x12345678UL;
  g_session_id = 0xABCDEF01UL;
  g_session_active = true;
  g_session_offer_active = false;
  g_state = ControllerState::ARMED;
  g_fault_bits = 0U;
  g_watchdog_timed_out = false;
  g_have_command_sequence = true;
  g_last_command_sequence = 10U;
  g_last_valid_command_ms = 0UL;
  g_last_session_activity_ms = 0UL;
  g_active_command_ttl_ms = 200U;

  g_test_millis = 201UL;
  enforceWatchdogs(g_test_millis);
  assert(!g_session_active);
  assert(!g_session_offer_active);
  assert(g_state == ControllerState::SAFE_STOP);
  assert(g_watchdog_timed_out);

  uint8_t command_payload[proto::COMMAND_PAYLOAD_SIZE] = {};
  proto::writeU16(command_payload + 4U, 200U);
  command_payload[6U] = 0U;
  proto::FrameView old_command = {
      proto::TYPE_COMMAND,
      0U,
      11U,
      proto::COMMAND_PAYLOAD_SIZE,
      0xABCDEF01UL,
      g_test_millis,
      command_payload,
  };
  assert(!handleCommand(old_command));

  uint8_t session_payload[proto::SESSION_START_PAYLOAD_SIZE] = {};
  proto::writeU32(session_payload, g_boot_id);
  session_payload[4U] = proto::BOARD_ROLE_DRIVE;
  session_payload[5U] = proto::VERSION;
  proto::writeU16(session_payload + 6U, proto::SCHEMA_ID);
  proto::writeU32(
      session_payload + 8U, proto::FIRMWARE_RELEASE_ID);
  proto::FrameView session_start = {
      proto::TYPE_SESSION_START,
      0U,
      12U,
      proto::SESSION_START_PAYLOAD_SIZE,
      0x11112222UL,
      g_test_millis,
      session_payload,
  };
  assert(!handleSessionStart(session_start));

  sendHello();
  assert(g_session_offer_active);
  assert(handleSessionStart(session_start));
  assert(g_session_active);
  assert(g_state == ControllerState::DISARMED);
  assert(g_watchdog_timed_out);

  command_payload[6U] = 1U;
  proto::FrameView enable_command = {
      proto::TYPE_COMMAND,
      0U,
      13U,
      proto::COMMAND_PAYLOAD_SIZE,
      g_session_id,
      g_test_millis,
      command_payload,
  };
  assert(!handleCommand(enable_command));
  assert(g_state == ControllerState::DISARMED);

  command_payload[6U] = 0U;
  proto::FrameView disable_command = {
      proto::TYPE_COMMAND,
      0U,
      14U,
      proto::COMMAND_PAYLOAD_SIZE,
      g_session_id,
      g_test_millis,
      command_payload,
  };
  assert(handleCommand(disable_command));
  assert(!g_watchdog_timed_out);
  assert(g_state == ControllerState::DISARMED);

  // Hall feedback mode still requires a short stationary dwell and an
  // explicit motion command before the controller can arm.
  g_stationary_tracking = true;
  g_stationary_since_ms =
      g_test_millis - cfg::ARM_STATIONARY_DWELL_MS;
  proto::writeI32(command_payload + 0U, 500L);
  command_payload[6U] = 1U;
  proto::FrameView motion_enable_command = {
      proto::TYPE_COMMAND,
      0U,
      15U,
      proto::COMMAND_PAYLOAD_SIZE,
      g_session_id,
      g_test_millis,
      command_payload,
  };
  assert(handleCommand(motion_enable_command));
  assert(g_state == ControllerState::ARMED);
  assert(g_requested_mrad_s == 500L);

  // Direct dead-man mode must re-arm after a complete release/press cycle
  // without waiting for Hall stationary dwell.
  g_pressure_adc = 0;
  g_test_millis += cfg::PRESSURE_SAMPLE_PERIOD_MS;
  g_pressure.update(g_test_millis);
  refreshPhysicalSafety();
  assert(!deadmanActive());
  assert(g_state == ControllerState::SAFE_STOP);

  command_payload[6U] = 0U;
  disable_command.sequence = 16U;
  assert(handleCommand(disable_command));
  assert(g_state == ControllerState::DISARMED);

  g_pressure_adc = 200;
  g_test_millis += cfg::PRESSURE_SAMPLE_PERIOD_MS;
  g_pressure.update(g_test_millis);
  assert(deadmanActive());
  g_stationary_tracking = false;
  command_payload[6U] = 1U;
  motion_enable_command.sequence = 17U;
  assert(handleCommand(motion_enable_command));
  assert(g_state == ControllerState::ARMED);

  printf("firmware watchdog/session state-machine tests: OK\n");
  return 0;
}
