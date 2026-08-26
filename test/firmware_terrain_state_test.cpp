#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/terrain_mcu/terrain_mcu.ino"

HardwareSerial Serial;
TwoWire Wire;

namespace {

uint32_t g_now_ms = 0UL;
uint16_t g_distance_mm = 500U;
uint8_t g_wire_index = 0U;
uint8_t g_wire_address = 0U;
uint8_t g_wire_register = 0U;
uint8_t g_wire_quantity = 0U;
uint8_t g_wire_write_index = 0U;
uint8_t g_serial_rx[256];
size_t g_serial_rx_length = 0U;
size_t g_serial_rx_index = 0U;
uint8_t g_serial_tx[512];
size_t g_serial_tx_length = 0U;

class BufferStream : public Stream {
 public:
  BufferStream() : length_(0U) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t write(uint8_t value) {
    if (length_ >= sizeof(data_)) {
      return 0U;
    }
    data_[length_++] = value;
    return 1U;
  }
  size_t write(const uint8_t* data, size_t length) {
    if (length_ + length > sizeof(data_)) {
      return 0U;
    }
    memcpy(data_ + length_, data, length);
    length_ += length;
    return length;
  }
  const uint8_t* data() const { return data_; }
  size_t length() const { return length_; }

 private:
  uint8_t data_[256];
  size_t length_;
};

void loadSerialRx(const uint8_t* data, size_t length) {
  assert(length <= sizeof(g_serial_rx));
  memcpy(g_serial_rx, data, length);
  g_serial_rx_length = length;
  g_serial_rx_index = 0U;
}

}  // namespace

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return LOW; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t) { return 0; }
int digitalPinToInterrupt(uint8_t) { return 0; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return g_now_ms; }
uint32_t micros() { return g_now_ms * 1000UL; }
void delayMicroseconds(unsigned int) {}

void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() {
  return static_cast<int>(g_serial_rx_length - g_serial_rx_index);
}
int HardwareSerial::read() {
  if (g_serial_rx_index >= g_serial_rx_length) {
    return -1;
  }
  return g_serial_rx[g_serial_rx_index++];
}
size_t HardwareSerial::write(uint8_t value) {
  if (g_serial_tx_length >= sizeof(g_serial_tx)) {
    return 0U;
  }
  g_serial_tx[g_serial_tx_length++] = value;
  return 1U;
}
size_t HardwareSerial::write(const uint8_t* data, size_t length) {
  if (g_serial_tx_length + length > sizeof(g_serial_tx)) {
    return 0U;
  }
  memcpy(g_serial_tx + g_serial_tx_length, data, length);
  g_serial_tx_length += length;
  return length;
}
size_t HardwareSerial::print(const char*) { return 1U; }
size_t HardwareSerial::println(const char*) { return 1U; }

void TwoWire::begin() {}
void TwoWire::beginTransmission(uint8_t address) {
  g_wire_address = address;
  g_wire_write_index = 0U;
}
size_t TwoWire::write(uint8_t value) {
  if (g_wire_write_index++ == 0U) {
    g_wire_register = value;
  }
  return 1U;
}
uint8_t TwoWire::endTransmission() { return 0U; }
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  g_wire_address = address;
  g_wire_index = 0U;
  g_wire_quantity = quantity;
  return quantity;
}
int TwoWire::available() { return g_wire_quantity - g_wire_index; }
int TwoWire::read() {
  const uint8_t index = g_wire_index++;
  if (g_wire_address == cfg::TOF_I2C_ADDRESS) {
    return index == 0U
        ? static_cast<int>((g_distance_mm >> 8U) & 0xFFU)
        : static_cast<int>(g_distance_mm & 0xFFU);
  }
  if (g_wire_register == 0x00U) {
    return 0xA0;
  }
  if (g_wire_register == 0x1AU) {
    const uint8_t euler[6U] = {0xA0U, 0x05U, 0xA0U, 0x00U, 0xB0U, 0xFFU};
    return euler[index];
  }
  if (g_wire_register == 0x35U) {
    return 0xFF;
  }
  return 0;
}

int main() {
  namespace proto = safestride_protocol;
  setup();
  assert(!g_session_active);
  assert(!g_session_offer_active);

  sendHello();
  assert(g_session_offer_active);
  g_serial_tx_length = 0U;

  uint8_t payload[proto::SESSION_START_PAYLOAD_SIZE];
  proto::writeU32(payload, g_boot_id);
  payload[4U] = proto::BOARD_ROLE_TERRAIN;
  payload[5U] = proto::VERSION;
  proto::writeU16(payload + 6U, proto::SCHEMA_ID);
  proto::writeU32(payload + 8U, proto::FIRMWARE_RELEASE_ID);
  BufferStream command;
  assert(proto::sendFrame(
      command,
      proto::TYPE_SESSION_START,
      7U,
      0x12345678UL,
      0UL,
      payload,
      sizeof(payload)));
  loadSerialRx(command.data(), command.length());
  processHostProtocol();
  assert(g_session_active);
  assert(!g_session_offer_active);
  assert(g_session_id == 0x12345678UL);

  // A delayed start from an older host cannot replace a live session.
  sendHello();
  BufferStream stale_command;
  assert(proto::sendFrame(
      stale_command,
      proto::TYPE_SESSION_START,
      8U,
      0x87654321UL,
      0UL,
      payload,
      sizeof(payload)));
  loadSerialRx(stale_command.data(), stale_command.length());
  processHostProtocol();
  assert(g_session_active);
  assert(g_session_id == 0x12345678UL);

  g_serial_tx_length = 0U;
  g_now_ms = cfg::TOF_SAMPLE_PERIOD_MS;
  g_tof.update(g_now_ms);
  sendTelemetry();

  proto::FrameReceiver receiver;
  proto::FrameView frame = {0U, 0U, 0U, 0U, 0UL, 0UL, NULL};
  proto::ReceiveResult result = proto::ReceiveResult::NONE;
  for (size_t i = 0U; i < g_serial_tx_length; ++i) {
    result = receiver.push(g_serial_tx[i], frame);
  }
  assert(result == proto::ReceiveResult::FRAME_READY);
  assert(frame.type == proto::TYPE_TERRAIN_TELEMETRY);
  assert(frame.session_id == 0x12345678UL);
  assert(frame.payload_length == proto::TERRAIN_TELEMETRY_PAYLOAD_SIZE);
  assert(proto::readU16(frame.payload) == g_distance_mm);
  assert(frame.payload[2U] == 1U);
  assert(frame.payload[18U] == 1U);
  assert(frame.payload[19U] == 0xFFU);

  g_serial_tx_length = 0U;
  sendGpsTelemetry(g_now_ms);
  proto::FrameReceiver gps_receiver;
  proto::FrameView gps_frame = {0U, 0U, 0U, 0U, 0UL, 0UL, NULL};
  proto::ReceiveResult gps_result = proto::ReceiveResult::NONE;
  for (size_t i = 0U; i < g_serial_tx_length; ++i) {
    gps_result = gps_receiver.push(g_serial_tx[i], gps_frame);
  }
  assert(gps_result == proto::ReceiveResult::FRAME_READY);
  assert(gps_frame.type == proto::TYPE_GPS_TELEMETRY);
  assert(gps_frame.session_id == 0x12345678UL);
  assert(gps_frame.payload_length == proto::GPS_TELEMETRY_PAYLOAD_SIZE);
  assert(proto::readU32(gps_frame.payload + 0U) == 0UL);
  assert(proto::readU32(gps_frame.payload + 4U) == 0UL);
  assert(proto::readU32(gps_frame.payload + 8U) == 0UL);
  assert(gps_frame.payload[12U] == 0U);
  assert(gps_frame.payload[13U] == 0U);

  g_now_ms = g_last_session_activity_ms +
      cfg::SESSION_LOSS_TIMEOUT_MS + 1UL;
  enforceHostSessionTimeout(g_now_ms);
  assert(!g_session_active);
  assert(!g_session_offer_active);
  assert(g_session_id == 0UL);

  printf("firmware terrain session/telemetry tests: OK\n");
  return 0;
}
