#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

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

int main() {
  namespace proto = safestride_protocol;

  static const uint8_t crc_vector[] = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(
      proto::crc16CcittFalse(crc_vector, sizeof(crc_vector)) ==
      0x29B1U);

  uint8_t command[proto::COMMAND_PAYLOAD_SIZE];
  proto::writeI32(command + 0U, -12345L);
  proto::writeU16(command + 4U, 150U);
  command[6] = 1U;
  command[7] = 0U;

  BufferStream stream;
  assert(proto::sendFrame(
      stream,
      proto::TYPE_COMMAND,
      0x1234U,
      0x89ABCDEFUL,
      0x10203040UL,
      command,
      sizeof(command)));

  // Protocol-v2 golden vector produced by
  // safestride_bridge.protocol.Frame.encode().
  static const uint8_t expected[] = {
      0x03, 0x02, 0x10, 0x01, 0x04, 0x34, 0x12, 0x08,
      0x0e, 0xef, 0xcd, 0xab, 0x89, 0x40, 0x30, 0x20,
      0x10, 0xc7, 0xcf, 0xff, 0xff, 0x96, 0x02, 0x01,
      0x03, 0x2f, 0x93, 0x00};
  assert(stream.length() == sizeof(expected));
  assert(memcmp(stream.data(), expected, sizeof(expected)) == 0);

  proto::FrameReceiver receiver;
  proto::FrameView frame = {
      0U, 0U, 0U, 0U, 0UL, 0UL, NULL};
  proto::ReceiveResult result = proto::ReceiveResult::NONE;
  for (size_t i = 0U; i < stream.length(); ++i) {
    result = receiver.push(stream.data()[i], frame);
  }
  assert(result == proto::ReceiveResult::FRAME_READY);
  assert(frame.type == proto::TYPE_COMMAND);
  assert(frame.sequence == 0x1234U);
  assert(frame.session_id == 0x89ABCDEFUL);
  assert(frame.timestamp_ms == 0x10203040UL);
  assert(frame.payload_length == proto::COMMAND_PAYLOAD_SIZE);
  assert(proto::readI32(frame.payload + 0U) == -12345L);
  assert(proto::readU16(frame.payload + 4U) == 150U);
  assert(frame.payload[6] == 1U);
  assert(frame.payload[7] == 0U);

  assert(proto::sequenceIsNewer(0U, 0xFFFFU));
  assert(!proto::sequenceIsNewer(10U, 10U));
  assert(!proto::sequenceIsNewer(0x8000U, 0U));
  assert(!proto::sequenceIsNewer(9U, 10U));

  printf("firmware protocol golden-vector test: OK\n");
  return 0;
}
