#pragma once

#include <Arduino.h>

namespace safestride_protocol {

constexpr uint8_t VERSION = 1U;
constexpr uint8_t TYPE_HELLO = 0x01U;
constexpr uint8_t TYPE_SESSION_START = 0x02U;
constexpr uint8_t TYPE_COMMAND = 0x10U;
constexpr uint8_t TYPE_TELEMETRY = 0x20U;

constexpr size_t HEADER_SIZE = 16U;
constexpr size_t CRC_SIZE = 2U;
constexpr size_t MAX_RAW_FRAME_SIZE = 128U;
constexpr size_t MAX_ENCODED_FRAME_SIZE = 160U;

constexpr size_t HELLO_PAYLOAD_SIZE = 8U;
constexpr size_t SESSION_START_PAYLOAD_SIZE = 4U;
constexpr size_t COMMAND_PAYLOAD_SIZE = 12U;
constexpr size_t TELEMETRY_PAYLOAD_SIZE = 32U;

enum class ReceiveResult : uint8_t {
  NONE = 0,
  FRAME_READY,
  FRAME_ERROR,
  CRC_ERROR,
};

struct FrameView {
  uint8_t type;
  uint8_t flags;
  uint16_t sequence;
  uint16_t payload_length;
  uint32_t session_id;
  uint32_t timestamp_ms;
  const uint8_t* payload;
};

class FrameReceiver {
 public:
  FrameReceiver();
  ReceiveResult push(uint8_t byte, FrameView& frame);
  void reset();

 private:
  uint8_t encoded_[MAX_ENCODED_FRAME_SIZE];
  uint8_t raw_[MAX_RAW_FRAME_SIZE];
  size_t encoded_length_;
  bool dropping_oversize_;

  ReceiveResult decode(FrameView& frame);
};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length);

uint16_t readU16(const uint8_t* data);
int16_t readI16(const uint8_t* data);
uint32_t readU32(const uint8_t* data);
int32_t readI32(const uint8_t* data);
void writeU16(uint8_t* data, uint16_t value);
void writeI16(uint8_t* data, int16_t value);
void writeU32(uint8_t* data, uint32_t value);
void writeI32(uint8_t* data, int32_t value);

bool sequenceIsNewer(uint16_t candidate, uint16_t previous);

// Serializes and writes one complete COBS-delimited frame.
bool sendFrame(
    Stream& stream,
    uint8_t type,
    uint16_t sequence,
    uint32_t session_id,
    uint32_t timestamp_ms,
    const uint8_t* payload,
    uint16_t payload_length);

}  // namespace safestride_protocol

