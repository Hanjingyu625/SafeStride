// Pi와 MCU가 공유하는 바이너리 통신 규약. 상세 바이트 배치는 저장소 루트 PROTOCOL.md 참조.
// 버전/스키마/릴리스 및 payload 크기는 Pi와 두 보드가 맞아야 한다.

#pragma once

#include <Arduino.h>

namespace safestride_protocol {

constexpr uint8_t VERSION = 6U;
constexpr uint16_t SCHEMA_ID = 0x0601U;
constexpr uint32_t FIRMWARE_RELEASE_ID = 20260908UL;
constexpr uint8_t BOARD_ROLE_DRIVE = 1U;
constexpr uint8_t BOARD_ROLE_TERRAIN = 2U;
constexpr uint8_t TYPE_HELLO = 0x01U;
constexpr uint8_t TYPE_SESSION_START = 0x02U;
constexpr uint8_t TYPE_COMMAND = 0x10U;
constexpr uint8_t TYPE_TELEMETRY = 0x20U;
constexpr uint8_t TYPE_TERRAIN_TELEMETRY = 0x21U;

constexpr size_t HEADER_SIZE = 16U;
constexpr size_t CRC_SIZE = 2U;
constexpr size_t MAX_RAW_FRAME_SIZE = 128U;
constexpr size_t MAX_ENCODED_FRAME_SIZE = 160U;

constexpr size_t HELLO_PAYLOAD_SIZE = 16U;
constexpr size_t SESSION_START_PAYLOAD_SIZE = 12U;
constexpr size_t COMMAND_PAYLOAD_SIZE = 12U;
constexpr size_t TELEMETRY_PAYLOAD_SIZE = 54U;
constexpr size_t TERRAIN_TELEMETRY_PAYLOAD_SIZE = 45U;

enum class ReceiveResult : uint8_t {
  NONE = 0,
  FRAME_READY,
  FRAME_ERROR,
  CRC_ERROR,
};

// payload는 수신기 내부 버퍼를 가리키는 뷰다. 다음 프레임 해석 전에 처리하거나 필요한 값을 복사한다.
struct FrameView {
  uint8_t type;
  uint8_t flags;
  uint16_t sequence;
  uint16_t payload_length;
  uint32_t session_id;
  uint32_t timestamp_ms;
  const uint8_t* payload;
};

// 고정 크기 버퍼를 사용한다. push()가 FRAME_READY일 때만 FrameView 내용을 사용한다.
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
