#include "protocol.h"

namespace safestride_protocol {
namespace {

size_t cobsEncode(
    const uint8_t* input,
    size_t length,
    uint8_t* output,
    size_t capacity) {
  if (capacity == 0U) {
    return 0U;
  }
  size_t read_index = 0U;
  size_t write_index = 1U;
  size_t code_index = 0U;
  uint8_t code = 1U;
  while (read_index < length) {
    if (input[read_index] == 0U) {
      if (code_index >= capacity || write_index >= capacity) {
        return 0U;
      }
      output[code_index] = code;
      code_index = write_index++;
      code = 1U;
      ++read_index;
    } else {
      if (write_index >= capacity) {
        return 0U;
      }
      output[write_index++] = input[read_index++];
      ++code;
      if (code == 0xFFU) {
        if (code_index >= capacity || write_index >= capacity) {
          return 0U;
        }
        output[code_index] = code;
        code_index = write_index++;
        code = 1U;
      }
    }
  }
  if (code_index >= capacity) {
    return 0U;
  }
  output[code_index] = code;
  return write_index;
}

size_t cobsDecode(
    const uint8_t* input,
    size_t length,
    uint8_t* output,
    size_t capacity) {
  if (length == 0U) {
    return 0U;
  }
  size_t read_index = 0U;
  size_t write_index = 0U;
  while (read_index < length) {
    const uint8_t code = input[read_index++];
    if (code == 0U) {
      return 0U;
    }
    const size_t copy_count = static_cast<size_t>(code - 1U);
    if (read_index + copy_count > length ||
        write_index + copy_count > capacity) {
      return 0U;
    }
    for (size_t i = 0U; i < copy_count; ++i) {
      output[write_index++] = input[read_index++];
    }
    if (code != 0xFFU && read_index < length) {
      if (write_index >= capacity) {
        return 0U;
      }
      output[write_index++] = 0U;
    }
  }
  return write_index;
}

}  // namespace

FrameReceiver::FrameReceiver()
    : encoded_length_(0U), dropping_oversize_(false) {}

void FrameReceiver::reset() {
  encoded_length_ = 0U;
  dropping_oversize_ = false;
}

ReceiveResult FrameReceiver::push(uint8_t byte, FrameView& frame) {
  if (byte != 0U) {
    if (dropping_oversize_) {
      return ReceiveResult::NONE;
    }
    if (encoded_length_ >= MAX_ENCODED_FRAME_SIZE) {
      encoded_length_ = 0U;
      dropping_oversize_ = true;
      return ReceiveResult::FRAME_ERROR;
    }
    encoded_[encoded_length_++] = byte;
    return ReceiveResult::NONE;
  }
  if (dropping_oversize_) {
    reset();
    return ReceiveResult::NONE;
  }
  if (encoded_length_ == 0U) {
    return ReceiveResult::NONE;
  }
  const ReceiveResult result = decode(frame);
  encoded_length_ = 0U;
  return result;
}

ReceiveResult FrameReceiver::decode(FrameView& frame) {
  const size_t raw_length = cobsDecode(
      encoded_, encoded_length_, raw_, sizeof(raw_));
  if (raw_length < HEADER_SIZE + CRC_SIZE ||
      raw_length > MAX_RAW_FRAME_SIZE) {
    return ReceiveResult::FRAME_ERROR;
  }
  const uint16_t received_crc = readU16(raw_ + raw_length - CRC_SIZE);
  const uint16_t expected_crc =
      crc16CcittFalse(raw_, raw_length - CRC_SIZE);
  if (received_crc != expected_crc) {
    return ReceiveResult::CRC_ERROR;
  }
  if (raw_[0] != VERSION || raw_[2] != 0U || raw_[3] != 0U) {
    return ReceiveResult::FRAME_ERROR;
  }
  const uint16_t payload_length = readU16(raw_ + 6U);
  if (HEADER_SIZE + payload_length + CRC_SIZE != raw_length) {
    return ReceiveResult::FRAME_ERROR;
  }
  frame.type = raw_[1];
  frame.flags = raw_[2];
  frame.sequence = readU16(raw_ + 4U);
  frame.payload_length = payload_length;
  frame.session_id = readU32(raw_ + 8U);
  frame.timestamp_ms = readU32(raw_ + 12U);
  frame.payload = raw_ + HEADER_SIZE;
  return ReceiveResult::FRAME_READY;
}

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
          ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
          : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

uint16_t readU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

int16_t readI16(const uint8_t* data) {
  return static_cast<int16_t>(readU16(data));
}

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

void writeU16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFU);
  data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void writeI16(uint8_t* data, int16_t value) {
  writeU16(data, static_cast<uint16_t>(value));
}

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFUL);
  data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFUL);
  data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFUL);
  data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFUL);
}

void writeI32(uint8_t* data, int32_t value) {
  writeU32(data, static_cast<uint32_t>(value));
}

bool sendFrame(
    Stream& stream,
    uint8_t type,
    uint16_t sequence,
    uint32_t session_id,
    uint32_t timestamp_ms,
    const uint8_t* payload,
    uint16_t payload_length) {
  if (HEADER_SIZE + payload_length + CRC_SIZE > MAX_RAW_FRAME_SIZE ||
      (payload_length > 0U && payload == NULL)) {
    return false;
  }
  static uint8_t raw[MAX_RAW_FRAME_SIZE];
  static uint8_t encoded[MAX_ENCODED_FRAME_SIZE];
  raw[0] = VERSION;
  raw[1] = type;
  raw[2] = 0U;
  raw[3] = 0U;
  writeU16(raw + 4U, sequence);
  writeU16(raw + 6U, payload_length);
  writeU32(raw + 8U, session_id);
  writeU32(raw + 12U, timestamp_ms);
  for (uint16_t i = 0U; i < payload_length; ++i) {
    raw[HEADER_SIZE + i] = payload[i];
  }
  const size_t checked_length = HEADER_SIZE + payload_length;
  writeU16(raw + checked_length, crc16CcittFalse(raw, checked_length));
  const size_t encoded_length = cobsEncode(
      raw, checked_length + CRC_SIZE, encoded, sizeof(encoded));
  if (encoded_length == 0U) {
    return false;
  }
  return stream.write(encoded, encoded_length) == encoded_length &&
         stream.write(static_cast<uint8_t>(0U)) == 1U;
}

}  // namespace safestride_protocol
