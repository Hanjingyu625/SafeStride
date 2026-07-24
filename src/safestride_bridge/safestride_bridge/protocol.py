"""Wire protocol shared by the SafeStride ROS bridge and controller firmware.

Every packet is COBS encoded and terminated by a zero byte.  The decoded
packet consists of a fixed little-endian header, a variable payload and a
CRC16-CCITT-FALSE.  This module intentionally has no ROS dependencies so it
can be unit-tested on any development machine.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import ClassVar, Iterable, List


PROTOCOL_VERSION = 1
FRAME_DELIMITER = 0x00
MAX_RAW_FRAME_SIZE = 128

HEADER_STRUCT = struct.Struct('<BBBBHHII')
CRC_STRUCT = struct.Struct('<H')

HELLO_STRUCT = struct.Struct('<II')
SESSION_START_STRUCT = struct.Struct('<I')
COMMAND_STRUCT = struct.Struct('<iiHBB')
TELEMETRY_STRUCT = struct.Struct('<iiiiHHHhhHHH')


class ProtocolError(ValueError):
    """Base class for malformed wire data."""


class CobsDecodeError(ProtocolError):
    """Raised when an encoded COBS packet is malformed."""


class FrameDecodeError(ProtocolError):
    """Raised when a decoded packet violates the frame format."""


class CrcMismatchError(FrameDecodeError):
    """Raised when a packet CRC does not match its contents."""


class PayloadDecodeError(ProtocolError):
    """Raised when a typed payload has an unexpected size or value."""


class PacketType(IntEnum):
    """SafeStride protocol packet type identifiers."""

    HELLO = 0x01
    SESSION_START = 0x02
    COMMAND = 0x10
    TELEMETRY = 0x20


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC16-CCITT-FALSE (poly 0x1021, init 0xffff)."""

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def sequence_is_newer(candidate: int, reference: int) -> bool:
    """Compare uint16 sequences using their half-range wraparound rule."""

    candidate = _u16('candidate', candidate)
    reference = _u16('reference', reference)
    delta = (candidate - reference) & 0xFFFF
    return 0 < delta < 0x8000


def cobs_encode(data: bytes) -> bytes:
    """Encode *data* using Consistent Overhead Byte Stuffing."""

    output = bytearray((0,))
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """Decode one COBS packet without its trailing delimiter."""

    if not data:
        raise CobsDecodeError('empty COBS packet')
    if 0 in data:
        raise CobsDecodeError('COBS packet contains a zero byte')

    output = bytearray()
    index = 0
    size = len(data)
    while index < size:
        code = data[index]
        index += 1
        block_end = index + code - 1
        if block_end > size:
            raise CobsDecodeError('COBS code exceeds packet length')
        output.extend(data[index:block_end])
        index = block_end
        if code != 0xFF and index < size:
            output.append(0)
    return bytes(output)


def _u8(name: str, value: int) -> int:
    if not 0 <= int(value) <= 0xFF:
        raise ValueError(f'{name} must fit uint8')
    return int(value)


def _u16(name: str, value: int) -> int:
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f'{name} must fit uint16')
    return int(value)


def _u32(name: str, value: int) -> int:
    if not 0 <= int(value) <= 0xFFFFFFFF:
        raise ValueError(f'{name} must fit uint32')
    return int(value)


@dataclass(frozen=True)
class Frame:
    """A decoded protocol frame."""

    packet_type: int
    sequence: int
    session_id: int
    timestamp_ms: int
    payload: bytes = b''
    flags: int = 0
    reserved: int = 0
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        _u8('version', self.version)
        _u8('packet_type', self.packet_type)
        _u8('flags', self.flags)
        _u8('reserved', self.reserved)
        _u16('sequence', self.sequence)
        _u32('session_id', self.session_id)
        _u32('timestamp_ms', self.timestamp_ms)
        if not isinstance(self.payload, bytes):
            object.__setattr__(self, 'payload', bytes(self.payload))
        _u16('payload length', len(self.payload))

    @property
    def type(self) -> int:
        """Compatibility alias matching the header field name."""

        return self.packet_type

    def raw_bytes(self) -> bytes:
        """Serialize the decoded portion, including its CRC."""

        header = HEADER_STRUCT.pack(
            self.version,
            self.packet_type,
            self.flags,
            self.reserved,
            self.sequence,
            len(self.payload),
            self.session_id,
            self.timestamp_ms,
        )
        checked = header + self.payload
        return checked + CRC_STRUCT.pack(crc16_ccitt_false(checked))

    def encode(self) -> bytes:
        """Serialize, COBS encode and append the 0x00 delimiter."""

        return cobs_encode(self.raw_bytes()) + bytes((FRAME_DELIMITER,))


def encode_frame(frame: Frame) -> bytes:
    """Encode a :class:`Frame` for the serial wire."""

    return frame.encode()


def decode_frame(encoded: bytes) -> Frame:
    """Decode one COBS frame.

    A single trailing delimiter is accepted for convenience.  Embedded or
    additional delimiters are rejected by the COBS decoder.
    """

    if encoded.endswith(bytes((FRAME_DELIMITER,))):
        encoded = encoded[:-1]
    raw = cobs_decode(encoded)

    minimum_size = HEADER_STRUCT.size + CRC_STRUCT.size
    if len(raw) < minimum_size:
        raise FrameDecodeError('frame is shorter than header plus CRC')
    if len(raw) > MAX_RAW_FRAME_SIZE:
        raise FrameDecodeError(
            f'decoded frame exceeds {MAX_RAW_FRAME_SIZE} bytes'
        )

    checked = raw[:-CRC_STRUCT.size]
    received_crc = CRC_STRUCT.unpack(raw[-CRC_STRUCT.size:])[0]
    expected_crc = crc16_ccitt_false(checked)
    if received_crc != expected_crc:
        raise CrcMismatchError(
            f'CRC mismatch: received 0x{received_crc:04x}, '
            f'expected 0x{expected_crc:04x}'
        )

    (
        version,
        packet_type,
        flags,
        reserved,
        sequence,
        payload_length,
        session_id,
        timestamp_ms,
    ) = HEADER_STRUCT.unpack(raw[:HEADER_STRUCT.size])

    if version != PROTOCOL_VERSION:
        raise FrameDecodeError(
            f'unsupported protocol version {version}; '
            f'expected {PROTOCOL_VERSION}'
        )
    if flags != 0:
        raise FrameDecodeError('unsupported nonzero header flags')
    if reserved != 0:
        raise FrameDecodeError('header reserved field must be zero')
    actual_payload_length = len(raw) - minimum_size
    if payload_length != actual_payload_length:
        raise FrameDecodeError(
            f'payload length is {payload_length}, '
            f'but frame contains {actual_payload_length}'
        )

    return Frame(
        version=version,
        packet_type=packet_type,
        flags=flags,
        reserved=reserved,
        sequence=sequence,
        session_id=session_id,
        timestamp_ms=timestamp_ms,
        payload=raw[HEADER_STRUCT.size:-CRC_STRUCT.size],
    )


class FrameParser:
    """Incrementally split and decode a serial byte stream."""

    def __init__(self, max_encoded_size: int = 160) -> None:
        if max_encoded_size < 2:
            raise ValueError('max_encoded_size must be at least 2')
        self.max_encoded_size = int(max_encoded_size)
        self._buffer = bytearray()
        self._dropping_oversize = False
        self.frames_received = 0
        self.crc_error_count = 0
        self.frame_error_count = 0

    def reset(self, clear_counters: bool = False) -> None:
        """Discard a partial packet, optionally resetting statistics."""

        self._buffer.clear()
        self._dropping_oversize = False
        if clear_counters:
            self.frames_received = 0
            self.crc_error_count = 0
            self.frame_error_count = 0

    def feed(self, data: Iterable[int]) -> List[Frame]:
        """Consume bytes and return every complete, valid frame."""

        frames: List[Frame] = []
        for value in data:
            byte = int(value)
            if not 0 <= byte <= 0xFF:
                raise ValueError('stream elements must fit uint8')

            if byte == FRAME_DELIMITER:
                if self._dropping_oversize:
                    self._dropping_oversize = False
                    self._buffer.clear()
                    continue
                if not self._buffer:
                    continue
                try:
                    frame = decode_frame(bytes(self._buffer))
                except CrcMismatchError:
                    self.crc_error_count += 1
                except ProtocolError:
                    self.frame_error_count += 1
                else:
                    self.frames_received += 1
                    frames.append(frame)
                finally:
                    self._buffer.clear()
                continue

            if self._dropping_oversize:
                continue
            if len(self._buffer) >= self.max_encoded_size:
                self.frame_error_count += 1
                self._buffer.clear()
                self._dropping_oversize = True
                continue
            self._buffer.append(byte)
        return frames


def _require_size(payload_name: str, data: bytes, expected: int) -> None:
    if len(data) != expected:
        raise PayloadDecodeError(
            f'{payload_name} payload must be {expected} bytes, '
            f'got {len(data)}'
        )


@dataclass(frozen=True)
class HelloPayload:
    """HELLO payload sent by firmware while waiting for a session."""

    boot_id: int
    capabilities: int
    TYPE: ClassVar[PacketType] = PacketType.HELLO

    def pack(self) -> bytes:
        return HELLO_STRUCT.pack(
            _u32('boot_id', self.boot_id),
            _u32('capabilities', self.capabilities),
        )

    @classmethod
    def unpack(cls, data: bytes) -> 'HelloPayload':
        _require_size('HELLO', data, HELLO_STRUCT.size)
        return cls(*HELLO_STRUCT.unpack(data))


@dataclass(frozen=True)
class SessionStartPayload:
    """SESSION_START payload sent by the host after a HELLO."""

    expected_boot_id: int
    TYPE: ClassVar[PacketType] = PacketType.SESSION_START

    def pack(self) -> bytes:
        return SESSION_START_STRUCT.pack(
            _u32('expected_boot_id', self.expected_boot_id)
        )

    @classmethod
    def unpack(cls, data: bytes) -> 'SessionStartPayload':
        _require_size('SESSION_START', data, SESSION_START_STRUCT.size)
        return cls(*SESSION_START_STRUCT.unpack(data))


@dataclass(frozen=True)
class CommandPayload:
    """COMMAND payload containing signed wheel targets in mrad/s."""

    left_mrad_s: int
    right_mrad_s: int
    ttl_ms: int
    enable: int
    reserved: int = 0
    TYPE: ClassVar[PacketType] = PacketType.COMMAND

    def pack(self) -> bytes:
        if self.enable not in (0, 1):
            raise ValueError('enable must be 0 or 1')
        if self.reserved != 0:
            raise ValueError('COMMAND reserved field must be zero')
        return COMMAND_STRUCT.pack(
            int(self.left_mrad_s),
            int(self.right_mrad_s),
            _u16('ttl_ms', self.ttl_ms),
            _u8('enable', self.enable),
            _u8('reserved', self.reserved),
        )

    @classmethod
    def unpack(cls, data: bytes) -> 'CommandPayload':
        _require_size('COMMAND', data, COMMAND_STRUCT.size)
        payload = cls(*COMMAND_STRUCT.unpack(data))
        if payload.enable not in (0, 1):
            raise PayloadDecodeError('COMMAND enable must be 0 or 1')
        if payload.reserved != 0:
            raise PayloadDecodeError('COMMAND reserved field must be zero')
        return payload


@dataclass(frozen=True)
class TelemetryPayload:
    """TELEMETRY payload returned by the motor controller."""

    encoder_left: int
    encoder_right: int
    velocity_left_mrad_s: int
    velocity_right_mrad_s: int
    range_left_mm: int
    range_right_mm: int
    battery_mv: int
    current_left_ma: int
    current_right_ma: int
    status_bits: int
    fault_bits: int
    last_command_sequence: int
    TYPE: ClassVar[PacketType] = PacketType.TELEMETRY

    def pack(self) -> bytes:
        return TELEMETRY_STRUCT.pack(
            int(self.encoder_left),
            int(self.encoder_right),
            int(self.velocity_left_mrad_s),
            int(self.velocity_right_mrad_s),
            _u16('range_left_mm', self.range_left_mm),
            _u16('range_right_mm', self.range_right_mm),
            _u16('battery_mv', self.battery_mv),
            int(self.current_left_ma),
            int(self.current_right_ma),
            _u16('status_bits', self.status_bits),
            _u16('fault_bits', self.fault_bits),
            _u16('last_command_sequence', self.last_command_sequence),
        )

    @classmethod
    def unpack(cls, data: bytes) -> 'TelemetryPayload':
        _require_size('TELEMETRY', data, TELEMETRY_STRUCT.size)
        return cls(*TELEMETRY_STRUCT.unpack(data))
