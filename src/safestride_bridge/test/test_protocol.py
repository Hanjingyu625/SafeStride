"""Pure-Python unit tests for the SafeStride serial protocol."""

import struct
import unittest

from safestride_bridge.protocol import (
    BOARD_ROLE_DRIVE,
    COMMAND_STRUCT,
    HEADER_STRUCT,
    TELEMETRY_STRUCT,
    TERRAIN_TELEMETRY_STRUCT,
    CobsDecodeError,
    CommandPayload,
    CrcMismatchError,
    Frame,
    FrameDecodeError,
    FrameParser,
    HelloPayload,
    PacketType,
    PayloadDecodeError,
    PROTOCOL_VERSION,
    SessionStartPayload,
    TelemetryPayload,
    TerrainTelemetryPayload,
    cobs_decode,
    cobs_encode,
    crc16_ccitt_false,
    decode_frame,
    sequence_is_newer,
)


class TestCrc(unittest.TestCase):

    def test_known_ccitt_false_vector(self):
        self.assertEqual(crc16_ccitt_false(b'123456789'), 0x29B1)


class TestSequence(unittest.TestCase):

    def test_uint16_half_range_comparison(self):
        self.assertTrue(sequence_is_newer(11, 10))
        self.assertFalse(sequence_is_newer(10, 10))
        self.assertFalse(sequence_is_newer(9, 10))
        self.assertTrue(sequence_is_newer(0, 0xFFFF))
        self.assertTrue(sequence_is_newer(5, 0xFFFA))
        self.assertFalse(sequence_is_newer(0xFFFA, 5))
        self.assertFalse(sequence_is_newer(0x8000, 0))


class TestCobs(unittest.TestCase):

    def test_round_trip_edge_cases(self):
        vectors = [
            b'',
            b'\x00',
            b'\x00\x00',
            b'plain bytes',
            b'\x11\x00\x22\x00\x33',
            bytes(range(256)),
            b'\xff' * 300,
        ]
        for value in vectors:
            with self.subTest(length=len(value), value=value[:8]):
                encoded = cobs_encode(value)
                self.assertNotIn(0, encoded)
                self.assertEqual(cobs_decode(encoded), value)

    def test_decode_rejects_empty_and_zero(self):
        with self.assertRaises(CobsDecodeError):
            cobs_decode(b'')
        with self.assertRaises(CobsDecodeError):
            cobs_decode(b'\x01\x00')

    def test_decode_rejects_truncated_block(self):
        with self.assertRaises(CobsDecodeError):
            cobs_decode(b'\x04\x11\x22')


class TestFrame(unittest.TestCase):

    def test_exact_header_layout_and_round_trip(self):
        frame = Frame(
            packet_type=PacketType.COMMAND,
            flags=0,
            reserved=0,
            sequence=0x1234,
            session_id=0x89ABCDEF,
            timestamp_ms=0x10203040,
            payload=b'\x00\xaa\x55',
        )
        raw = frame.raw_bytes()
        self.assertEqual(
            raw[:HEADER_STRUCT.size],
            struct.pack(
                '<BBBBHHII',
                PROTOCOL_VERSION,
                0x10,
                0,
                0,
                0x1234,
                3,
                0x89ABCDEF,
                0x10203040,
            ),
        )
        self.assertEqual(decode_frame(frame.encode()), frame)
        self.assertEqual(decode_frame(frame.encode()[:-1]), frame)

    def test_nonzero_flags_are_rejected(self):
        frame = Frame(
            packet_type=PacketType.HELLO,
            sequence=1,
            session_id=0,
            timestamp_ms=10,
            payload=HelloPayload(3, 7, BOARD_ROLE_DRIVE).pack(),
        )
        raw = bytearray(frame.raw_bytes())
        raw[2] = 1
        from safestride_bridge.protocol import CRC_STRUCT
        raw[-2:] = CRC_STRUCT.pack(crc16_ccitt_false(raw[:-2]))
        with self.assertRaises(FrameDecodeError):
            decode_frame(cobs_encode(bytes(raw)))

    def test_crc_corruption_is_rejected(self):
        frame = Frame(
            packet_type=PacketType.HELLO,
            sequence=1,
            session_id=0,
            timestamp_ms=10,
            payload=HelloPayload(3, 7, BOARD_ROLE_DRIVE).pack(),
        )
        raw = bytearray(frame.raw_bytes())
        raw[HEADER_STRUCT.size] ^= 0x80
        encoded = cobs_encode(bytes(raw))
        with self.assertRaises(CrcMismatchError):
            decode_frame(encoded)

    def test_payload_length_mismatch_is_rejected_after_valid_crc(self):
        frame = Frame(
            packet_type=PacketType.HELLO,
            sequence=1,
            session_id=0,
            timestamp_ms=10,
            payload=b'1234',
        )
        raw = bytearray(frame.raw_bytes())
        raw[6:8] = struct.pack('<H', 99)
        from safestride_bridge.protocol import CRC_STRUCT
        raw[-2:] = CRC_STRUCT.pack(crc16_ccitt_false(raw[:-2]))
        with self.assertRaises(FrameDecodeError):
            decode_frame(cobs_encode(bytes(raw)))

    def test_unsupported_version_is_rejected(self):
        frame = Frame(
            version=1,
            packet_type=PacketType.HELLO,
            sequence=0,
            session_id=0,
            timestamp_ms=0,
        )
        with self.assertRaises(FrameDecodeError):
            decode_frame(frame.encode())

    def test_nonzero_reserved_header_is_rejected(self):
        frame = Frame(
            reserved=1,
            packet_type=PacketType.HELLO,
            sequence=0,
            session_id=0,
            timestamp_ms=0,
        )
        with self.assertRaises(FrameDecodeError):
            decode_frame(frame.encode())

    def test_decoded_frame_over_128_bytes_is_rejected(self):
        frame = Frame(
            packet_type=PacketType.TELEMETRY,
            sequence=0,
            session_id=1,
            timestamp_ms=0,
            payload=b'x' * 111,
        )
        self.assertEqual(len(frame.raw_bytes()), 129)
        with self.assertRaises(FrameDecodeError):
            decode_frame(frame.encode())


class TestPayloads(unittest.TestCase):

    def test_hello(self):
        payload = HelloPayload(
            0xDEADBEEF, 0x01020304, BOARD_ROLE_DRIVE
        )
        self.assertEqual(HelloPayload.unpack(payload.pack()), payload)
        self.assertEqual(len(payload.pack()), 16)

    def test_session_start(self):
        payload = SessionStartPayload(0xDEADBEEF, BOARD_ROLE_DRIVE)
        self.assertEqual(SessionStartPayload.unpack(payload.pack()), payload)
        self.assertEqual(len(payload.pack()), 12)

    def test_command_exact_layout(self):
        payload = CommandPayload(-12345, 150, 1)
        self.assertEqual(
            payload.pack(),
            struct.pack('<iHBB', -12345, 150, 1, 0),
        )
        self.assertEqual(CommandPayload.unpack(payload.pack()), payload)
        self.assertEqual(len(payload.pack()), COMMAND_STRUCT.size)

    def test_telemetry_exact_layout(self):
        payload = TelemetryPayload(
            hall_left_pulses=-123456,
            hall_right_pulses=789012,
            velocity_left_mrad_s=-2000,
            velocity_right_mrad_s=3000,
            range_left_mm=450,
            range_right_mm=650,
            battery_mv=12100,
            current_left_ma=-350,
            current_right_ma=420,
            status_bits=0x0207,
            fault_bits=0,
            last_command_sequence=65535,
            pressure_left_raw=321,
            pressure_right_raw=654,
            pressure_left_filtered=300,
            pressure_right_filtered=600,
            pressure_flags=0x07,
            pressure_alert=1,
        )
        packed = payload.pack()
        self.assertEqual(len(packed), TELEMETRY_STRUCT.size)
        self.assertEqual(
            packed,
            struct.pack(
                '<iiiiHHHhhHHHHHHHBB',
                -123456,
                789012,
                -2000,
                3000,
                450,
                650,
                12100,
                -350,
                420,
                0x0207,
                0,
                65535,
                321,
                654,
                300,
                600,
                0x07,
                1,
            ),
        )
        self.assertEqual(TelemetryPayload.unpack(packed), payload)

    def test_terrain_telemetry_exact_layout(self):
        payload = TerrainTelemetryPayload(
            tof_distance_mm=725,
            tof_valid=1,
            tof_alert=2,
            tof_filtered_mm=710,
            tof_reference_mm=500,
            tof_error_mm=210,
            tof_change_mm=-15,
            mpu_accel_x_mg=10,
            mpu_accel_y_mg=-20,
            mpu_accel_z_mg=1000,
            mpu_gyro_x_mrad_s=31,
            mpu_gyro_y_mrad_s=-42,
            mpu_gyro_z_mrad_s=53,
            mpu_roll_mrad=-120,
            mpu_pitch_mrad=340,
            mpu_valid=1,
            fault_bits=0,
        )
        self.assertEqual(
            payload.pack(),
            struct.pack(
                '<HBBHHhhhhhhhhhhBH14x',
                725, 1, 2, 710, 500, 210, -15,
                10, -20, 1000, 31, -42, 53, -120, 340, 1, 0,
            ),
        )
        self.assertEqual(
            len(payload.pack()), TERRAIN_TELEMETRY_STRUCT.size
        )
        self.assertEqual(
            TerrainTelemetryPayload.unpack(payload.pack()), payload
        )

    def test_wrong_payload_size_is_rejected(self):
        with self.assertRaises(PayloadDecodeError):
            HelloPayload.unpack(b'\x00')
        with self.assertRaises(PayloadDecodeError):
            CommandPayload.unpack(b'\x00' * (COMMAND_STRUCT.size - 1))
        with self.assertRaises(PayloadDecodeError):
            TelemetryPayload.unpack(b'')
        with self.assertRaises(PayloadDecodeError):
            TerrainTelemetryPayload.unpack(b'\x00' * 13)

    def test_command_rejects_invalid_enable_and_reserved(self):
        with self.assertRaises(ValueError):
            CommandPayload(0, 100, 2).pack()
        with self.assertRaises(PayloadDecodeError):
            CommandPayload.unpack(
                struct.pack('<iHBB', 0, 100, 2, 0)
            )
        with self.assertRaises(PayloadDecodeError):
            CommandPayload.unpack(
                struct.pack('<iHBB', 0, 100, 1, 1)
            )

    def test_telemetry_rejects_invalid_pressure_fields(self):
        values = [0] * 18
        values[16] = 0x08
        with self.assertRaises(PayloadDecodeError):
            TelemetryPayload.unpack(TELEMETRY_STRUCT.pack(*values))

        values[16] = 0
        values[17] = 3
        with self.assertRaises(PayloadDecodeError):
            TelemetryPayload.unpack(TELEMETRY_STRUCT.pack(*values))

        with self.assertRaises(ValueError):
            TelemetryPayload(*values).pack()

    def test_terrain_telemetry_rejects_invalid_alert_fields(self):
        values = [0] * 17
        values[2] = 6
        with self.assertRaises(PayloadDecodeError):
            TerrainTelemetryPayload.unpack(
                TERRAIN_TELEMETRY_STRUCT.pack(*values)
            )
        with self.assertRaises(ValueError):
            TerrainTelemetryPayload(*values).pack()

    def test_terrain_telemetry_keeps_legacy_gps_bytes_zero(self):
        values = [0] * 17
        packed = TerrainTelemetryPayload(*values).pack()
        self.assertEqual(len(packed), 45)
        self.assertEqual(packed[31:], bytes(14))


class TestFrameParser(unittest.TestCase):

    @staticmethod
    def make_frame(sequence, payload=b'\x00payload'):
        return Frame(
            packet_type=PacketType.COMMAND,
            sequence=sequence,
            session_id=0x12345678,
            timestamp_ms=sequence,
            payload=payload,
        )

    def test_fragmented_and_back_to_back_frames(self):
        first = self.make_frame(1)
        second = self.make_frame(2, b'another')
        wire = first.encode() + second.encode()
        parser = FrameParser()
        result = []
        for byte in wire:
            result.extend(parser.feed(bytes((byte,))))
        self.assertEqual(result, [first, second])
        self.assertEqual(parser.frames_received, 2)
        self.assertEqual(parser.crc_error_count, 0)
        self.assertEqual(parser.frame_error_count, 0)

    def test_noise_packet_does_not_prevent_recovery(self):
        expected = self.make_frame(5)
        parser = FrameParser()
        result = parser.feed(b'\x03\xff\x00' + expected.encode())
        self.assertEqual(result, [expected])
        self.assertEqual(parser.frame_error_count, 1)

    def test_crc_error_is_counted_separately(self):
        frame = self.make_frame(7)
        raw = bytearray(frame.raw_bytes())
        raw[-1] ^= 1
        parser = FrameParser()
        self.assertEqual(parser.feed(cobs_encode(raw) + b'\x00'), [])
        self.assertEqual(parser.crc_error_count, 1)
        self.assertEqual(parser.frame_error_count, 0)

    def test_oversized_packet_is_dropped_until_delimiter(self):
        expected = self.make_frame(9)
        parser = FrameParser(max_encoded_size=32)
        result = parser.feed(b'\x01' * 100 + b'\x00' + expected.encode())
        self.assertEqual(result, [expected])
        self.assertEqual(parser.frame_error_count, 1)

    def test_empty_delimiters_are_ignored(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b'\x00\x00\x00'), [])
        self.assertEqual(parser.frame_error_count, 0)

    def test_version_mismatch_is_reported_separately(self):
        parser = FrameParser()
        old_frame = Frame(
            version=PROTOCOL_VERSION - 1,
            packet_type=PacketType.HELLO,
            sequence=1,
            session_id=0,
            timestamp_ms=0,
        )
        self.assertEqual(parser.feed(old_frame.encode()), [])
        self.assertEqual(parser.version_error_count, 1)
        self.assertEqual(
            parser.last_unsupported_version, PROTOCOL_VERSION - 1
        )
        self.assertEqual(parser.frame_error_count, 0)


if __name__ == '__main__':
    unittest.main()
