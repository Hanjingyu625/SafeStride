#!/usr/bin/env python3
"""Verify one SafeStride Uno link without starting ROS 2."""

from __future__ import annotations

import argparse
from pathlib import Path
import secrets
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_SOURCE = ROOT / 'src' / 'safestride_bridge'
sys.path.insert(0, str(BRIDGE_SOURCE))

try:
    import serial
except ImportError:  # pragma: no cover - depends on target image
    serial = None

from safestride_bridge.protocol import (  # noqa: E402
    BOARD_ROLE_DRIVE,
    BOARD_ROLE_TERRAIN,
    FIRMWARE_RELEASE_ID,
    PROTOCOL_SCHEMA_ID,
    PROTOCOL_VERSION,
    Frame,
    FrameParser,
    HelloPayload,
    PacketType,
    PayloadDecodeError,
    SessionStartPayload,
)


ROLES = {
    'drive': BOARD_ROLE_DRIVE,
    'terrain': BOARD_ROLE_TERRAIN,
}
REQUIRED_CAPABILITIES = {
    BOARD_ROLE_DRIVE: (1 << 0) | (1 << 4) | (1 << 6) | (1 << 7),
    BOARD_ROLE_TERRAIN: (1 << 8) | (1 << 9),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Probe SafeStride COBS/CRC serial handshake and telemetry'
    )
    parser.add_argument('--port', required=True)
    parser.add_argument('--role', choices=ROLES, required=True)
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=8.0)
    return parser.parse_args()


def compatible(hello: HelloPayload, expected_role: int) -> list[str]:
    errors = []
    if hello.board_role != expected_role:
        errors.append(
            f'board role {hello.board_role} != expected {expected_role}'
        )
    if hello.protocol_version != PROTOCOL_VERSION:
        errors.append(
            f'HELLO protocol {hello.protocol_version} != {PROTOCOL_VERSION}'
        )
    if hello.schema_id != PROTOCOL_SCHEMA_ID:
        errors.append(
            f'schema 0x{hello.schema_id:04x} != 0x{PROTOCOL_SCHEMA_ID:04x}'
        )
    if hello.firmware_release_id != FIRMWARE_RELEASE_ID:
        errors.append(
            f'firmware release {hello.firmware_release_id} != '
            f'{FIRMWARE_RELEASE_ID}'
        )
    missing_capabilities = (
        REQUIRED_CAPABILITIES[expected_role] & ~hello.capabilities
    )
    if missing_capabilities:
        errors.append(
            f'missing capabilities 0x{missing_capabilities:08x}'
        )
    return errors


def main() -> int:
    args = parse_args()
    if serial is None:
        print(
            'FAIL: pyserial is required: sudo apt install python3-serial',
            file=sys.stderr,
        )
        return 1
    if args.timeout <= 0.0:
        raise SystemExit('--timeout must be positive')
    expected_role = ROLES[args.role]
    parser = FrameParser()
    deadline = time.monotonic() + args.timeout

    try:
        port = serial.Serial(
            args.port,
            args.baudrate,
            timeout=0.05,
            write_timeout=0.5,
        )
    except (OSError, serial.SerialException) as error:
        print(f'FAIL: cannot open {args.port}: {error}', file=sys.stderr)
        return 2

    try:
        # Opening USB serial normally resets an Uno. Periodic HELLO frames make
        # a fixed reset delay unnecessary; consume bytes until the deadline.
        hello = None
        while time.monotonic() < deadline and hello is None:
            for frame in parser.feed(port.read(256)):
                if frame.packet_type != PacketType.HELLO:
                    continue
                try:
                    hello = HelloPayload.unpack(frame.payload)
                except PayloadDecodeError as error:
                    print(f'FAIL: malformed HELLO: {error}', file=sys.stderr)
                    return 3
                break
            if parser.last_unsupported_version is not None:
                print(
                    'FAIL: protocol version mismatch: Uno '
                    f'v{parser.last_unsupported_version}, '
                    f'probe v{PROTOCOL_VERSION}',
                    file=sys.stderr,
                )
                return 4

        if hello is None:
            print(
                f'FAIL: no HELLO from {args.port} at {args.baudrate} baud',
                file=sys.stderr,
            )
            return 5

        print(
            f'HELLO boot=0x{hello.boot_id:08x} '
            f'role={hello.board_role} capabilities=0x{hello.capabilities:08x} '
            f'protocol={hello.protocol_version} '
            f'schema=0x{hello.schema_id:04x} '
            f'release={hello.firmware_release_id}'
        )
        errors = compatible(hello, expected_role)
        if errors:
            print(f"FAIL: {'; '.join(errors)}", file=sys.stderr)
            return 6

        session_id = secrets.randbits(32) or 1
        session = Frame(
            packet_type=PacketType.SESSION_START,
            sequence=0,
            session_id=session_id,
            timestamp_ms=int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            payload=SessionStartPayload(
                hello.boot_id,
                expected_role,
            ).pack(),
        )
        port.write(session.encode())
        port.flush()

        expected_packet = (
            PacketType.TELEMETRY
            if expected_role == BOARD_ROLE_DRIVE
            else PacketType.TERRAIN_TELEMETRY
        )
        telemetry_deadline = time.monotonic() + args.timeout
        while time.monotonic() < telemetry_deadline:
            for frame in parser.feed(port.read(256)):
                if (
                    frame.packet_type == expected_packet
                    and frame.session_id == session_id
                ):
                    print(
                        f'PASS: bidirectional {args.role.upper()} link; '
                        f'telemetry_bytes={len(frame.payload)} '
                        f'crc_errors={parser.crc_error_count} '
                        f'frame_errors={parser.frame_error_count}'
                    )
                    return 0
        print(
            'FAIL: HELLO was compatible but no telemetry returned after '
            'SESSION_START',
            file=sys.stderr,
        )
        return 7
    finally:
        port.close()


if __name__ == '__main__':
    raise SystemExit(main())
