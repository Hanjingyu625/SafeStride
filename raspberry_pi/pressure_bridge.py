#!/usr/bin/env python3
import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports


LINE_PATTERN = re.compile(
    r"Left\s*:\s*(?P<left>\d+)\s+Right\s*:\s*(?P<right>\d+)"
)


def find_serial_port() -> str:
    candidates = [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
    ]

    for port in candidates:
        if Path(port).exists():
            return port

    detected = [item.device for item in list_ports.comports()]
    if detected:
        return detected[0]

    raise RuntimeError("No Arduino serial port found")


def write_status(path: Path, data: dict) -> None:
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temp_path.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read two FSR values from Arduino and block Pi motion commands when a handle is released."
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--left-threshold", type=int, default=50)
    parser.add_argument("--right-threshold", type=int, default=50)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--status-file",
        default="/home/pi/pressure_status.json",
    )
    args = parser.parse_args()

    port = args.port or find_serial_port()
    status_path = Path(args.status_file)

    print("PRESSURE BRIDGE")
    print("===============")
    print("Port             :", port)
    print("Baud             :", args.baud)
    print("Left threshold   :", args.left_threshold)
    print("Right threshold  :", args.right_threshold)
    print("Status file      :", status_path)
    print("Press Ctrl+C to stop.")
    print()

    ser = serial.Serial(port, args.baud, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    last_valid_time = 0.0
    last_left = None
    last_right = None

    try:
        while True:
            now = time.monotonic()
            raw = ser.readline()

            if raw:
                line = raw.decode("ascii", errors="ignore").strip()
                match = LINE_PATTERN.search(line)

                if match:
                    last_left = int(match.group("left"))
                    last_right = int(match.group("right"))
                    last_valid_time = now

            timed_out = (
                last_valid_time == 0.0
                or now - last_valid_time > args.timeout
            )

            if timed_out:
                left_contact = False
                right_contact = False
                both_contact = False
                command_allowed = False
                reason = "SERIAL_TIMEOUT"
            else:
                left_contact = last_left > args.left_threshold
                right_contact = last_right > args.right_threshold
                both_contact = left_contact and right_contact
                command_allowed = both_contact

                if both_contact:
                    reason = "BOTH_HANDLES_HELD"
                elif not left_contact and not right_contact:
                    reason = "BOTH_HANDLES_RELEASED"
                elif not left_contact:
                    reason = "LEFT_HANDLE_RELEASED"
                else:
                    reason = "RIGHT_HANDLE_RELEASED"

            status = {
                "timestamp": time.time(),
                "port": port,
                "baud": args.baud,
                "left_pressure": last_left,
                "right_pressure": last_right,
                "left_threshold": args.left_threshold,
                "right_threshold": args.right_threshold,
                "left_contact": left_contact,
                "right_contact": right_contact,
                "both_contact": both_contact,
                "pi_command_allowed": command_allowed,
                "reason": reason,
                "serial_timeout": timed_out,
            }

            write_status(status_path, status)

            left_text = "-" if last_left is None else str(last_left)
            right_text = "-" if last_right is None else str(last_right)
            result = "ALLOW" if command_allowed else "BLOCK"

            print(
                f"LEFT={left_text:>4} "
                f"RIGHT={right_text:>4} "
                f"PI_COMMAND={result:<5} "
                f"REASON={reason}"
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        raise SystemExit(1)
