#!/usr/bin/env python3
"""
Two-handle FSR monitor for Raspberry Pi.

Arduino input format:
    Left : 123    Right : 456
Baud:
    115200

Outputs:
    /home/pi/pressure_control.json
    /home/pi/pressure_session.csv
    /home/pi/pressure_profile.json

Safety behavior:
- Any required handle released for RELEASE_DEBOUNCE_MS:
  pi_command_allowed = false
  requested_action = HARD_STOP_REQUEST
- Serial timeout:
  pi_command_allowed = false
  requested_action = HARD_STOP_REQUEST
- Persistent left/right asymmetry:
  requested_action = CAUTION
- Excessive pressure:
  requested_action = CAUTION

This program blocks/permits Raspberry Pi commands.
Actual motor cutoff must still be enforced in Arduino motor-control firmware.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


LINE_RE = re.compile(
    r"Left\s*:\s*(?P<left>\d+)\s+Right\s*:\s*(?P<right>\d+)",
    re.IGNORECASE,
)


@dataclass
class Settings:
    left_threshold: int = 50
    right_threshold: int = 50
    overload_threshold: int = 900
    asymmetry_ratio_threshold: float = 0.35
    release_debounce_ms: int = 250
    asymmetry_hold_ms: int = 2000
    serial_timeout_ms: int = 1000
    both_hands_required: bool = True
    profile_alpha: float = 0.02


@dataclass
class RuntimeState:
    left_raw: Optional[int] = None
    right_raw: Optional[int] = None
    left_contact: bool = False
    right_contact: bool = False
    both_contact: bool = False
    asymmetry_ratio: float = 0.0
    pressure_total: int = 0
    pi_command_allowed: bool = False
    requested_action: str = "HARD_STOP_REQUEST"
    reason: str = "STARTING"
    serial_timeout: bool = True


def find_port() -> str:
    preferred = [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
    ]
    for item in preferred:
        if Path(item).exists():
            return item

    ports = [item.device for item in list_ports.comports()]
    if ports:
        return ports[0]

    raise RuntimeError("No Arduino serial port found")


def atomic_write_json(path: Path, payload: dict) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temp.replace(path)


def load_profile(path: Path) -> dict:
    if not path.exists():
        return {
            "sample_count": 0,
            "left_held_ema": None,
            "right_held_ema": None,
            "left_load_total": 0.0,
            "right_load_total": 0.0,
            "release_events": 0,
            "left_release_events": 0,
            "right_release_events": 0,
            "asymmetry_events": 0,
            "overload_events": 0,
            "total_session_seconds": 0.0,
        }

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
    except (OSError, json.JSONDecodeError):
        pass

    return load_profile(Path("/nonexistent"))


def update_ema(previous: Optional[float], value: float, alpha: float) -> float:
    if previous is None:
        return value
    return (1.0 - alpha) * previous + alpha * value


def clamp_adc(value: int) -> int:
    return max(0, min(1023, value))


def asymmetry_ratio(left: int, right: int) -> float:
    total = left + right
    if total <= 0:
        return 0.0
    return abs(left - right) / total


def append_csv(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()

    fields = [
        "timestamp",
        "elapsed_s",
        "left_raw",
        "right_raw",
        "left_contact",
        "right_contact",
        "both_contact",
        "asymmetry_ratio",
        "pressure_total",
        "pi_command_allowed",
        "requested_action",
        "reason",
        "serial_timeout",
    ]

    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        if not exists:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in fields})


def profile_summary(profile: dict) -> dict:
    left_load = float(profile.get("left_load_total", 0.0))
    right_load = float(profile.get("right_load_total", 0.0))
    total_load = left_load + right_load

    if total_load > 0:
        left_share = left_load / total_load
        right_share = right_load / total_load
    else:
        left_share = 0.5
        right_share = 0.5

    imbalance = abs(left_share - right_share)

    if total_load <= 0:
        interpretation = "INSUFFICIENT_DATA"
    elif imbalance < 0.10:
        interpretation = "BALANCED"
    elif left_share > right_share:
        interpretation = "LEFT_DOMINANT"
    else:
        interpretation = "RIGHT_DOMINANT"

    return {
        "left_load_share": round(left_share, 4),
        "right_load_share": round(right_share, 4),
        "load_imbalance": round(imbalance, 4),
        "interpretation": interpretation,
        "note": (
            "This is a support-use pattern, not a medical diagnosis. "
            "One FSR per handle can compare left versus right only; "
            "it cannot identify palm position within a handle."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--left-threshold", type=int, default=50)
    parser.add_argument("--right-threshold", type=int, default=50)
    parser.add_argument("--overload-threshold", type=int, default=900)
    parser.add_argument("--asymmetry-ratio", type=float, default=0.35)
    parser.add_argument("--release-ms", type=int, default=250)
    parser.add_argument("--asymmetry-ms", type=int, default=2000)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument(
        "--allow-one-hand",
        action="store_true",
        help="Allow Pi commands when at least one handle is held.",
    )
    parser.add_argument(
        "--control-file",
        default="/home/pi/pressure_control.json",
    )
    parser.add_argument(
        "--profile-file",
        default="/home/pi/pressure_profile.json",
    )
    parser.add_argument(
        "--log-file",
        default="/home/pi/pressure_session.csv",
    )
    args = parser.parse_args()

    settings = Settings(
        left_threshold=args.left_threshold,
        right_threshold=args.right_threshold,
        overload_threshold=args.overload_threshold,
        asymmetry_ratio_threshold=args.asymmetry_ratio,
        release_debounce_ms=args.release_ms,
        asymmetry_hold_ms=args.asymmetry_ms,
        serial_timeout_ms=args.timeout_ms,
        both_hands_required=not args.allow_one_hand,
    )

    port = args.port or find_port()
    control_path = Path(args.control_file)
    profile_path = Path(args.profile_file)
    log_path = Path(args.log_file)
    profile = load_profile(profile_path)

    ser = serial.Serial(port, args.baud, timeout=0.15)
    time.sleep(2.0)
    ser.reset_input_buffer()

    start_monotonic = time.monotonic()
    last_packet_monotonic = 0.0
    last_loop_monotonic = start_monotonic
    last_log_monotonic = 0.0

    left_release_since: Optional[float] = None
    right_release_since: Optional[float] = None
    asymmetry_since: Optional[float] = None

    previous_left_released = False
    previous_right_released = False
    previous_asymmetry_event = False
    previous_overload = False

    state = RuntimeState()

    print("PRESSURE SAFETY + REHAB MONITOR")
    print("===============================")
    print("Port                :", port)
    print("Baud                :", args.baud)
    print("Both hands required :", settings.both_hands_required)
    print("Left threshold      :", settings.left_threshold)
    print("Right threshold     :", settings.right_threshold)
    print("Release debounce    :", settings.release_debounce_ms, "ms")
    print("Asymmetry threshold :", settings.asymmetry_ratio_threshold)
    print("Control JSON        :", control_path)
    print("Profile JSON        :", profile_path)
    print("Session CSV         :", log_path)
    print("Press Ctrl+C to stop.")
    print()

    try:
        while True:
            now = time.monotonic()
            dt = max(0.0, now - last_loop_monotonic)
            last_loop_monotonic = now

            raw_line = ser.readline()
            if raw_line:
                line = raw_line.decode("ascii", errors="ignore").strip()
                match = LINE_RE.search(line)
                if match:
                    state.left_raw = clamp_adc(int(match.group("left")))
                    state.right_raw = clamp_adc(int(match.group("right")))
                    last_packet_monotonic = now

            state.serial_timeout = (
                last_packet_monotonic == 0.0
                or (now - last_packet_monotonic) * 1000.0
                > settings.serial_timeout_ms
            )

            if state.left_raw is None or state.right_raw is None:
                state.left_contact = False
                state.right_contact = False
            else:
                state.left_contact = (
                    state.left_raw > settings.left_threshold
                )
                state.right_contact = (
                    state.right_raw > settings.right_threshold
                )

            state.both_contact = (
                state.left_contact and state.right_contact
            )
            state.pressure_total = (
                (state.left_raw or 0) + (state.right_raw or 0)
            )
            state.asymmetry_ratio = asymmetry_ratio(
                state.left_raw or 0,
                state.right_raw or 0,
            )

            if not state.left_contact:
                if left_release_since is None:
                    left_release_since = now
            else:
                left_release_since = None

            if not state.right_contact:
                if right_release_since is None:
                    right_release_since = now
            else:
                right_release_since = None

            left_released = (
                left_release_since is not None
                and (now - left_release_since) * 1000.0
                >= settings.release_debounce_ms
            )
            right_released = (
                right_release_since is not None
                and (now - right_release_since) * 1000.0
                >= settings.release_debounce_ms
            )

            asymmetry_condition = (
                state.both_contact
                and state.asymmetry_ratio
                >= settings.asymmetry_ratio_threshold
            )
            if asymmetry_condition:
                if asymmetry_since is None:
                    asymmetry_since = now
            else:
                asymmetry_since = None

            persistent_asymmetry = (
                asymmetry_since is not None
                and (now - asymmetry_since) * 1000.0
                >= settings.asymmetry_hold_ms
            )

            overload = (
                (state.left_raw or 0) >= settings.overload_threshold
                or (state.right_raw or 0) >= settings.overload_threshold
            )

            if state.serial_timeout:
                state.pi_command_allowed = False
                state.requested_action = "HARD_STOP_REQUEST"
                state.reason = "SERIAL_TIMEOUT"
            elif settings.both_hands_required and (
                left_released or right_released
            ):
                state.pi_command_allowed = False
                state.requested_action = "HARD_STOP_REQUEST"
                if left_released and right_released:
                    state.reason = "BOTH_HANDLES_RELEASED"
                elif left_released:
                    state.reason = "LEFT_HANDLE_RELEASED"
                else:
                    state.reason = "RIGHT_HANDLE_RELEASED"
            elif not settings.both_hands_required and (
                left_released and right_released
            ):
                state.pi_command_allowed = False
                state.requested_action = "HARD_STOP_REQUEST"
                state.reason = "BOTH_HANDLES_RELEASED"
            elif overload:
                state.pi_command_allowed = True
                state.requested_action = "CAUTION"
                state.reason = "PRESSURE_OVERLOAD"
            elif persistent_asymmetry:
                state.pi_command_allowed = True
                state.requested_action = "CAUTION"
                state.reason = "PERSISTENT_ASYMMETRY"
            else:
                required_contact = (
                    state.both_contact
                    if settings.both_hands_required
                    else (state.left_contact or state.right_contact)
                )
                state.pi_command_allowed = required_contact
                state.requested_action = (
                    "DRIVE_ALLOWED"
                    if required_contact
                    else "HARD_STOP_REQUEST"
                )
                state.reason = (
                    "HANDLES_HELD"
                    if required_contact
                    else "WAITING_FOR_HAND_CONTACT"
                )

            # Session accumulation and simple personalized profile learning.
            if state.left_contact and state.left_raw is not None:
                profile["left_load_total"] = (
                    float(profile.get("left_load_total", 0.0))
                    + state.left_raw * dt
                )
                profile["left_held_ema"] = update_ema(
                    profile.get("left_held_ema"),
                    float(state.left_raw),
                    settings.profile_alpha,
                )

            if state.right_contact and state.right_raw is not None:
                profile["right_load_total"] = (
                    float(profile.get("right_load_total", 0.0))
                    + state.right_raw * dt
                )
                profile["right_held_ema"] = update_ema(
                    profile.get("right_held_ema"),
                    float(state.right_raw),
                    settings.profile_alpha,
                )

            if state.both_contact:
                profile["sample_count"] = int(
                    profile.get("sample_count", 0)
                ) + 1

            if left_released and not previous_left_released:
                profile["left_release_events"] = int(
                    profile.get("left_release_events", 0)
                ) + 1
                profile["release_events"] = int(
                    profile.get("release_events", 0)
                ) + 1

            if right_released and not previous_right_released:
                profile["right_release_events"] = int(
                    profile.get("right_release_events", 0)
                ) + 1
                profile["release_events"] = int(
                    profile.get("release_events", 0)
                ) + 1

            if persistent_asymmetry and not previous_asymmetry_event:
                profile["asymmetry_events"] = int(
                    profile.get("asymmetry_events", 0)
                ) + 1

            if overload and not previous_overload:
                profile["overload_events"] = int(
                    profile.get("overload_events", 0)
                ) + 1

            profile["total_session_seconds"] = (
                float(profile.get("total_session_seconds", 0.0)) + dt
            )

            previous_left_released = left_released
            previous_right_released = right_released
            previous_asymmetry_event = persistent_asymmetry
            previous_overload = overload

            summary = profile_summary(profile)

            control_payload = {
                "timestamp": time.time(),
                "elapsed_s": round(now - start_monotonic, 3),
                "settings": asdict(settings),
                "left_pressure": state.left_raw,
                "right_pressure": state.right_raw,
                "left_contact": state.left_contact,
                "right_contact": state.right_contact,
                "both_contact": state.both_contact,
                "asymmetry_ratio": round(state.asymmetry_ratio, 4),
                "pressure_total": state.pressure_total,
                "pi_command_allowed": state.pi_command_allowed,
                "requested_action": state.requested_action,
                "reason": state.reason,
                "serial_timeout": state.serial_timeout,
                "rehab_summary": summary,
            }
            atomic_write_json(control_path, control_payload)

            profile_payload = dict(profile)
            profile_payload["summary"] = summary
            atomic_write_json(profile_path, profile_payload)

            if now - last_log_monotonic >= 0.2:
                append_csv(
                    log_path,
                    {
                        "timestamp": time.time(),
                        "elapsed_s": round(now - start_monotonic, 3),
                        "left_raw": state.left_raw,
                        "right_raw": state.right_raw,
                        "left_contact": state.left_contact,
                        "right_contact": state.right_contact,
                        "both_contact": state.both_contact,
                        "asymmetry_ratio": round(
                            state.asymmetry_ratio, 4
                        ),
                        "pressure_total": state.pressure_total,
                        "pi_command_allowed": (
                            state.pi_command_allowed
                        ),
                        "requested_action": state.requested_action,
                        "reason": state.reason,
                        "serial_timeout": state.serial_timeout,
                    },
                )
                last_log_monotonic = now

            left_text = (
                "-" if state.left_raw is None else str(state.left_raw)
            )
            right_text = (
                "-" if state.right_raw is None else str(state.right_raw)
            )
            gate = "ALLOW" if state.pi_command_allowed else "BLOCK"

            print(
                f"L={left_text:>4} "
                f"R={right_text:>4} "
                f"ASYM={state.asymmetry_ratio:>5.2f} "
                f"PI={gate:<5} "
                f"ACTION={state.requested_action:<17} "
                f"REASON={state.reason}"
            )

            time.sleep(0.03)

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
