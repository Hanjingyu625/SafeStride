#!/usr/bin/env python3
"""Run the guarded, time-limited SafeStride motor-driver bench sequence."""

from __future__ import annotations

import argparse
import time

import serial


def read_for(port: serial.Serial, duration: float) -> str:
    deadline = time.monotonic() + duration
    received = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            received.extend(chunk)
    return received.decode('utf-8', errors='replace')


def send_line(port: serial.Serial, line: str) -> None:
    port.write((line + '\n').encode('ascii'))
    port.flush()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/safestride-drive')
    parser.add_argument(
        '--confirmed-wheels-lifted',
        action='store_true',
        help='required physical-safety acknowledgement',
    )
    args = parser.parse_args()
    if not args.confirmed_wheels_lifted:
        parser.error('--confirmed-wheels-lifted is required')

    port = serial.Serial(
        args.port,
        baudrate=115200,
        timeout=0.1,
        write_timeout=1.0,
    )
    try:
        time.sleep(2.5)
        greeting = read_for(port, 0.5)
        send_line(port, 'STATUS')
        status = read_for(port, 0.5)
        startup = greeting + status
        print(startup, end='', flush=True)
        if 'SafeStride SZH-GNP521 bench test' not in startup:
            send_line(port, 'HELP')
            help_text = read_for(port, 0.7)
            print(help_text, end='', flush=True)
            startup += help_text
        if 'SafeStride SZH-GNP521 bench test' not in startup:
            raise RuntimeError('bench firmware did not identify itself')
        if 'STATUS STOPPED' not in startup:
            raise RuntimeError('bench firmware did not confirm stopped state')

        for pwm in (90, 100):
            print(f'BEGIN PWM={pwm} DURATION_MS=1000', flush=True)
            send_line(port, f'RUN {pwm} 1000 CONFIRM')
            result = read_for(port, 1.5)
            print(result, end='', flush=True)
            if f'RUNNING pwm={pwm}' not in result:
                raise RuntimeError(f'PWM {pwm} command was not accepted')
            if 'STOPPED duration elapsed' not in result:
                raise RuntimeError(f'PWM {pwm} did not report timed stop')
            print(f'END PWM={pwm}', flush=True)
            time.sleep(2.0)
    finally:
        try:
            send_line(port, 'STOP')
            print(read_for(port, 0.3), end='', flush=True)
        finally:
            port.close()


if __name__ == '__main__':
    main()
