#!/usr/bin/env python3
"""Capture HandlePressure samples in one-second buckets for bench calibration."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import HandlePressure


class PressureCapture(Node):
    def __init__(self, duration: float) -> None:
        super().__init__('pressure_capture')
        self._started = time.monotonic()
        self._duration = duration
        self._bucket = -1
        self._count = 0
        self._left_min = math.inf
        self._left_max = -math.inf
        self._right_min = math.inf
        self._right_max = -math.inf
        self.create_subscription(
            HandlePressure,
            '/handle/pressure',
            self._on_pressure,
            qos_profile_sensor_data,
        )
        print('READY: pressure capture subscribed', flush=True)

    @property
    def expired(self) -> bool:
        return time.monotonic() - self._started >= self._duration

    def _flush(self) -> None:
        if self._count == 0:
            return
        print(
            f'second={self._bucket:02d} samples={self._count} '
            f'left={self._left_min:.0f}..{self._left_max:.0f} '
            f'right={self._right_min:.0f}..{self._right_max:.0f}',
            flush=True,
        )

    def _on_pressure(self, message: HandlePressure) -> None:
        bucket = int(time.monotonic() - self._started)
        if bucket != self._bucket:
            self._flush()
            self._bucket = bucket
            self._count = 0
            self._left_min = math.inf
            self._left_max = -math.inf
            self._right_min = math.inf
            self._right_max = -math.inf
        self._count += 1
        self._left_min = min(self._left_min, message.left_raw)
        self._left_max = max(self._left_max, message.left_raw)
        self._right_min = min(self._right_min, message.right_raw)
        self._right_max = max(self._right_max, message.right_raw)

    def finish(self) -> None:
        self._flush()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error('--duration must be positive')

    rclpy.init()
    node = PressureCapture(args.duration)
    try:
        while rclpy.ok() and not node.expired:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
