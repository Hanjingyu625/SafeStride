"""Non-blocking BE-220 NMEA serial adapter for ROS 2."""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

try:
    import serial
except ImportError:  # pragma: no cover - ROS dependency is checked at runtime
    serial = None

from .nmea import parse_fix


class GpsNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_node')
        self.declare_parameter('port', '/dev/serial/by-id/CHANGE_ME_BE220')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('speed_topic', '/gps/speed')

        self._port = str(self.get_parameter('port').value).strip()
        self._baudrate = int(self.get_parameter('baudrate').value)
        poll_rate = float(self.get_parameter('poll_rate_hz').value)
        self._reconnect_period = float(
            self.get_parameter('reconnect_period_s').value
        )
        if not self._port:
            raise ValueError('GPS port must not be empty')
        if not 1200 <= self._baudrate <= 2_000_000:
            raise ValueError('GPS baudrate must be in [1200, 2000000]')
        if (
            not math.isfinite(poll_rate)
            or not math.isfinite(self._reconnect_period)
            or poll_rate <= 0.0
            or poll_rate > 1000.0
            or self._reconnect_period <= 0.0
            or self._reconnect_period > 60.0
        ):
            raise ValueError('GPS poll and reconnect periods must be positive')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._device = None
        self._buffer = bytearray()
        self._last_connect_attempt = -math.inf
        self._last_error = ''
        self._fix_publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            10,
        )
        self._speed_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter('speed_topic').value),
            10,
        )
        self._timer = self.create_timer(1.0 / poll_rate, self._poll)
        self.get_logger().info(
            'BE-220 GPS adapter configured for %s at %d baud'
            % (self._port, self._baudrate)
        )

    def _now(self) -> float:
        return time.monotonic()

    def _close(self) -> None:
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        self._device = None
        self._buffer.clear()

    def _connect(self) -> None:
        if self._device is not None:
            return
        now = self._now()
        if now - self._last_connect_attempt < self._reconnect_period:
            return
        self._last_connect_attempt = now
        if serial is None:
            error = 'python3-serial is not installed'
            if error != self._last_error:
                self.get_logger().error(error)
                self._last_error = error
            return
        try:
            self._device = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0,
                write_timeout=0,
            )
            self._device.reset_input_buffer()
            self._last_error = ''
            self.get_logger().info('GPS serial connected')
        except Exception as error:
            detail = 'GPS serial unavailable: %s' % error
            if detail != self._last_error:
                self.get_logger().warning(detail)
                self._last_error = detail
            self._close()

    def _publish_sentence(self, sentence: str) -> None:
        fix = parse_fix(sentence)
        if fix is None:
            return
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.status.status = (
            NavSatStatus.STATUS_FIX
            if fix.valid
            else NavSatStatus.STATUS_NO_FIX
        )
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = fix.latitude if fix.valid else math.nan
        message.longitude = fix.longitude if fix.valid else math.nan
        message.altitude = math.nan
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._fix_publisher.publish(message)
        if fix.valid and fix.speed_mps is not None:
            speed = Float32()
            speed.data = float(fix.speed_mps)
            self._speed_publisher.publish(speed)

    def _poll(self) -> None:
        self._connect()
        if self._device is None:
            return
        try:
            waiting = int(self._device.in_waiting)
            if waiting > 0:
                self._buffer.extend(self._device.read(min(waiting, 4096)))
            if len(self._buffer) > 8192:
                self._buffer = self._buffer[-4096:]
            while b'\n' in self._buffer:
                raw, _, remaining = self._buffer.partition(b'\n')
                self._buffer = bytearray(remaining)
                sentence = raw.decode('ascii', errors='ignore').strip()
                self._publish_sentence(sentence)
        except Exception as error:
            self.get_logger().warning('GPS serial read failed: %s' % error)
            self._close()

    def destroy_node(self) -> bool:
        self._close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
