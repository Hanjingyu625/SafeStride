"""Non-blocking BE-220 NMEA serial adapter for ROS 2."""

import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('speed_topic', '/gps/speed')
        self.declare_parameter('course_topic', '/gps/course')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('nmea_timeout_s', 2.0)
        self.declare_parameter('fix_timeout_s', 2.0)
        self.declare_parameter('diagnostic_rate_hz', 1.0)
        self.declare_parameter('course_min_speed_mps', 0.20)

        self._port = str(self.get_parameter('port').value).strip()
        self._baudrate = int(self.get_parameter('baudrate').value)
        poll_rate = float(self.get_parameter('poll_rate_hz').value)
        self._reconnect_period = float(
            self.get_parameter('reconnect_period_s').value
        )
        self._nmea_timeout = float(self.get_parameter('nmea_timeout_s').value)
        self._fix_timeout = float(self.get_parameter('fix_timeout_s').value)
        diagnostic_rate = float(
            self.get_parameter('diagnostic_rate_hz').value
        )
        self._course_min_speed = float(
            self.get_parameter('course_min_speed_mps').value
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
            or self._nmea_timeout <= 0.0
            or self._fix_timeout <= 0.0
            or diagnostic_rate <= 0.0
            or diagnostic_rate > 10.0
            or self._course_min_speed < 0.0
        ):
            raise ValueError('GPS timing and speed parameters are invalid')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._device = None
        self._buffer = bytearray()
        self._last_connect_attempt = -math.inf
        self._last_error = ''
        self._connected_at: Optional[float] = None
        self._last_sentence_at: Optional[float] = None
        self._last_fix_sentence_at: Optional[float] = None
        self._last_valid_fix_at: Optional[float] = None
        self._last_latitude = math.nan
        self._last_longitude = math.nan
        self._last_speed = math.nan
        self._last_course = math.nan
        self._sentence_count = 0
        self._parsed_fix_count = 0
        self._valid_fix_count = 0
        self._invalid_fix_count = 0
        self._fix_publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            qos_profile_sensor_data,
        )
        self._speed_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter('speed_topic').value),
            qos_profile_sensor_data,
        )
        self._course_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter('course_topic').value),
            qos_profile_sensor_data,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self._timer = self.create_timer(1.0 / poll_rate, self._poll)
        self._diagnostic_timer = self.create_timer(
            1.0 / diagnostic_rate,
            self._publish_diagnostic,
        )
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
        self._connected_at = None
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
            self._connected_at = now
            self._last_error = ''
            self.get_logger().info('GPS serial connected')
        except Exception as error:
            detail = 'GPS serial unavailable: %s' % error
            if detail != self._last_error:
                self.get_logger().warning(detail)
                self._last_error = detail
            self._close()

    def _publish_sentence(self, sentence: str) -> None:
        if not sentence.startswith('$'):
            return
        now = self._now()
        self._sentence_count += 1
        self._last_sentence_at = now
        fix = parse_fix(sentence)
        if fix is None:
            return
        self._parsed_fix_count += 1
        self._last_fix_sentence_at = now
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
        if fix.valid:
            self._valid_fix_count += 1
            self._last_valid_fix_at = now
            self._last_latitude = fix.latitude
            self._last_longitude = fix.longitude
        else:
            self._invalid_fix_count += 1
        if fix.valid and fix.speed_mps is not None:
            speed = Float32()
            speed.data = float(fix.speed_mps)
            self._speed_publisher.publish(speed)
            self._last_speed = speed.data
        if (
            fix.valid
            and fix.course_deg is not None
            and fix.speed_mps is not None
            and fix.speed_mps >= self._course_min_speed
        ):
            course = Float32()
            course.data = float(fix.course_deg)
            self._course_publisher.publish(course)
            self._last_course = course.data

    @staticmethod
    def _age(now: float, received_at: Optional[float]) -> float:
        if received_at is None or now < received_at:
            return math.inf
        return now - received_at

    @staticmethod
    def _format_float(value: float, digits: int = 3) -> str:
        return ('%.*f' % (digits, value)) if math.isfinite(value) else 'nan'

    def _publish_diagnostic(self) -> None:
        now = self._now()
        sentence_age = self._age(now, self._last_sentence_at)
        fix_age = self._age(now, self._last_valid_fix_at)
        status = DiagnosticStatus()
        status.name = 'SafeStride/GPS'
        status.hardware_id = self._port
        if self._device is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_error or 'GPS serial disconnected'
        elif sentence_age > self._nmea_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = 'GPS serial connected but NMEA is stale'
        elif fix_age > self._fix_timeout:
            status.level = DiagnosticStatus.WARN
            status.message = 'NMEA received but no fresh valid GPS fix'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'GPS fix is fresh'
        status.values = [
            KeyValue(
                key='serial_connected',
                value=str(self._device is not None).lower(),
            ),
            KeyValue(key='port', value=self._port),
            KeyValue(key='baudrate', value=str(self._baudrate)),
            KeyValue(key='sentence_count', value=str(self._sentence_count)),
            KeyValue(
                key='parsed_fix_count', value=str(self._parsed_fix_count)
            ),
            KeyValue(key='valid_fix_count', value=str(self._valid_fix_count)),
            KeyValue(
                key='invalid_fix_count', value=str(self._invalid_fix_count)
            ),
            KeyValue(
                key='nmea_age_s', value=self._format_float(sentence_age)
            ),
            KeyValue(key='fix_age_s', value=self._format_float(fix_age)),
            KeyValue(
                key='latitude',
                value=self._format_float(self._last_latitude, 8),
            ),
            KeyValue(
                key='longitude',
                value=self._format_float(self._last_longitude, 8),
            ),
            KeyValue(
                key='speed_mps', value=self._format_float(self._last_speed)
            ),
            KeyValue(
                key='course_deg',
                value=self._format_float(self._last_course, 1),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostic_publisher.publish(array)

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
