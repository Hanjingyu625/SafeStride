"""Publish one read-only diagnostic status for every SafeStride device."""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import socket
import time
from typing import Dict, Iterable, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import (
    CrosswalkStatus,
    DriveCommand,
    HandlePressure,
    SurfaceCondition,
    TerrainStatus,
    WalkerStatus,
    WheelHall,
)
from sensor_msgs.msg import BatteryState, CompressedImage, NavSatFix


CRITICAL_DRIVE_FAULT_MASK = (
    WalkerStatus.FAULT_COMMUNICATION
    | WalkerStatus.FAULT_MOTOR_DRIVER
    | WalkerStatus.FAULT_PROTOCOL
)
HALL_FAULT_MASK = WalkerStatus.FAULT_LEFT_HALL | WalkerStatus.FAULT_RIGHT_HALL


def _text(value) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, float):
        return 'unavailable' if not math.isfinite(value) else f'{value:.4f}'
    return str(value)


class DeviceHealthMonitor(Node):
    """Aggregate connection and telemetry health without controlling motion."""

    def __init__(self) -> None:
        super().__init__('device_health_monitor')
        defaults = {
            'publish_rate_hz': 1.0,
            'drive_timeout_s': 0.60,
            'terrain_timeout_s': 0.60,
            'gps_timeout_s': 2.50,
            'camera_timeout_s': 2.50,
            'perception_timeout_s': 2.50,
            'crosswalk_timeout_s': 2.50,
            'command_timeout_s': 0.75,
            'terrain_expected': True,
            'gps_expected': True,
            'camera_expected': False,
            'perception_expected': False,
            'crosswalk_expected': True,
            'motor_driver_fault_input_available': False,
            'pressure_disconnect_detection_available': False,
            'independent_motor_feedback_available': False,
            'battery_monitoring_available': False,
            'battery_nominal_voltage_v': 12.0,
            'battery_nominal_capacity_ah': 0.0,
            'converter_monitoring_available': False,
            'converter_nominal_input_v': 12.0,
            'converter_nominal_output_v': 5.0,
            'walker_status_topic': '/walker/status',
            'pressure_topic': '/handle/pressure',
            'hall_topic': '/wheel/hall',
            'terrain_status_topic': '/terrain/status',
            'battery_topic': '/battery_state',
            'gps_fix_topic': '/gps/fix',
            'camera_topic': '/camera/image/compressed',
            'surface_topic': '/perception/surface_condition',
            'crosswalk_topic': '/crosswalk/status',
            'drive_command_topic': '/drive/command',
            'diagnostics_topic': '/diagnostics',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._params = {
            name: self.get_parameter(name).value for name in defaults
        }
        rate = float(self._params['publish_rate_hz'])
        if not math.isfinite(rate) or rate <= 0.0 or rate > 20.0:
            raise ValueError('publish_rate_hz must be in (0, 20]')

        self._samples: Dict[str, Tuple[object, float]] = {}
        self._publisher = self.create_publisher(
            DiagnosticArray,
            str(self._params['diagnostics_topic']),
            10,
        )
        subscriptions = (
            ('drive', WalkerStatus, 'walker_status_topic'),
            ('pressure', HandlePressure, 'pressure_topic'),
            ('hall', WheelHall, 'hall_topic'),
            ('terrain', TerrainStatus, 'terrain_status_topic'),
            ('battery', BatteryState, 'battery_topic'),
            ('gps', NavSatFix, 'gps_fix_topic'),
            ('camera', CompressedImage, 'camera_topic'),
            ('surface', SurfaceCondition, 'surface_topic'),
            ('crosswalk', CrosswalkStatus, 'crosswalk_topic'),
            ('command', DriveCommand, 'drive_command_topic'),
        )
        for key, message_type, topic_param in subscriptions:
            self.create_subscription(
                message_type,
                str(self._params[topic_param]),
                lambda message, sample_key=key: self._store(
                    sample_key, message
                ),
                qos_profile_sensor_data,
            )
        self.create_timer(1.0 / rate, self._publish)

    def _store(self, key: str, message: object) -> None:
        self._samples[key] = (message, time.monotonic())

    def _sample(self, key: str) -> Tuple[Optional[object], float]:
        stored = self._samples.get(key)
        if stored is None:
            return None, math.inf
        return stored[0], max(0.0, time.monotonic() - stored[1])

    @staticmethod
    def _diagnostic(
        name: str,
        hardware_id: str,
        level: int,
        message: str,
        values: Iterable[Tuple[str, object]] = (),
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = hardware_id
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key=key, value=_text(value)) for key, value in values
        ]
        return status

    def _host_statuses(self) -> list[DiagnosticStatus]:
        values = [('hostname', socket.gethostname())]
        try:
            values.append(('load_1m', os.getloadavg()[0]))
        except OSError:
            values.append(('load_1m', math.nan))
        try:
            memory = {}
            for line in Path('/proc/meminfo').read_text().splitlines():
                key, raw = line.split(':', 1)
                memory[key] = int(raw.strip().split()[0])
            total = float(memory['MemTotal'])
            available = float(memory['MemAvailable'])
            values.append(('memory_used_percent', 100.0 * (1 - available / total)))
        except (OSError, KeyError, ValueError, ZeroDivisionError):
            values.append(('memory_used_percent', math.nan))
        disk = shutil.disk_usage('/')
        values.append(('disk_used_percent', 100.0 * disk.used / disk.total))
        try:
            raw_temp = Path(
                '/sys/class/thermal/thermal_zone0/temp'
            ).read_text().strip()
            values.append(('cpu_temperature_c', float(raw_temp) / 1000.0))
        except (OSError, ValueError):
            values.append(('cpu_temperature_c', math.nan))
        statuses = [self._diagnostic(
            'SafeStride devices/Raspberry Pi',
            socket.gethostname(),
            DiagnosticStatus.OK,
            'host monitor active',
            values,
        )]
        interfaces = []
        network_root = Path('/sys/class/net')
        if network_root.exists():
            for interface in sorted(network_root.iterdir()):
                try:
                    state = (interface / 'operstate').read_text().strip()
                except OSError:
                    state = 'unknown'
                interfaces.append((interface.name, state))
        statuses.append(self._diagnostic(
            'SafeStride devices/network interfaces',
            socket.gethostname(),
            DiagnosticStatus.OK if interfaces else DiagnosticStatus.WARN,
            'interface link states' if interfaces else 'no interface data',
            interfaces,
        ))
        return statuses

    def _drive_status(self) -> DiagnosticStatus:
        message, age = self._sample('drive')
        if message is None or age > float(self._params['drive_timeout_s']):
            return self._diagnostic(
                'SafeStride devices/Drive Arduino',
                'drive-uno',
                DiagnosticStatus.ERROR,
                'telemetry missing or stale; motion must be inhibited',
                [('age_s', age)],
            )
        critical = int(message.fault_bits) & CRITICAL_DRIVE_FAULT_MASK
        level = DiagnosticStatus.OK
        summary = 'serial session and telemetry active'
        if not message.link_ok or critical:
            level = DiagnosticStatus.ERROR
            summary = 'drive link or critical fault'
        elif int(message.fault_bits) & HALL_FAULT_MASK:
            level = DiagnosticStatus.WARN
            summary = 'drive active with advisory Hall fault'
        return self._diagnostic(
            'SafeStride devices/Drive Arduino',
            f'drive-boot-{int(message.boot_id):08x}',
            level,
            summary,
            (
                ('age_s', age),
                ('link_ok', bool(message.link_ok)),
                ('state', int(message.state)),
                ('armed', bool(message.armed)),
                ('watchdog_timeout', bool(message.watchdog_timeout)),
                ('fault_bits', f'0x{int(message.fault_bits):04x}'),
                ('boot_id', int(message.boot_id)),
                ('session_id', int(message.session_id)),
            ),
        )

    def _pressure_statuses(self) -> list[DiagnosticStatus]:
        message, age = self._sample('pressure')
        timeout = float(self._params['drive_timeout_s'])
        continuity = bool(
            self._params['pressure_disconnect_detection_available']
        )
        statuses = []
        for side in ('left', 'right'):
            name = f'SafeStride devices/pressure {side}'
            if message is None or age > timeout:
                statuses.append(self._diagnostic(
                    name,
                    f'pressure-{side}',
                    DiagnosticStatus.ERROR,
                    'pressure telemetry missing or stale',
                    [('age_s', age)],
                ))
                continue
            raw = float(getattr(message, f'{side}_raw'))
            filtered = float(getattr(message, f'{side}_filtered'))
            present = bool(getattr(message, f'{side}_present'))
            telemetry_valid = math.isfinite(raw) and math.isfinite(filtered)
            if not telemetry_valid or not bool(message.calibrated):
                level = DiagnosticStatus.ERROR
                summary = 'pressure telemetry unavailable or uncalibrated'
            elif not continuity:
                level = DiagnosticStatus.WARN
                summary = (
                    'telemetry active; electrical disconnect is not '
                    'distinguishable from a released FSR'
                )
            else:
                level = DiagnosticStatus.OK
                summary = 'pressure channel monitoring active'
            statuses.append(self._diagnostic(
                name,
                f'pressure-{side}',
                level,
                summary,
                (
                    ('age_s', age),
                    ('raw_adc', raw),
                    ('filtered_adc', filtered),
                    ('hand_present', present),
                    ('calibrated', bool(message.calibrated)),
                    ('electrical_continuity_observable', continuity),
                ),
            ))
        return statuses

    def _hall_status(self) -> DiagnosticStatus:
        hall, age = self._sample('hall')
        drive, _ = self._sample('drive')
        command, _ = self._sample('command')
        moving_command = bool(
            command is not None
            and abs(float(command.target_linear_m_s)) > 0.02
        )
        if hall is None or age > float(self._params['drive_timeout_s']):
            return self._diagnostic(
                'SafeStride devices/Hall sensor',
                'left-a3-wsh135',
                DiagnosticStatus.WARN,
                'Hall telemetry unavailable; it does not inhibit motion',
                [('age_s', age), ('motion_inhibit', False)],
            )
        speed_valid = bool(drive and drive.speed_valid)
        fault = bool(
            drive and int(drive.fault_bits) & HALL_FAULT_MASK
        )
        level = DiagnosticStatus.WARN if fault or (
            moving_command and not speed_valid
        ) else DiagnosticStatus.OK
        summary = (
            'Hall feedback degraded; feed-forward motion remains available'
            if level == DiagnosticStatus.WARN
            else 'Hall pulse telemetry active'
        )
        return self._diagnostic(
            'SafeStride devices/Hall sensor',
            'left-a3-wsh135',
            level,
            summary,
            (
                ('age_s', age),
                ('left_pulses', int(hall.left_pulses)),
                ('velocity_rad_s', float(hall.left_velocity_rad_s)),
                ('speed_valid', speed_valid),
                ('calibrated', bool(hall.calibrated)),
                ('right_channel_is_mirrored', True),
                ('motion_inhibit', False),
            ),
        )

    def _motor_statuses(self) -> list[DiagnosticStatus]:
        drive, age = self._sample('drive')
        drive_fresh = bool(
            drive is not None
            and age <= float(self._params['drive_timeout_s'])
        )
        independent = bool(
            self._params['independent_motor_feedback_available']
        )
        driver_fault_input = bool(
            self._params['motor_driver_fault_input_available']
        )
        applied_pwm = int(drive.applied_pwm) if drive else 0
        driver_fault = bool(
            drive and int(drive.fault_bits) & WalkerStatus.FAULT_MOTOR_DRIVER
        )
        if not drive_fresh:
            driver_level = DiagnosticStatus.ERROR
            driver_summary = 'Drive Arduino telemetry missing; state unknown'
        elif driver_fault:
            driver_level = DiagnosticStatus.ERROR
            driver_summary = 'motor-driver fault reported'
        elif not driver_fault_input:
            driver_level = DiagnosticStatus.WARN
            driver_summary = 'fault pin not installed; connection is unobservable'
        else:
            driver_level = DiagnosticStatus.OK
            driver_summary = 'driver fault input healthy'
        statuses = [self._diagnostic(
            'SafeStride devices/motor driver',
            'szh-gnp521',
            driver_level,
            driver_summary,
            (
                ('drive_telemetry_age_s', age),
                ('fault_input_available', driver_fault_input),
                ('applied_pwm', applied_pwm),
            ),
        )]
        for side in ('left', 'right'):
            if not drive_fresh:
                motor_level = DiagnosticStatus.ERROR
                motor_summary = 'Drive Arduino telemetry missing; state unknown'
            elif independent:
                motor_level = DiagnosticStatus.OK
                motor_summary = 'independent motor feedback active'
            else:
                motor_level = DiagnosticStatus.WARN
                motor_summary = (
                    'shared output; individual connection is unobservable'
                )
            statuses.append(self._diagnostic(
                f'SafeStride devices/motor {side}',
                f'motor-{side}',
                motor_level,
                motor_summary,
                (
                    ('drive_telemetry_age_s', age),
                    ('applied_pwm', applied_pwm),
                    ('shared_driver_output', True),
                    ('independent_feedback_available', independent),
                ),
            ))
        return statuses

    def _terrain_statuses(self) -> list[DiagnosticStatus]:
        expected = bool(self._params['terrain_expected'])
        terrain, age = self._sample('terrain')
        timeout = float(self._params['terrain_timeout_s'])
        if not expected:
            disabled = self._diagnostic(
                'SafeStride devices/Terrain Arduino',
                'terrain-uno',
                DiagnosticStatus.OK,
                'disabled by launch configuration',
            )
            return [disabled]
        if terrain is None or age > timeout:
            missing = self._diagnostic(
                'SafeStride devices/Terrain Arduino',
                'terrain-uno',
                DiagnosticStatus.ERROR,
                'terrain telemetry missing or stale; motion is not inhibited',
                [('age_s', age), ('motion_inhibit', False)],
            )
            return [missing]
        board = self._diagnostic(
            'SafeStride devices/Terrain Arduino',
            'terrain-uno',
            DiagnosticStatus.OK,
            'terrain telemetry active',
            [('age_s', age), ('fault_bits', f'0x{int(terrain.fault_bits):04x}')],
        )
        tof_ok = bool(terrain.tof_valid) and not bool(
            int(terrain.fault_bits) & TerrainStatus.FAULT_TOF_INVALID
        )
        imu_ok = bool(terrain.mpu_valid) and not bool(
            int(terrain.fault_bits) & TerrainStatus.FAULT_MPU_INVALID
        )
        tof = self._diagnostic(
            'SafeStride devices/TOF10120',
            'tof10120-i2c-0x52',
            DiagnosticStatus.OK if tof_ok else DiagnosticStatus.WARN,
            'TOF active' if tof_ok else 'TOF degraded; motion is not inhibited',
            (
                ('distance_m', float(terrain.tof_distance_m)),
                ('alert', int(terrain.tof_alert)),
                ('terrain_hazard', bool(terrain.terrain_hazard)),
                ('motion_inhibit', False),
            ),
        )
        imu = self._diagnostic(
            'SafeStride devices/MPU6050',
            'mpu6050-i2c-0x68-or-0x69',
            DiagnosticStatus.OK if imu_ok else DiagnosticStatus.WARN,
            'IMU active' if imu_ok else 'IMU degraded; motion is not inhibited',
            (
                ('pitch_rad', float(terrain.pitch_rad)),
                ('roll_rad', float(terrain.roll_rad)),
                ('valid', bool(terrain.mpu_valid)),
                ('motion_inhibit', False),
            ),
        )
        return [board, tof, imu]

    def _optional_topic_status(
        self,
        key: str,
        name: str,
        hardware_id: str,
        expected_param: str,
        timeout_param: str,
    ) -> DiagnosticStatus:
        expected = bool(self._params[expected_param])
        message, age = self._sample(key)
        if not expected:
            return self._diagnostic(
                name,
                hardware_id,
                DiagnosticStatus.OK,
                'disabled by launch configuration',
            )
        if message is None or age > float(self._params[timeout_param]):
            return self._diagnostic(
                name,
                hardware_id,
                DiagnosticStatus.WARN,
                'expected telemetry missing or stale; motion is not inhibited',
                [('age_s', age), ('motion_inhibit', False)],
            )
        return self._diagnostic(
            name,
            hardware_id,
            DiagnosticStatus.OK,
            'telemetry active',
            [('age_s', age)],
        )

    def _battery_status(self) -> DiagnosticStatus:
        available = bool(self._params['battery_monitoring_available'])
        battery, age = self._sample('battery')
        values = [
            ('monitoring_available', available),
            ('nominal_voltage_v', self._params['battery_nominal_voltage_v']),
            ('nominal_capacity_ah', self._params['battery_nominal_capacity_ah']),
            ('age_s', age),
        ]
        if battery is not None:
            values.extend((
                ('present', bool(battery.present)),
                ('voltage_v', float(battery.voltage)),
                ('current_a', float(battery.current)),
                ('percentage', float(battery.percentage)),
                ('capacity_ah', float(battery.capacity)),
                ('design_capacity_ah', float(battery.design_capacity)),
            ))
        if not available:
            return self._diagnostic(
                'SafeStride devices/battery',
                '12v-battery',
                DiagnosticStatus.WARN,
                'voltage/current/capacity sensing is not installed',
                values,
            )
        valid = bool(
            battery is not None
            and age <= float(self._params['drive_timeout_s'])
            and battery.present
            and math.isfinite(float(battery.voltage))
        )
        return self._diagnostic(
            'SafeStride devices/battery',
            '12v-battery',
            DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR,
            'battery telemetry active' if valid else 'battery telemetry invalid',
            values,
        )

    def _power_status(self) -> DiagnosticStatus:
        available = bool(self._params['converter_monitoring_available'])
        return self._diagnostic(
            'SafeStride devices/DC-DC converter',
            'xl4015',
            DiagnosticStatus.OK if available else DiagnosticStatus.WARN,
            (
                'converter monitoring configured'
                if available
                else 'no converter voltage/temperature telemetry is installed'
            ),
            (
                ('monitoring_available', available),
                ('nominal_input_v', self._params['converter_nominal_input_v']),
                ('nominal_output_v', self._params['converter_nominal_output_v']),
            ),
        )

    def _command_status(self) -> DiagnosticStatus:
        command, age = self._sample('command')
        fresh = bool(
            command is not None
            and age <= float(self._params['command_timeout_s'])
        )
        values = [('age_s', age)]
        if command is not None:
            values.extend((
                ('target_linear_m_s', float(command.target_linear_m_s)),
                ('slope_ff_pwm', int(command.slope_ff_pwm)),
                ('drive_pwm_cap', int(command.drive_pwm_cap)),
                ('mode', int(command.mode)),
            ))
        return self._diagnostic(
            'SafeStride devices/supervised command link',
            'ros-drive-command',
            DiagnosticStatus.OK if fresh else DiagnosticStatus.ERROR,
            'command stream active' if fresh else 'command stream missing or stale',
            values,
        )

    def _publish(self) -> None:
        statuses = self._host_statuses()
        statuses.append(self._drive_status())
        statuses.extend(self._pressure_statuses())
        statuses.append(self._hall_status())
        statuses.extend(self._motor_statuses())
        statuses.extend(self._terrain_statuses())
        statuses.append(self._optional_topic_status(
            'gps',
            'SafeStride devices/GPS receiver',
            'be-220-uart',
            'gps_expected',
            'gps_timeout_s',
        ))
        statuses.append(self._optional_topic_status(
            'camera',
            'SafeStride devices/camera',
            'v4l2-camera',
            'camera_expected',
            'camera_timeout_s',
        ))
        statuses.append(self._optional_topic_status(
            'surface',
            'SafeStride devices/road-surface model',
            'torchscript-model',
            'perception_expected',
            'perception_timeout_s',
        ))
        statuses.append(self._optional_topic_status(
            'crosswalk',
            'SafeStride devices/crosswalk controller',
            'seoul-v2x',
            'crosswalk_expected',
            'crosswalk_timeout_s',
        ))
        statuses.append(self._battery_status())
        statuses.append(self._power_status())
        statuses.append(self._command_status())

        levels = [status.level for status in statuses]
        worst = (
            DiagnosticStatus.ERROR
            if DiagnosticStatus.ERROR in levels
            else DiagnosticStatus.WARN
            if DiagnosticStatus.WARN in levels
            else DiagnosticStatus.OK
        )
        counts = {
            level: levels.count(level)
            for level in (
                DiagnosticStatus.OK,
                DiagnosticStatus.WARN,
                DiagnosticStatus.ERROR,
                DiagnosticStatus.STALE,
            )
        }
        statuses.insert(0, self._diagnostic(
            'SafeStride devices/summary',
            'safestride',
            worst,
            'device inventory status',
            (
                ('ok_count', counts[DiagnosticStatus.OK]),
                ('warn_count', counts[DiagnosticStatus.WARN]),
                ('error_count', counts[DiagnosticStatus.ERROR]),
                ('stale_count', counts[DiagnosticStatus.STALE]),
                ('monitor_is_read_only', True),
            ),
        ))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._publisher.publish(array)


def main(args=None) -> None:
    """Run the SafeStride device health monitor."""
    rclpy.init(args=args)
    node = DeviceHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
