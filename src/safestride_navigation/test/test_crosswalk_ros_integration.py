"""Synthetic GPS/IMU replay through the real ROS node (monitor-only)."""
import json
import math
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('safestride_interfaces.msg')
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from safestride_navigation.crosswalk_controller_node import CrosswalkController


def test_pivot_reselects_crossing_and_rechecks_signal(tmp_path):
    map_file = tmp_path / 'map.json'
    map_file.write_text(json.dumps([
        dict(latitude=0, longitude=0, length_m=10, width_m=3,
             axis_bearing_deg=0, itstId='north', signal_direction='nt'),
        dict(latitude=-8 / 111320, longitude=8 / 111320,
             length_m=10, width_m=3, axis_bearing_deg=90, itstId='east', signal_direction='et'),
    ]))
    config = Path(__file__).resolve().parents[3] / 'config/raspberry_pi.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config),
                    '-p', 'crosswalk_file:=' + str(map_file),
                    '-p', "intersection_map_cache_file:=''",
                    '-p', "intersection_map_file:=''"], domain_id=91)
    node = None
    try:
        node = CrosswalkController()
        now = 0.0
        node._now = lambda: now
        node._controller._clock = lambda: now
        requests = []
        def signal(identifier, direction, timestamp, *, directions=None):
            requests.append((identifier, direction))
            return 60.0, True, ''
        node._signal_state = signal
        assert not node._motion_output_enabled

        def feed(rate=0, course=None):
            imu = Imu()
            imu.header.frame_id = 'imu_link'
            ns = round(now * 1e9)
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(ns, 1000000000)
            imu.orientation.w = 1.0
            imu.angular_velocity.z = float(rate)
            node._imu_callback(imu)
            odom = Odometry()
            odom.twist.twist.linear.x = 0.3 if course is not None else 0.0
            node._odom_callback(odom)
            fix = NavSatFix()
            north = -8 - max(2.2 - now, 0) * 0.3
            fix.latitude, fix.longitude = north / 111320, 0.0
            node._fix_callback(fix)
            if course is not None:
                node._gps_speed_callback(Float32(data=0.3))
                node._gps_course_callback(Float32(data=float(course)))
            node._tick()

        for i in range(12):
            now = i / 5
            feed(course=0)
        assert node._controller.state == 'ENTRY_ALLOWED'
        assert node._controller.locked_intersection_id == 'north'
        for i in range(1, 11):
            now = 2.2 + i / 10
            feed(rate=-math.pi / 2)
        # The current nearest-candidate policy switches immediately before
        # crossing; the mocked fresh green for the new crossing allows entry.
        assert node._controller.locked_intersection_id == 'east'
        assert requests[-1][0] == 'east'
        for i in range(1, 14):
            now = 3.2 + i / 5
            feed()
        assert node._controller.state == 'ENTRY_ALLOWED'
        assert node._controller.locked_intersection_id == 'east'
        assert requests[0][0] == 'north'
        assert requests[-1][0] == 'east'
        assert requests[0][1] != requests[-1][1]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_stationary_drift_and_red_countdown_on_real_ros_messages(tmp_path):
    from rclpy.serialization import serialize_message, deserialize_message
    from safestride_interfaces.msg import CrosswalkStatus
    from safestride_bridge.hmi_model import Snapshot, ENTRY_ALLOWED
    from unittest.mock import Mock

    map_file = tmp_path / 'map.json'
    map_file.write_text('[]')
    rclpy.init(args=['--ros-args', '-p', 'crosswalk_file:=' + str(map_file),
                    '-p', "intersection_map_cache_file:=''",
                    '-p', "intersection_map_file:=''"], domain_id=92)
    node = None
    try:
        node = CrosswalkController()
        now = 1000.0
        node._now = lambda: now
        node._status_publisher = Mock()
        node._guidance_publisher = Mock()
        for second in range(41):
            now = 1000.0 + second
            node._odom_callback(Odometry())
            fix = NavSatFix()
            fix.latitude, fix.longitude = 37 + min(second, 31) / 111320, 127.0
            node._fix_callback(fix)
            assert node._fix == (37.0, 127.0)
        # Moving wheel evidence releases the position on the next accepted fix.
        now += 1
        odom = Odometry()
        odom.twist.twist.linear.x = 0.3
        node._odom_callback(odom)
        node._fix_callback(fix)
        assert node._fix == (fix.latitude, fix.longitude)

        now = 2000.0
        node._signal_countdown.observe(
            dict(itstId='2742', trsmUtcTime=2000000, wtPdsgRmdrCs=600),
            dict(itstId='2742', trsmUtcTime=2000000, wtPdsgStatNm='stop-And-Remain'), now)
        now += 14
        node._controller.state = 'WAIT_AT_CURB'
        node._publish_status(active=dict(index=8566, signal_direction='wt', edge_distance_m=4.4),
            gps_valid=True, signal_valid=True, signal_remaining_s=0.0, required_entry_s=25,
            crossing_eta_s=None, command=dict(entry_allowed=False, mode='WAIT'),
            target_speed_mps=0.0, intersection_id='2742', intersection_name='test',
            signal_reason='red pedestrian signal')
        message = node._status_publisher.publish.call_args.args[0]
        wire = deserialize_message(serialize_message(message), CrosswalkStatus)
        assert wire.signal_countdown_s == 46.0
        assert wire.signal_remaining_s == 0.0
        model = Snapshot()
        model.update('crosswalk', wire, now)
        words = model.words(now)
        assert words[6] == 46
        assert not words[15] & ENTRY_ALLOWED
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
