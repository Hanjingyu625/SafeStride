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
             axis_bearing_deg=0, itstId='north'),
        dict(latitude=-8 / 111320, longitude=8 / 111320,
             length_m=10, width_m=3, axis_bearing_deg=90, itstId='east'),
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
        def signal(identifier, direction, timestamp):
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
        assert not node._controller.command(1, 0)['entry_allowed']
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
