"""Exercise the production ROS callback without requiring a DDS installation."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Optional

import pytest

from safestride_navigation.heading_fusion import HeadingFusion


def adapter():
    path = Path(__file__).parents[1] / 'safestride_navigation/crosswalk_controller_node.py'
    tree = ast.parse(path.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef)
               and item.name in ('_imu_callback', '_heading')]
    scope = dict(math=math, Imu=object, Optional=Optional)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    n = NS(now=0, _heading_fusion=HeadingFusion(), _imu_frame='imu_link',
           _gyro_sign=1, _odom_time=0, _speed_timeout=1,
           _odom_speed=0, _wheel_motion_min_speed=0.02, _heading_timeout=5)
    n._now = lambda: n.now
    n._fresh = lambda now, timestamp, timeout: 0 <= now - timestamp <= timeout
    n._wheel_motion_active = lambda now: False
    n._imu_callback = lambda message: scope['_imu_callback'](n, message)
    n._heading = lambda now: scope['_heading'](n, now)
    return n


def message(now, rate=0, pitch=0):
    seconds = int(now)
    return NS(header=NS(frame_id='imu_link',
                        stamp=NS(sec=seconds, nanosec=round((now - seconds) * 1e9))),
              orientation=NS(x=0, y=math.sin(pitch/2), z=0, w=math.cos(pitch/2)),
              angular_velocity=NS(x=0, y=0, z=rate),
              angular_velocity_covariance=[0.02] * 9,
              orientation_covariance=[0.08] * 9)


def test_ros_clockwise_pivot_and_tilt_compensation():
    n = adapter()
    pitch = math.radians(20)
    n._imu_callback(message(0, pitch=pitch))
    n._heading_fusion.gps(0, 0, 0)
    for i in range(1, 11):
        n.now = i / 10
        n._imu_callback(message(n.now, -math.pi / 2 * math.cos(pitch), pitch))
    assert n._heading_fusion.heading(1) == pytest.approx(90)


@pytest.mark.parametrize('fault', ['stale', 'frame', 'invalid', 'quaternion', 'tilt', 'nan'])
def test_bad_ros_imu_revokes_heading(fault):
    n = adapter()
    n._heading_fusion.gps(0, 0, 0)
    msg = message(0)
    if fault == 'stale':
        n.now = 1
    elif fault == 'frame':
        msg.header.frame_id = 'unexpected'
    elif fault == 'invalid':
        msg.angular_velocity_covariance[0] = -1
    elif fault == 'quaternion':
        msg.orientation.w = 0
    elif fault == 'tilt':
        msg = message(0, pitch=math.radians(50))
    else:
        msg.angular_velocity.z = math.nan
    n._imu_callback(msg)
    assert n._heading_fusion.heading(n.now) is None


def test_polling_old_gps_cannot_keep_heading_alive():
    n = adapter()
    n._gps_motion = NS(heading=lambda now, timeout: 0 if now <= timeout else None,
                       heading_time=0)
    assert n._heading(0) == 0
    assert n._heading(2) == 0
    assert n._heading(5.1) is None
