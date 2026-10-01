"""Exercise ROS adapter callbacks with message stubs, without a DDS runtime."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Optional
from unittest.mock import Mock

from safestride_navigation.crosswalk_guidance import describe_crosswalk
from safestride_navigation.signal_logic import SignalCountdown


def method(name, **extra):
    path = Path(__file__).parents[1] / 'safestride_navigation/crosswalk_controller_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    target = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == name)
    scope = dict(Optional=Optional, Mapping=Mapping, Any=Any, math=math,
                 json=json, Float32=SimpleNamespace, String=SimpleNamespace,
                 describe_crosswalk=describe_crosswalk, **extra)
    exec(compile(ast.Module(body=[target], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


def test_course_requires_fresh_gps_speed_as_well_as_wheel_motion():
    callback = method('_gps_course_callback')
    node = SimpleNamespace(_now=lambda: 10, _motion_confirmed=lambda _: True,
        _fresh=lambda now, stamp, timeout: now - stamp <= timeout,
        _gps_speed_time=10, _speed_timeout=1, _gps_speed=0.0, _gps_motion=Mock(),
        _gps_course_min_speed=0.20)
    callback(node, SimpleNamespace(data=90))
    node._gps_motion.set_course.assert_not_called()
    node._gps_speed = 0.5
    node._gps_speed_time = 8
    callback(node, SimpleNamespace(data=90))
    node._gps_motion.set_course.assert_not_called()
    node._gps_speed_time = 10
    callback(node, SimpleNamespace(data=90))
    node._gps_motion.set_course.assert_called_once_with(90, 10)


def test_guidance_reports_mapping_and_finite_time_budget_without_json_nan():
    publish = method('_publish_status',
        CrosswalkStatus=lambda: SimpleNamespace(header=SimpleNamespace()),
        STATE_VALUES={'ENTRY_ALLOWED': 3},
        _finite_or_nan=lambda value: value if value is not None else math.nan)
    node = SimpleNamespace(_controller=SimpleNamespace(state='ENTRY_ALLOWED', reason='enough time'),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        _status_publisher=Mock(), _guidance_publisher=Mock(),
        _signal_countdown=SignalCountdown(), _now=lambda: 1000)
    publish(node, active=dict(index=2, signal_direction='et',
            signal_direction_source='intersection_arm_inferred', signal_mapping_reason='inferred'),
        gps_valid=True, signal_valid=True, signal_remaining_s=20, required_entry_s=13,
        crossing_eta_s=None, command=dict(entry_allowed=True, mode='ENTRY_ALLOWED'),
        target_speed_mps=1, intersection_id='42', intersection_name='test', signal_reason='green')
    data = json.loads(node._guidance_publisher.publish.call_args.args[0].data)
    assert data['time_margin_s'] == 7
    assert data['crossing_eta_s'] is None
    assert data['signal_mapping_reason'] == 'inferred'
    assert data['signal_direction_source'] == 'intersection_arm_inferred'


def test_red_phase_countdown_reaches_status_and_guidance_but_not_entry_budget():
    publish = method('_publish_status',
        CrosswalkStatus=lambda: SimpleNamespace(header=SimpleNamespace()),
        STATE_VALUES={'WAIT_AT_CURB': 2},
        _finite_or_nan=lambda value: value if value is not None else math.nan)
    countdown = SignalCountdown()
    countdown.observe(
        dict(itstId='2742', trsmUtcTime=1000000, wtPdsgRmdrCs=600),
        dict(itstId='2742', trsmUtcTime=1000000, wtPdsgStatNm='stop-And-Remain'), 1000)
    node = SimpleNamespace(_controller=SimpleNamespace(state='WAIT_AT_CURB', reason='wait'),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        _status_publisher=Mock(), _guidance_publisher=Mock(),
        _signal_countdown=countdown, _now=lambda: 1014)
    kwargs = dict(active=dict(index=8566, signal_direction='wt'),
        gps_valid=True, signal_valid=True, signal_remaining_s=0, required_entry_s=25,
        crossing_eta_s=None, command=dict(entry_allowed=False, mode='WAIT'),
        target_speed_mps=0, intersection_id='2742', intersection_name='test',
        signal_reason='red pedestrian signal')
    publish(node, **kwargs)
    status = node._status_publisher.publish.call_args.args[0]
    guidance = json.loads(node._guidance_publisher.publish.call_args.args[0].data)
    assert status.signal_countdown_s == guidance['signal_countdown_s'] == 46
    assert status.signal_remaining_s == 0
    assert not status.entry_allowed
    assert guidance['signal_color'] == 'RED'
    kwargs['signal_valid'] = False
    publish(node, **kwargs)
    assert math.isnan(node._status_publisher.publish.call_args.args[0].signal_countdown_s)
    assert json.loads(node._guidance_publisher.publish.call_args.args[0].data)['signal_countdown_s'] is None
