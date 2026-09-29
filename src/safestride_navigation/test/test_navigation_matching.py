import math

import pytest

from safestride_navigation.crosswalk_data import nearest_crosswalk, resolve_signal_direction
from safestride_navigation.crossing_policy import CrossingStateMachine
from safestride_navigation.gps_motion import GpsMotionTracker


def crossing(index=1, east=20, north=0, axis=0):
    return dict(index=index, latitude=north / 111320, longitude=east / 111320,
                length_m=10.0, width_m=3.0, axis_bearing_deg=axis)


def arm(item, heading):
    selected = nearest_crosswalk([item], item['latitude'], item['longitude'], heading_deg=heading)
    return resolve_signal_direction(selected,
        dict(intersection_id='42', latitude=0.0, longitude=0.0), '42')


def test_same_crosswalk_signal_arm_does_not_flip_with_walking_direction():
    assert arm(crossing(), 0)['signal_direction'] == 'et'
    assert arm(crossing(), 180)['signal_direction'] == 'et'
    assert arm(crossing(east=-20), 0)['signal_direction'] == 'wt'


@pytest.mark.parametrize('item', [crossing(east=1), crossing(east=0, north=20), crossing(axis=22.5)])
def test_uncertain_geometry_never_falls_back_to_walking_direction(item):
    assert arm(item, item['axis_bearing_deg'])['signal_direction'] == ''


def test_near_centre_field_geometry_retains_both_arms_without_user_heading():
    item = crossing(east=1, north=0, axis=83.1)
    for heading in (None, 83.1, 263.1):
        resolved = arm(item, heading)
        assert resolved['signal_direction'] == ''
        assert set(resolved['signal_direction_candidates']) == {'nt', 'st'}


def test_explicit_mapping_overrides_inference_and_mismatched_map_is_not_used():
    item = crossing()
    item.update(signal_direction='nt', signal_direction_source='crosswalk_data')
    assert resolve_signal_direction(item, None, '42')['signal_direction'] == 'nt'
    item['signal_direction_source'] = 'walking_axis_inferred'
    assert resolve_signal_direction(item, dict(intersection_id='43'), '42')['signal_direction'] == ''


def test_pre_entry_candidate_changes_without_heading_or_hold():
    now = [0.0]
    machine = CrossingStateMachine(clock=lambda: now[0])
    old = nearest_crosswalk([crossing(east=0)], -10 / 111320, 0.0, heading_deg=0)
    machine.lock(old, '42')
    machine.set_state('WAIT_AT_CURB', 'test')
    alternative = nearest_crosswalk([crossing(2, east=8, north=-10, axis=90)],
                                    -10 / 111320, 0.0, heading_deg=90)
    machine.reconsider_candidate(alternative, -10 / 111320, 0, None)
    assert machine.locked_crosswalk is None
    machine.lock(old, '42')
    machine.set_state('CROSSING', 'test')
    for moment in (3.0, 6.0):
        now[0] = moment
        machine.reconsider_candidate(alternative, -10 / 111320, 0, 90)
    assert machine.locked_crosswalk['index'] == 1


def test_same_candidate_preserves_lock_without_heading():
    now = [0.0]
    machine = CrossingStateMachine(clock=lambda: now[0])
    old = nearest_crosswalk([crossing(east=0)], -10 / 111320, 0.0, heading_deg=0)
    machine.lock(old, '42')
    machine.set_state('WAIT_AT_CURB', 'test')
    machine.reconsider_candidate(old, -10 / 111320, 0, None)
    assert machine.locked_crosswalk is not None


def test_course_wrap_and_position_fallback():
    tracker = GpsMotionTracker(change_threshold_m=0.5, heading_min_move_m=2, heading_max_step_m=30)
    tracker.update(0, 0, 0)
    tracker.set_course(359, 0)
    tracker.set_course(1, 1)
    assert tracker.heading(1, 5) == pytest.approx(0)
    tracker.update(0, 5 / 111320, 1.5)
    assert tracker.heading_source == 'rmc_course'
    tracker.update(0, 10 / 111320, 4)
    assert tracker.heading_source == 'position_delta'
    assert tracker.heading(4, 5) == pytest.approx(90)
    assert tracker.heading(10, 5) is None


@pytest.mark.parametrize('speed', [0.0, None, math.nan])
def test_stopped_or_missing_speed_does_not_produce_optimistic_crossing_eta(speed):
    machine = CrossingStateMachine()
    selected = nearest_crosswalk([crossing(east=0)], -10 / 111320, 0, heading_deg=0)
    machine.lock(selected, '42')
    machine.set_state('CROSSING', 'test')
    _, _, eta = machine.update(candidate=selected, intersection_id='42',
        latitude=0, longitude=0, signal_remaining_s=20, signal_valid=True,
        safe_speed_mps=1.0, measured_speed_mps=speed)
    assert eta is None
    assert machine.state == 'CROSSING_URGENT'


def test_slow_entry_estimate_is_not_raised_to_minimum_speed():
    machine = CrossingStateMachine()
    selected = nearest_crosswalk([crossing(east=0)], -8 / 111320, 0, heading_deg=0)
    _, required, _ = machine.update(candidate=selected, intersection_id='42',
        latitude=-8 / 111320, longitude=0, signal_remaining_s=90, signal_valid=True,
        safe_speed_mps=0.1, measured_speed_mps=0.1)
    assert required == pytest.approx(103)
    assert machine.state == 'WAIT_AT_CURB'
