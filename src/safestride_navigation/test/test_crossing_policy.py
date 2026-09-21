import unittest

from safestride_navigation.crossing_policy import (
    CrossingParameters, CrossingStateMachine,
)
from safestride_navigation.crosswalk_data import nearest_crosswalk


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def candidate(latitude):
    record = {
        'index': 0,
        'latitude': 0.0,
        'longitude': 0.0,
        'length_m': 10.0,
        'width_m': 3.0,
        'axis_bearing_deg': 0.0,
        'intersection_id': '42',
    }
    return nearest_crosswalk([record], latitude, 0.0)


def update(machine, latitude, signal_s, signal_valid=True, speed=0.5):
    return machine.update(
        candidate=candidate(latitude),
        intersection_id='42',
        latitude=latitude,
        longitude=0.0,
        signal_remaining_s=signal_s,
        signal_valid=signal_valid,
        safe_speed_mps=0.5,
        measured_speed_mps=speed,
        wheel_distance_m=None,
    )


class TestCrossingPolicy(unittest.TestCase):
    def test_nominal_drive_speed_is_not_reduced_by_crosswalk_cap(self):
        machine = CrossingStateMachine(clock=FakeClock())
        for state in ('IDLE', 'ENTRY_ALLOWED', 'CROSSING'):
            with self.subTest(state=state):
                machine.set_state(state, 'test')
                self.assertAlmostEqual(
                    machine.command(1.0, 1.0)['target_speed_mps'], 1.0)
        machine.set_state('CROSSING_URGENT', 'remaining signal is tight')
        self.assertAlmostEqual(
            machine.command(1.0, 1.0)['target_speed_mps'], 1.1)

    def test_fast_feedback_remains_bounded_by_drive_ceiling(self):
        machine = CrossingStateMachine(clock=FakeClock())
        for state in ('CROSSING', 'CROSSING_URGENT'):
            with self.subTest(state=state):
                machine.set_state(state, 'test')
                self.assertAlmostEqual(
                    machine.command(1.0, 1.5)['target_speed_mps'], 1.15)

    def test_custom_cap_and_curb_stop_still_apply(self):
        machine = CrossingStateMachine(
            CrossingParameters(maximum_assist_speed_mps=0.7),
            clock=FakeClock())
        machine.set_state('CROSSING_URGENT', 'test')
        self.assertAlmostEqual(
            machine.command(1.0, 1.0)['target_speed_mps'], 0.7)
        machine.set_state('WAIT_AT_CURB', 'red signal')
        self.assertEqual(machine.command(1.0, 1.0)['target_speed_mps'], 0.0)

    def test_slow_profile_is_not_replaced_with_nominal_speed(self):
        machine = CrossingStateMachine(clock=FakeClock())
        machine.set_state('CROSSING', 'test')
        self.assertAlmostEqual(
            machine.command(0.35, 0.35)['target_speed_mps'], 0.35)
        machine.set_state('CROSSING_URGENT', 'test')
        self.assertAlmostEqual(
            machine.command(0.35, 0.35)['target_speed_mps'], 0.45)

    def test_entry_time_uses_three_seconds_and_actual_profile_speed(self):
        for speed, seconds, expected in (
            (1.0, 26.0, 'ENTRY_ALLOWED'),
            (1.0, 18.0, 'ENTRY_ALLOWED'),
            (0.35, 18.0, 'WAIT_AT_CURB'),
            (1.0, 0.0, 'WAIT_AT_CURB'),
        ):
            with self.subTest(speed=speed, seconds=seconds):
                machine = CrossingStateMachine(clock=FakeClock())
                latitude = -8 / 111320.0
                _, required, _ = machine.update(
                    candidate=candidate(latitude), intersection_id='42',
                    latitude=latitude, longitude=0.0,
                    signal_remaining_s=seconds, signal_valid=True,
                    safe_speed_mps=speed, measured_speed_mps=0.0,
                    wheel_distance_m=0.0)
                self.assertAlmostEqual(required, 10.0 / speed + 3.0)
                self.assertEqual(machine.state, expected)

    def wheel_update(self, machine, north_m, wheel_m, signal=40.0, east_m=0.0):
        latitude = north_m / 111_320.0
        return machine.update(
            candidate=candidate(latitude), intersection_id='42',
            latitude=latitude, longitude=east_m / 111_320.0,
            signal_remaining_s=signal, signal_valid=signal is not None,
            safe_speed_mps=0.5, measured_speed_mps=0.5,
            wheel_distance_m=wheel_m)

    def test_wheel_movement_before_curb_does_not_start_crossing(self):
        machine = CrossingStateMachine(clock=FakeClock())
        self.wheel_update(machine, -8, 0, None)
        self.wheel_update(machine, -8, 0, None)
        self.wheel_update(machine, -8, 2, None)
        self.assertEqual(machine.state, 'WAIT_AT_CURB')

    def test_sidewalk_motion_outside_corridor_does_not_start_crossing(self):
        machine = CrossingStateMachine(clock=FakeClock())
        self.wheel_update(machine, -5, 0, None)
        self.wheel_update(machine, -5, 0, None)
        self.wheel_update(machine, 0, 6, None, east_m=6)
        self.assertEqual(machine.state, 'WAIT_AT_CURB')

    def test_gps_jump_cannot_complete_with_short_wheel_distance(self):
        clock = FakeClock()
        machine = CrossingStateMachine(clock=clock)
        self.wheel_update(machine, -5, 0)
        self.wheel_update(machine, -5, 0)
        self.wheel_update(machine, -3.5, 1.5)
        self.assertEqual(machine.state, 'CROSSING')
        self.wheel_update(machine, 7, 1.5)
        clock.advance(3)
        self.wheel_update(machine, 7, 1.5)
        self.assertEqual(machine.state, 'CROSSING')
        self.wheel_update(machine, 7, 12)
        clock.advance(2.1)
        self.wheel_update(machine, 7, 12)
        self.assertEqual(machine.state, 'EXITING')

    def test_signal_loss_and_short_time_have_distinct_reasons(self):
        machine = CrossingStateMachine(clock=FakeClock())
        update(machine, -5 / 111_320, 40)
        update(machine, -5 / 111_320, 40)
        update(machine, -3.5 / 111_320, 40)
        update(machine, -3.5 / 111_320, None, signal_valid=False)
        self.assertIn('signal data unavailable', machine.reason)
        update(machine, -3.5 / 111_320, 5)
        self.assertIn('remaining signal is tight', machine.reason)
        update(machine, -3.5 / 111_320, 26)
        self.assertEqual(machine.state, 'CROSSING')
        update(machine, -3.5 / 111_320, 18)
        self.assertEqual(machine.state, 'CROSSING_URGENT')

    def test_sufficient_signal_allows_entry_and_starts_crossing(self):
        clock = FakeClock()
        machine = CrossingStateMachine(clock=clock)
        south_curb = -5.0 / 111_320.0
        update(machine, south_curb, 30.0)
        self.assertEqual(machine.state, 'ENTRY_ALLOWED')
        update(machine, south_curb, 30.0)
        update(machine, -3.5 / 111_320.0, 30.0)
        self.assertEqual(machine.state, 'CROSSING')
        self.assertGreater(
            machine.command(0.5, 0.5)['target_speed_mps'],
            0.0,
        )

    def test_entry_while_waiting_becomes_urgent(self):
        clock = FakeClock()
        machine = CrossingStateMachine(clock=clock)
        south_curb = -5.0 / 111_320.0
        update(machine, south_curb, None, signal_valid=False)
        self.assertEqual(machine.state, 'WAIT_AT_CURB')
        update(machine, south_curb, None, signal_valid=False)
        update(machine, -3.5 / 111_320.0, None, signal_valid=False)
        self.assertEqual(machine.state, 'CROSSING_URGENT')
        command = machine.command(0.5, 0.4)
        self.assertEqual(command['mode'], 'CROSSING_URGENT')
        self.assertGreater(command['target_speed_mps'], 0.0)

    def test_crossing_exit_requires_stable_clearance(self):
        clock = FakeClock()
        machine = CrossingStateMachine(clock=clock)
        south_curb = -5.0 / 111_320.0
        update(machine, south_curb, 30.0)
        update(machine, south_curb, 30.0)
        update(machine, -3.5 / 111_320.0, 30.0)
        far_clear = 6.6 / 111_320.0
        update(machine, far_clear, 30.0)
        self.assertEqual(machine.state, 'CROSSING')
        clock.advance(2.1)
        update(machine, far_clear, 30.0)
        self.assertEqual(machine.state, 'EXITING')
        clock.advance(4.1)
        update(machine, far_clear, 30.0)
        self.assertEqual(machine.state, 'IDLE')


if __name__ == '__main__':
    unittest.main()
