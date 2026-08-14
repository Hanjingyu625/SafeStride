import unittest

from safestride_navigation.crossing_policy import CrossingStateMachine
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
