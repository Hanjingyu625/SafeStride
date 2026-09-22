import unittest
from unittest.mock import patch

from safestride_navigation.signal_logic import evaluate_pedestrian_signal, request_signal_bundle
from safestride_navigation.gps_motion import StationaryPosition


class SignalPhaseTests(unittest.TestCase):
    def setUp(self):
        self.timing = {'itstId': '42', 'trsmUtcTime': 1000000, 'ntPdsgRmdrCs': 260}
        self.phase = {'itstId': '42', 'trsmUtcTime': 1000000,
                      'ntPdsgStatNm': 'protected-Movement-Allowed'}

    def evaluate(self, now=1001):
        return evaluate_pedestrian_signal(self.timing, self.phase, 'nt', now)

    def test_green_countdown_includes_source_age(self):
        self.assertEqual(self.evaluate()[:2], (25.0, True))

    def test_red_does_not_require_countdown(self):
        self.phase['ntPdsgStatNm'] = 'stop-And-Remain'
        self.timing = None
        self.assertEqual(self.evaluate(), (0.0, True, 'red pedestrian signal'))

    def test_countdown_alone_does_not_imply_green(self):
        self.phase = None
        self.assertFalse(self.evaluate()[1])

    def test_opposite_head_is_not_substituted(self):
        self.timing['stPdsgRmdrCs'] = self.timing.pop('ntPdsgRmdrCs')
        self.assertFalse(self.evaluate()[1])

    def test_stale_future_unknown_mismatched_records(self):
        self.assertFalse(self.evaluate(1013)[1])
        self.assertFalse(self.evaluate(998)[1])
        self.timing['itstId'] = '43'
        self.assertFalse(self.evaluate()[1])
        self.timing['itstId'] = '42'
        self.timing['trsmUtcTime'] = 996000
        self.assertFalse(self.evaluate()[1])
        self.phase['ntPdsgStatNm'] = 'unknown'
        self.assertFalse(self.evaluate()[1])

    def test_independent_phase_survives_timing_failure(self):
        def fetch(_key, _intersection, *, url, timeout_s):
            if url == 'timing':
                raise RuntimeError('timing unavailable')
            return self.phase

        with patch('safestride_navigation.signal_logic.request_signal_data', side_effect=fetch):
            result = request_signal_bundle('test', '42', url='timing',
                                           phase_url='phase', timeout_s=1)
        self.assertIsNone(result['timing'])
        self.assertEqual(result['phase'], self.phase)
        self.assertEqual(result['timing_error'], 'timing unavailable')


class StationaryPositionTests(unittest.TestCase):
    def test_hold_and_release(self):
        hold = StationaryPosition()
        self.assertEqual(hold.update(37, 127, 0, False), (37, 127))
        self.assertEqual(hold.update(37.00001, 127, 1, True), (37, 127))
        self.assertTrue(hold.held)
        self.assertEqual(hold.update(37.00002, 127, 2, False), (37.00002, 127))
        self.assertFalse(hold.held)

    def test_outage_reacquires_instead_of_holding_old_fix(self):
        hold = StationaryPosition()
        hold.update(37, 127, 0, True)
        self.assertEqual(hold.update(38, 127, 4, True), (38, 127))
