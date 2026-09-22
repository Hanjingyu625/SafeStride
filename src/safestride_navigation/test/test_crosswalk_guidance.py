import unittest

from safestride_navigation.crosswalk_guidance import describe_crosswalk


class GuidanceTest(unittest.TestCase):
    def test_entry_and_crossing(self):
        for state in ('ENTRY_ALLOWED', 'CROSSING'):
            self.assertEqual(describe_crosswalk(state, '', True, True)['state'],
                             '건널 수 있음')

    def test_red_and_short_window_wait(self):
        for reason in ('wait; pedestrian signal is red', 'wait for a safer signal window'):
            self.assertEqual(describe_crosswalk('WAIT_AT_CURB', reason, True, True)['state'],
                             '기다려')

    def test_signal_loss_does_not_instruct_hurrying_or_stopping_in_road(self):
        result = describe_crosswalk('CROSSING_URGENT', '', True, False)
        self.assertEqual(result['state'], '신호 없음')
        self.assertIn('횡단 중', result['reason'])

    def test_timeout_is_not_reported_as_short_signal(self):
        result = describe_crosswalk('CROSSING_URGENT',
                                    'crossing timeout; continue assistance and alert', True, True)
        self.assertEqual(result['state'], '주의')
        self.assertIn('완료를 확인하지 못함', result['reason'])

    def test_short_signal_explains_urgency(self):
        result = describe_crosswalk('CROSSING_URGENT',
                                    'continue crossing; remaining signal is tight', True, True)
        self.assertIn('신속히', result['reason'])

    def test_inactive_and_complete_are_not_crossing_permission(self):
        for state in ('IDLE', 'EXITING'):
            self.assertEqual(describe_crosswalk(state, '', True, False)['state'], '안내 없음')

    def test_gps_loss_overrides_permission(self):
        self.assertEqual(describe_crosswalk('ENTRY_ALLOWED', '', False, True)['state'], '안내 없음')
        self.assertEqual(describe_crosswalk('CROSSING', '', False, True)['state'], '주의')
