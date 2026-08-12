import unittest

from safestride_navigation.signal_logic import (
    latest_signal_record,
    signal_remaining_for_crosswalk,
)


class TestSignalLogic(unittest.TestCase):
    def test_latest_record_and_conservative_opposing_value(self):
        document = {
            'items': [
                {
                    'itstId': '42',
                    'trsmUtcTime': '100',
                    'ntPdsgRmdrCs': 180,
                },
                {
                    'itstId': '42',
                    'trsmUtcTime': '200',
                    'ntPdsgRmdrCs': 150,
                    'stPdsgRmdrCs': 120,
                },
            ]
        }
        record = latest_signal_record(document, '42')
        (seconds, field), raw = signal_remaining_for_crosswalk(record, 'nt')
        self.assertEqual(seconds, 12.0)
        self.assertEqual(field, 'stPdsgRmdrCs')
        self.assertEqual(raw['ntPdsgRmdrCs'], 150)

    def test_invalid_sentinel_fails_closed(self):
        with self.assertRaises(ValueError):
            signal_remaining_for_crosswalk(
                {'ntPdsgRmdrCs': 36000, 'stPdsgRmdrCs': -1},
                'nt',
            )

    def test_unrelated_intersection_is_not_silently_accepted(self):
        with self.assertRaises(ValueError):
            latest_signal_record(
                {'itstId': 'wrong', 'trsmUtcTime': '1'},
                'expected',
            )


if __name__ == '__main__':
    unittest.main()
