import math
import unittest

from safestride_sensors.gps_speed_filter import GpsSpeedFilter


METRES_PER_DEGREE = 111_320.0


def latitude_offset(metres):
    return metres / METRES_PER_DEGREE


def make_speed_filter(**kwargs):
    kwargs.setdefault('settling_time_s', 0.0)
    return GpsSpeedFilter(**kwargs)


class TestGpsSpeedFilter(unittest.TestCase):
    def test_stationary_jitter_does_not_become_walking_speed(self):
        speed_filter = make_speed_filter()
        jitter_m = (
            0.0, 0.7, -0.5, 0.8, -0.6, 0.4, -0.2, 0.3, 0.0, -0.4,
        )
        raw_speeds = (
            0.16, 0.51, 1.37, 0.47, 0.83, 0.67, 1.04, 0.42, 0.73, 0.30,
        )
        courses = (
            55.0, 175.0, 202.0, 31.0, 229.0,
            95.0, 187.0, 23.0, 212.0, 110.0,
        )

        estimates = []
        for second, (jitter, raw_speed, course) in enumerate(
            zip(jitter_m, raw_speeds, courses)
        ):
            estimates.append(
                speed_filter.update(
                    time_s=float(second),
                    latitude=37.54 + latitude_offset(jitter),
                    longitude=127.08,
                    speed_mps=raw_speed,
                    course_deg=course,
                    hdop=2.2,
                    satellites=6,
                )
            )

        self.assertFalse(any(estimate.moving for estimate in estimates))
        self.assertEqual(estimates[-1].filtered_speed_mps, 0.0)
        self.assertEqual(estimates[-1].state, 'stationary')

    def test_sustained_slow_walk_is_preserved_after_confirmation(self):
        speed_filter = make_speed_filter()
        estimates = []
        for second in range(14):
            estimates.append(
                speed_filter.update(
                    time_s=float(second),
                    latitude=37.54 + latitude_offset(0.20 * second),
                    longitude=127.08,
                    speed_mps=0.20 + (0.01 if second % 2 else -0.01),
                    course_deg=0.0,
                    hdop=1.2,
                    satellites=8,
                )
            )

        moving = [estimate for estimate in estimates if estimate.moving]
        self.assertTrue(moving)
        self.assertGreaterEqual(moving[0].window_span_s, 4.0)
        self.assertAlmostEqual(
            estimates[-1].filtered_speed_mps,
            0.20,
            delta=0.03,
        )

    def test_poor_fix_quality_cannot_confirm_false_motion(self):
        speed_filter = make_speed_filter()
        estimate = None
        for second in range(12):
            estimate = speed_filter.update(
                time_s=float(second),
                latitude=37.54 + latitude_offset(0.8 * second),
                longitude=127.08,
                speed_mps=1.2,
                course_deg=0.0,
                hdop=9.7,
                satellites=4,
            )

        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.moving)
        self.assertEqual(estimate.filtered_speed_mps, 0.0)
        self.assertEqual(estimate.state, 'degraded')

    def test_coherent_position_drift_disagrees_with_doppler_speed(self):
        speed_filter = make_speed_filter()
        raw_speeds = (0.17, 0.07, 0.03, 0.19, 0.06, 0.11, 0.06, 0.08, 0.03)
        estimate = None
        for second, raw_speed in enumerate(raw_speeds):
            estimate = speed_filter.update(
                time_s=float(second),
                latitude=37.54 + latitude_offset(0.80 * second),
                longitude=127.08,
                speed_mps=raw_speed,
                hdop=1.53,
                satellites=8,
            )

        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.moving)
        self.assertEqual(estimate.filtered_speed_mps, 0.0)
        self.assertLess(estimate.speed_agreement, 0.20)
        self.assertAlmostEqual(estimate.position_speed_mps, 0.80, delta=0.01)

    def test_speed_agreement_threshold_is_configurable(self):
        speed_filter = make_speed_filter(minimum_speed_agreement=0.10)
        estimate = None
        for second in range(10):
            estimate = speed_filter.update(
                time_s=float(second),
                latitude=37.54 + latitude_offset(0.80 * second),
                longitude=127.08,
                speed_mps=0.10,
                hdop=1.5,
                satellites=8,
            )

        self.assertIsNotNone(estimate)
        self.assertTrue(estimate.moving)

    def test_single_speed_spike_is_rejected(self):
        speed_filter = make_speed_filter()
        estimate = None
        for second, speed in enumerate(
            (0.0, 0.0, 2.8, 0.0, 0.0, 0.0, 0.0)
        ):
            estimate = speed_filter.update(
                time_s=float(second),
                latitude=37.54,
                longitude=127.08,
                speed_mps=speed,
                hdop=1.0,
                satellites=9,
            )

        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.moving)
        self.assertEqual(estimate.filtered_speed_mps, 0.0)

    def test_missing_quality_cannot_confirm_motion(self):
        speed_filter = make_speed_filter()
        estimate = None
        for second in range(12):
            estimate = speed_filter.update(
                time_s=float(second),
                latitude=37.54 + latitude_offset(0.3 * second),
                longitude=127.08,
                speed_mps=0.3,
                course_deg=0.0,
            )

        self.assertIsNotNone(estimate)
        self.assertFalse(estimate.moving)
        self.assertEqual(estimate.state, 'degraded')

    def test_outlier_above_maximum_is_bounded_without_crashing(self):
        speed_filter = make_speed_filter()
        estimate = speed_filter.update(
            time_s=0.0,
            latitude=37.54,
            longitude=127.08,
            speed_mps=25.0,
            hdop=1.0,
            satellites=9,
        )
        self.assertEqual(estimate.raw_speed_mps, 25.0)
        self.assertEqual(estimate.filtered_speed_mps, 0.0)

    def test_rejects_non_monotonic_time(self):
        speed_filter = make_speed_filter()
        speed_filter.update(
            time_s=1.0,
            latitude=37.54,
            longitude=127.08,
            speed_mps=0.0,
        )
        with self.assertRaises(ValueError):
            speed_filter.update(
                time_s=1.0,
                latitude=37.54,
                longitude=127.08,
                speed_mps=0.0,
            )

    def test_course_coherence_is_nan_without_enough_courses(self):
        speed_filter = make_speed_filter()
        estimate = speed_filter.update(
            time_s=0.0,
            latitude=37.54,
            longitude=127.08,
            speed_mps=0.0,
        )
        self.assertTrue(math.isnan(estimate.course_coherence))

    def test_settling_period_blocks_cold_start_false_motion(self):
        speed_filter = GpsSpeedFilter(settling_time_s=15.0)
        estimates = []
        for second in range(15):
            estimates.append(
                speed_filter.update(
                    time_s=float(second),
                    latitude=37.54 + latitude_offset(0.60 * second),
                    longitude=127.08,
                    speed_mps=0.60,
                    course_deg=0.0,
                    hdop=1.5,
                    satellites=8,
                )
            )

        self.assertFalse(any(estimate.moving for estimate in estimates))
        self.assertEqual(estimates[-1].state, 'settling')
        self.assertEqual(estimates[-1].filtered_speed_mps, 0.0)


if __name__ == '__main__':
    unittest.main()
