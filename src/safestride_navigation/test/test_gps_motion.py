import unittest

from safestride_navigation.gps_motion import (
    GpsMotionTracker,
    select_motion_measurement,
)


class TestGpsMotionTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = GpsMotionTracker(
            change_threshold_m=0.5,
            heading_min_move_m=2.0,
            heading_max_step_m=30.0,
        )

    def test_derives_heading_after_real_movement(self):
        self.tracker.update(0.0, 0.0, 0.0)
        self.tracker.update(3.0 / 111_320.0, 0.0, 2.0)
        self.assertAlmostEqual(self.tracker.heading(2.0, 5.0), 0.0, places=2)
        self.assertEqual(self.tracker.heading_source, 'position_delta')

    def test_rmc_course_overrides_position_heading(self):
        self.tracker.update(0.0, 0.0, 0.0)
        self.tracker.set_course(91.0, 1.0)
        self.assertEqual(self.tracker.heading(1.0, 5.0), 91.0)
        self.assertEqual(self.tracker.heading_source, 'rmc_course')

    def test_stationary_position_drift_does_not_create_heading(self):
        self.tracker.update(0.0, 0.0, 0.0, allow_heading=False)
        self.tracker.update(
            8.0 / 111_320.0,
            0.0,
            2.0,
            allow_heading=False,
        )
        self.assertIsNone(self.tracker.heading(2.0, 5.0))
        self.assertEqual(self.tracker.heading_source, 'unavailable')

    def test_heading_anchor_restarts_when_motion_is_confirmed(self):
        self.tracker.update(0.0, 0.0, 0.0, allow_heading=False)
        drifted = 8.0 / 111_320.0
        self.tracker.update(drifted, 0.0, 2.0, allow_heading=False)
        self.tracker.update(
            drifted + 3.0 / 111_320.0,
            0.0,
            4.0,
            allow_heading=True,
        )
        self.assertAlmostEqual(self.tracker.heading(4.0, 5.0), 0.0, places=2)

    def test_reports_stuck_only_when_motion_is_reported(self):
        self.tracker.update(0.0, 0.0, 0.0)
        self.assertFalse(
            self.tracker.coordinates_stuck(
                10.0,
                0.0,
                timeout_s=5.0,
                minimum_speed_mps=0.15,
            )
        )
        self.assertTrue(
            self.tracker.coordinates_stuck(
                10.0,
                0.4,
                timeout_s=5.0,
                minimum_speed_mps=0.15,
            )
        )
        self.tracker.update(1.0 / 111_320.0, 0.0, 10.1)
        self.assertFalse(
            self.tracker.coordinates_stuck(
                10.1,
                0.4,
                timeout_s=5.0,
                minimum_speed_mps=0.15,
            )
        )


class TestMotionSourceSelection(unittest.TestCase):
    def test_wheel_odometry_is_authoritative(self):
        measurement = select_motion_measurement(
            odom_fresh=True,
            odom_speed_mps=0.18,
            wheel_motion_active=True,
            gps_fresh=True,
            gps_speed_mps=0.75,
            allow_gps_speed_fallback=True,
        )
        self.assertEqual(measurement, (0.18, 'wheel_odom', True))

    def test_gps_motion_is_blocked_by_default_without_odometry(self):
        measurement = select_motion_measurement(
            odom_fresh=False,
            odom_speed_mps=None,
            wheel_motion_active=False,
            gps_fresh=True,
            gps_speed_mps=0.75,
            allow_gps_speed_fallback=False,
        )
        self.assertEqual(measurement, (None, 'unavailable', False))

    def test_gps_fallback_requires_explicit_opt_in(self):
        measurement = select_motion_measurement(
            odom_fresh=False,
            odom_speed_mps=None,
            wheel_motion_active=False,
            gps_fresh=True,
            gps_speed_mps=0.30,
            allow_gps_speed_fallback=True,
        )
        self.assertEqual(measurement, (0.30, 'gps_filtered', True))


if __name__ == '__main__':
    unittest.main()
