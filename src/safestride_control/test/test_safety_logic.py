"""Unit tests for ROS-independent SafeStride safety primitives."""

import math
import unittest

from safestride_control.safety_logic import (
    SlopeSpeedPolicy,
    combine_speed_scales,
    finite_parameter,
)


class TestFiniteParameter(unittest.TestCase):

    def test_rejects_nan_and_invalid_bounds(self) -> None:
        for value in (True, math.nan):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    finite_parameter('deceleration', value)
        with self.assertRaises(ValueError):
            finite_parameter(
                'deceleration',
                0.0,
                minimum=0.0,
                minimum_inclusive=False,
            )

    def test_accepts_valid_value(self) -> None:
        self.assertEqual(
            finite_parameter(
                'stop_distance',
                0.35,
                minimum=0.0,
                minimum_inclusive=False,
            ),
            0.35,
        )


class TestSlopeSpeedPolicy(unittest.TestCase):

    def setUp(self) -> None:
        self.policy = SlopeSpeedPolicy(
            enter_angle_rad=math.radians(5.0),
            exit_angle_rad=math.radians(3.0),
            confirmation_time_s=0.5,
            uphill_pitch_sign=1.0,
            pitch_offset_rad=0.0,
            downhill_speed_scale=0.6,
            uphill_speed_scale=1.25,
        )

    def test_slope_must_remain_above_threshold_before_confirmation(
        self,
    ) -> None:
        scale, state, _ = self.policy.update(
            pitch_rad=math.radians(7.0),
            sample_valid=True,
            now_s=1.0,
        )
        self.assertEqual((scale, state), (1.0, SlopeSpeedPolicy.LEVEL))
        scale, state, _ = self.policy.update(
            pitch_rad=math.radians(7.0),
            sample_valid=True,
            now_s=1.49,
        )
        self.assertEqual((scale, state), (1.0, SlopeSpeedPolicy.LEVEL))
        scale, state, _ = self.policy.update(
            pitch_rad=math.radians(7.0),
            sample_valid=True,
            now_s=1.50,
        )
        self.assertEqual((scale, state), (1.25, SlopeSpeedPolicy.UPHILL))

    def test_hysteresis_and_pitch_polarity(self) -> None:
        self.policy.update(
            pitch_rad=math.radians(-7.0),
            sample_valid=True,
            now_s=2.0,
        )
        scale, state, pitch = self.policy.update(
            pitch_rad=math.radians(-7.0),
            sample_valid=True,
            now_s=2.5,
        )
        self.assertEqual((scale, state), (0.6, SlopeSpeedPolicy.DOWNHILL))
        self.assertLess(pitch, 0.0)
        scale, state, _ = self.policy.update(
            pitch_rad=math.radians(-4.0),
            sample_valid=True,
            now_s=3.0,
        )
        self.assertEqual((scale, state), (0.6, SlopeSpeedPolicy.DOWNHILL))

    def test_invalid_sample_resets_to_neutral(self) -> None:
        self.policy.update(
            pitch_rad=math.radians(8.0),
            sample_valid=True,
            now_s=1.0,
        )
        self.policy.update(
            pitch_rad=math.radians(8.0),
            sample_valid=True,
            now_s=1.5,
        )
        scale, state, pitch = self.policy.update(
            pitch_rad=math.nan,
            sample_valid=False,
            now_s=2.0,
        )
        self.assertEqual((scale, state), (1.0, SlopeSpeedPolicy.LEVEL))
        self.assertTrue(math.isnan(pitch))

    def test_slowdown_dominates_assist_when_scales_are_combined(self) -> None:
        self.assertEqual(combine_speed_scales(0.7, 1.25, 1.25), 0.7)
        self.assertEqual(combine_speed_scales(1.2, 1.25, 1.25), 1.25)
        self.assertEqual(combine_speed_scales(0.7, 0.6, 1.25), 0.6)


if __name__ == '__main__':
    unittest.main()
