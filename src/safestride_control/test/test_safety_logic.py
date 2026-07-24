"""Unit tests for ROS-independent SafeStride safety primitives."""

import math
import unittest

from safestride_control.safety_logic import (
    PostArmNeutralGate,
    apply_command_deadband,
    finite_parameter,
)


class TestPostArmNeutralGate(unittest.TestCase):

    def setUp(self) -> None:
        self.gate = PostArmNeutralGate(0.01, 0.02)
        self.gate.require_new_neutral(5)

    def test_cached_or_deflected_input_cannot_open_gate(self) -> None:
        self.assertFalse(self.gate.observe(5, 0.0, 0.0))
        self.assertFalse(self.gate.observe(6, 0.2, 0.0))
        self.assertFalse(self.gate.neutral_seen)

    def test_new_neutral_sample_qualifies_but_is_held_one_cycle(self) -> None:
        self.assertFalse(self.gate.observe(6, 0.005, -0.01))
        self.assertTrue(self.gate.neutral_seen)
        self.assertTrue(self.gate.observe(6, 0.005, -0.01))
        self.assertTrue(self.gate.observe(7, 0.2, 0.1))

    def test_reset_requires_another_new_neutral_sample(self) -> None:
        self.gate.observe(6, 0.0, 0.0)
        self.gate.require_new_neutral(8)
        self.assertFalse(self.gate.observe(8, 0.0, 0.0))
        self.assertFalse(self.gate.observe(9, 0.2, 0.0))
        self.assertFalse(self.gate.observe(10, math.nan, 0.0))
        self.assertFalse(self.gate.observe(11, 0.0, 0.0))
        self.assertTrue(self.gate.neutral_seen)


class TestCommandDeadband(unittest.TestCase):

    def test_axes_inside_deadband_become_exact_zero(self) -> None:
        self.assertEqual(
            apply_command_deadband(0.009, -0.019, 0.01, 0.02),
            (0.0, 0.0),
        )

    def test_axes_outside_deadband_are_unchanged(self) -> None:
        self.assertEqual(
            apply_command_deadband(-0.011, 0.021, 0.01, 0.02),
            (-0.011, 0.021),
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


if __name__ == '__main__':
    unittest.main()
