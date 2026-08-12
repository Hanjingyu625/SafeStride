"""Unit tests for ROS-independent SafeStride safety primitives."""

import math
import unittest

from safestride_control.safety_logic import finite_parameter


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
