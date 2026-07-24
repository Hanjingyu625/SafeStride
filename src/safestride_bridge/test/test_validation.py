"""Tests for fail-fast parameter validation."""

import math
import unittest

from safestride_bridge.validation import bounded_int, finite_float


class TestFiniteFloat(unittest.TestCase):

    def test_accepts_value_inside_bounds(self) -> None:
        self.assertEqual(
            finite_float('radius', 0.15, minimum=0.0),
            0.15,
        )

    def test_rejects_nonfinite_and_nonpositive_values(self) -> None:
        for value in (True, math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    finite_float('radius', value)
        with self.assertRaises(ValueError):
            finite_float(
                'radius',
                0.0,
                minimum=0.0,
                minimum_inclusive=False,
            )


class TestBoundedInt(unittest.TestCase):

    def test_accepts_integer_inside_bounds(self) -> None:
        self.assertEqual(
            bounded_int('ttl', 200, minimum=20, maximum=250),
            200,
        )

    def test_rejects_bool_fraction_and_out_of_range(self) -> None:
        for value in (True, 20.5, 19, 251):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    bounded_int(
                        'ttl',
                        value,
                        minimum=20,
                        maximum=250,
                    )


if __name__ == '__main__':
    unittest.main()
