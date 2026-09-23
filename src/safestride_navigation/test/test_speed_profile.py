import tempfile
import unittest
from pathlib import Path

from safestride_navigation.speed_profile import UserSpeedProfile, percentile


class TestSpeedProfile(unittest.TestCase):
    def test_waiting_does_not_reduce_learned_walking_speed(self):
        profile = UserSpeedProfile()
        for _ in range(30):
            profile.add(0.7)
        for _ in range(600):
            profile.add(0.0)
            profile.add(0.2, allow_update=False)
        self.assertAlmostEqual(profile.safe_speed(), 0.7)

    def test_slow_walker_is_not_assumed_to_walk_at_point_three(self):
        profile = UserSpeedProfile()
        profile.add(0.16)
        self.assertAlmostEqual(profile.safe_speed(), 0.16)

    def test_profile_uses_median(self):
        self.assertEqual(percentile([1.0, 2.0, 3.0], 0.5), 2.0)
        profile = UserSpeedProfile(default_speed_mps=0.5)
        for speed in (0.4, 0.5, 0.6, 0.7, 0.8):
            profile.add(speed)
        self.assertAlmostEqual(profile.safe_speed(), 0.6)

    def test_default_speed_is_one_until_motion_is_learned(self):
        profile = UserSpeedProfile()
        self.assertEqual(profile.safe_speed(), 1.0)
        profile.add(0.35)
        self.assertAlmostEqual(profile.safe_speed(), 0.35)

    def test_profile_persists_recent_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.json'
            profile = UserSpeedProfile(str(path))
            profile.add(0.45)
            profile.save()
            restored = UserSpeedProfile(str(path))
            self.assertEqual(list(restored.samples), [0.45])

    def test_unconfirmed_motion_is_not_learned(self):
        profile = UserSpeedProfile(default_speed_mps=0.5)
        profile.add(0.75, allow_update=False)
        self.assertEqual(list(profile.samples), [])
        self.assertEqual(profile.safe_speed(), 0.5)


if __name__ == '__main__':
    unittest.main()
