import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / 'pi'))
from hmi_model import Snapshot, UNKNOWN, SPEED, HANDS, CROSS, PITCH, TOF, WALKER


class Obj: pass


class HmiModelTests(unittest.TestCase):
    def test_stale_sources_are_unknown(self):
        s = Snapshot(); w = Obj(); w.link_ok = True; w.state = 2
        w.speed_valid = True; w.measured_speed_kmh = 1.25; w.braking = False; w.fault_bits = 0
        s.update('walker', w, 0.0)
        words = s.words(0.7)
        self.assertEqual(words[3], UNKNOWN)
        self.assertFalse(words[2] & SPEED)

    def test_normal_snapshot_contains_valid_fields(self):
        s = Snapshot()
        w = Obj(); w.link_ok=True; w.state=2; w.speed_valid=True; w.measured_speed_kmh=1.25; w.braking=False; w.fault_bits=0
        p = Obj(); p.calibrated=True; p.left_present=True; p.right_present=False
        c = Obj(); c.gps_valid=True; c.state=1; c.edge_distance_m=4.2; c.signal_valid=True; c.signal_remaining_s=6.4
        t = Obj(); t.mpu_valid=True; t.pitch_rad=0.1; t.tof_valid=True; t.tof_alert=0; t.terrain_hazard=False
        for k, v in [('walker', w), ('pressure', p), ('crosswalk', c), ('terrain', t)]: s.update(k, v, 1.0)
        words = s.words(1.1)
        self.assertTrue(words[2] & (SPEED|HANDS|CROSS|PITCH|TOF|WALKER))
        self.assertEqual(words[3], 125)
        self.assertEqual(words[6], 6)

    def test_confirmed_hazard_latches_through_invalid_terrain(self):
        s = Snapshot(); t = Obj(); t.mpu_valid=False; t.tof_valid=True; t.tof_alert=3; t.terrain_hazard=True
        s.update('terrain', t, 0.0); self.assertEqual(s.words(0.1)[10], 1)
        t.tof_valid=False; t.tof_alert=5; t.terrain_hazard=False; s.update('terrain', t, 0.2)
        self.assertEqual(s.words(0.3)[10], 1)


if __name__ == '__main__': unittest.main()
