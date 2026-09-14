import math
import unittest
from types import SimpleNamespace as Msg

from safestride_bridge.hmi_model import (
    Snapshot, FORMAT, UNKNOWN, SPEED, HANDS, CROSS, PITCH, WALKER,
    ARMED, DEADMAN, ENTRY_ALLOWED, SIGNAL_VALID,
)


def walker(**overrides):
    values = dict(link_ok=True, state=2, speed_valid=True,
                  measured_speed_kmh=1.25, braking=False, fault_bits=0,
                  armed=True, deadman=True, estop=False, watchdog_timeout=False,
                  telemetry_age=0.02)
    values.update(overrides)
    return Msg(**values)


class HmiModelTests(unittest.TestCase):
    def test_snapshot_wire_and_freshness(self):
        model = Snapshot()
        model.update('walker', walker(), 1.0)
        words = FORMAT.unpack(model.pack(1.1))
        self.assertEqual(words[0], 2)
        self.assertEqual(words[3], 125)
        self.assertEqual(words[2], SPEED | WALKER)
        self.assertEqual(words[15], ARMED | DEADMAN)
        stale = model.words(1.6)
        self.assertEqual(stale[3], UNKNOWN)
        self.assertEqual(stale[15], 0)
        self.assertEqual(stale[2], 0)

    def test_source_age_and_invalid_values(self):
        model = Snapshot()
        for values in (dict(telemetry_age=.7), dict(telemetry_age=math.nan),
                       dict(link_ok=False)):
            model.update('walker', walker(**values), 0)
            self.assertEqual(model.words(.1)[2], 0)
        model.update('walker', walker(measured_speed_kmh=math.nan), 0)
        self.assertFalse(model.words(.1)[2] & SPEED)

    def test_pressure_requires_drive_link(self):
        model = Snapshot()
        model.update('pressure', Msg(calibrated=True, left_present=True,
                                     right_present=True), 0)
        self.assertFalse(model.words(.1)[2] & HANDS)
        model.update('walker', walker(), 0)
        self.assertEqual(model.words(.1)[4], 3)

    def test_signal_permission_is_explicit(self):
        model = Snapshot()
        cross = Msg(gps_valid=True, state=3, signal_valid=True,
                    entry_allowed=False, urgent=False, edge_distance_m=4.2,
                    signal_remaining_s=6.4)
        model.update('crosswalk', cross, 0)
        self.assertFalse(model.words(.1)[15] & ENTRY_ALLOWED)
        cross.entry_allowed = True
        words = model.words(.1)
        self.assertEqual(words[15], ENTRY_ALLOWED | SIGNAL_VALID)
        self.assertEqual(words[6:8], [6, 42])
        cross.signal_valid = False
        self.assertFalse(model.words(.1)[15] & ENTRY_ALLOWED)
        self.assertEqual(model.words(.1)[6], UNKNOWN)
        self.assertFalse(model.words(1.6)[2] & CROSS)

    def test_pitch_and_hazard_latch(self):
        model = Snapshot(-1, .01)
        terrain = Msg(mpu_valid=True, pitch_rad=-.1, tof_valid=True,
                      tof_alert=3, terrain_hazard=True)
        model.update('terrain', terrain, 0)
        self.assertTrue(model.words(.1)[2] & PITCH)
        self.assertEqual(model.words(.1)[8], 63)
        terrain.tof_valid = False
        terrain.terrain_hazard = False
        self.assertEqual(model.words(.2)[10], 1)
        self.assertEqual(model.words(2)[10], 1)
        terrain.tof_valid = True
        terrain.tof_alert = 0
        model.update('terrain', terrain, 2)
        self.assertEqual(model.words(2.1)[10], 0)

    def test_clock_rewind_is_not_fresh(self):
        model = Snapshot()
        model.update('walker', walker(), 5)
        self.assertEqual(model.words(4)[2], 0)


if __name__ == '__main__':
    unittest.main()
