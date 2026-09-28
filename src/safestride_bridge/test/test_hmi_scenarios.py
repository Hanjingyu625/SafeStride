"""PC-only operating scenarios for the ROS-topic-to-HMI register contract."""

import math
import unittest
from types import SimpleNamespace as Msg

from safestride_bridge.hmi_model import FORMAT, Snapshot, UNKNOWN


def walker(**overrides):
    values = dict(
        link_ok=True,
        state=2,
        speed_valid=True,
        measured_speed_kmh=1.25,
        braking=False,
        fault_bits=0,
        armed=True,
        deadman=True,
        estop=False,
        watchdog_timeout=False,
        telemetry_age=0.02,
    )
    values.update(overrides)
    return Msg(**values)


def pressure():
    return Msg(calibrated=True, left_present=True, right_present=True)


def crosswalk(
    state,
    distance,
    seconds=0,
    signal_valid=False,
    entry_allowed=False,
):
    return Msg(
        gps_valid=True,
        state=state,
        signal_valid=signal_valid,
        entry_allowed=entry_allowed,
        urgent=False,
        edge_distance_m=distance,
        signal_remaining_s=seconds,
    )


def terrain(pitch_degrees=0, tof_alert=0, hazard=False):
    return Msg(
        mpu_valid=True,
        pitch_rad=math.radians(pitch_degrees),
        tof_valid=True,
        tof_alert=tof_alert,
        terrain_hazard=hazard,
    )


def populated_snapshot(walker_message, crosswalk_message, terrain_message):
    model = Snapshot()
    model.update("walker", walker_message, 0.0)
    model.update("pressure", pressure(), 0.0)
    model.update("crosswalk", crosswalk_message, 0.0)
    model.update("terrain", terrain_message, 0.0)
    return model


class HmiOperatingScenarioTests(unittest.TestCase):
    def test_six_operating_scenarios_encode_exact_register_words(self):
        # Register order is status 0..15 plus packed location 16..25.
        scenarios = {
            "ready": (
                populated_snapshot(
                    walker(),
                    crosswalk(3, 4.2, 15, signal_valid=True, entry_allowed=True),
                    terrain(),
                ),
                0.1,
                (3, 1, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83) + (0,) * 10,
            ),
            "uphill": (
                populated_snapshot(
                    walker(measured_speed_kmh=0.85),
                    crosswalk(2, 3.5),
                    terrain(8.5),
                ),
                0.1,
                (3, 1, 63, 85, 3, 2, UNKNOWN, 35, 85, 0, 0, 2, 0, 0, 1, 3) + (0,) * 10,
            ),
            "downhill": (
                populated_snapshot(
                    walker(measured_speed_kmh=0.70),
                    crosswalk(4, 2.1, 9, signal_valid=True),
                    terrain(-7.0),
                ),
                0.1,
                (3, 1, 63, 70, 3, 4, 9, 21, 65466, 0, 0, 2, 0, 0, 1, 67) + (0,) * 10,
            ),
            "braking": (
                populated_snapshot(
                    walker(measured_speed_kmh=0.20, braking=True),
                    crosswalk(2, 1.5, 5, signal_valid=True),
                    terrain(),
                ),
                0.1,
                (3, 1, 63, 20, 3, 2, 5, 15, 0, 0, 0, 2, 1, 0, 1, 67) + (0,) * 10,
            ),
            "hazard": (
                populated_snapshot(
                    walker(measured_speed_kmh=0.0),
                    crosswalk(2, 1.5),
                    terrain(tof_alert=3, hazard=True),
                ),
                0.1,
                (3, 1, 63, 0, 3, 2, UNKNOWN, 15, 0, 3, 1, 2, 0, 0, 1, 3) + (0,) * 10,
            ),
            "stale_topics": (
                populated_snapshot(walker(), crosswalk(3, 4.2), terrain()),
                2.0,
                (3, 1, 0, UNKNOWN, 0, 0, UNKNOWN, UNKNOWN, 0, 5, 0, 0, 0, 0, 1, 0) + (0,) * 10,
            ),
        }

        for name, (model, now, expected) in scenarios.items():
            with self.subTest(name=name):
                payload = model.pack(now)
                self.assertEqual(len(payload), 52)
                self.assertEqual(FORMAT.unpack(payload), expected)


if __name__ == "__main__":
    unittest.main()
