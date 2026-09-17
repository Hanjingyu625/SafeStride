"""Exercise production status publication without ROS or a serial device."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import yaml

from safestride_bridge.hmi_model import Snapshot
from safestride_bridge.validation import bounded_int, finite_float

ROOT = Path(__file__).resolve().parents[3]


class Status(NS):
    TOF_INVALID = 5
    TOF_RAISED = 3
    TOF_DROP = 4

    def __init__(self):
        super().__init__(header=NS())


source = ROOT / 'src/safestride_bridge/safestride_bridge/terrain_bridge_node.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
tree.body = [item for item in tree.body if isinstance(item, ast.ClassDef)
             or isinstance(item, ast.ImportFrom) and item.module == '__future__']
scope = dict(Node=object, math=math, bounded_int=bounded_int,
             finite_float=finite_float, TerrainStatus=Status)
exec(compile(tree, str(source), 'exec'), scope)
Bridge = scope['TerrainBridgeNode']


class TerrainAttitudeTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/raspberry_pi.yaml').read_text(
            encoding='utf-8'))
        params = {}
        self.node = Bridge.__new__(Bridge)
        self.node.declare_parameter = lambda name, default: params.setdefault(name, default)
        self.node._declare_parameters()
        params.update({'attitude.' + key: value for key, value in
                       self.config['terrain_bridge']['ros__parameters']['attitude'].items()})
        self.params = params
        self.node.get_parameter = lambda name: NS(value=params[name])
        self.node._load_parameters()
        self.node._now = lambda: 1.0
        self.node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
        self.messages = []
        self.node._status_pub = NS(publish=self.messages.append)
        self.node._last_telemetry_time = 1.0

    def publish(self, pitch=-116, roll=-59, valid=True):
        self.node._last_telemetry = NS(
            mpu_pitch_mrad=pitch, mpu_roll_mrad=roll, mpu_valid=valid,
            tof_distance_mm=625, tof_valid=True, tof_filtered_mm=625,
            tof_reference_mm=625, tof_error_mm=0, tof_change_mm=0,
            tof_alert=0, fault_bits=0)
        self.node._publish_status()
        return self.messages[-1]

    def test_level_status_and_hmi_are_zero_without_double_offset(self):
        status = self.publish()
        self.assertAlmostEqual(status.pitch_rad, 0.0)
        self.assertAlmostEqual(status.roll_rad, 0.0)
        self.assertTrue(status.mpu_valid)
        hmi = self.config['terrain_bridge']['ros__parameters']['hmi']
        supervisor = self.config['safety_supervisor']['ros__parameters']
        self.assertEqual(supervisor['pitch_offset_rad'], 0.0)
        self.assertEqual(hmi['pitch_offset_rad'], 0.0)
        model = Snapshot(hmi['pitch_sign'], hmi['pitch_offset_rad'])
        model.update('terrain', status, 1.0)
        self.assertEqual(model.words(1.1)[8], 0)
        # Calibration must not mutate the raw serial sample used by Imu.
        self.assertEqual(self.node._last_telemetry.mpu_pitch_mrad, -116)

    def test_relative_angles_preserve_direction(self):
        status = self.publish(pitch=-216, roll=41)
        self.assertAlmostEqual(status.pitch_rad, -0.1)
        self.assertAlmostEqual(status.roll_rad, 0.1)
        self.assertAlmostEqual(self.publish(pitch=-16).pitch_rad, 0.1)

    def test_invalid_and_missing_samples_remain_invalid(self):
        status = self.publish(valid=False)
        self.assertFalse(status.mpu_valid)
        self.assertTrue(math.isnan(status.pitch_rad))
        self.assertTrue(math.isnan(status.roll_rad))
        self.node._last_telemetry = None
        self.node._publish_status()
        self.assertTrue(math.isnan(self.messages[-1].pitch_rad))

    def test_offsets_are_finite_and_configs_agree(self):
        other = yaml.safe_load((ROOT / 'src/safestride_bringup/config/safestride.yaml')
                               .read_text(encoding='utf-8'))
        self.assertEqual(self.config['terrain_bridge']['ros__parameters']['attitude'],
                         other['terrain_bridge']['ros__parameters']['attitude'])
        for key in ('attitude.pitch_offset_rad', 'attitude.roll_offset_rad'):
            original = self.params[key]
            for invalid in (math.nan, math.inf, 4.0):
                self.params[key] = invalid
                with self.assertRaises(ValueError):
                    self.node._load_parameters()
            self.params[key] = original


if __name__ == '__main__':
    unittest.main()
