"""Static checks for production MCU pin ownership and disabled placeholders."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
DRIVE_CONFIG = ROOT / "firmware/safestride_mcu/config.h"
DRIVE_FIRMWARE = ROOT / "firmware/safestride_mcu/safestride_mcu.ino"
TERRAIN_FIRMWARE = ROOT / "firmware/terrain_mcu/terrain_mcu.ino"
TERRAIN_CONFIG = ROOT / "firmware/terrain_mcu/config.h"
DRIVE_SKETCH_DIR = ROOT / "firmware/safestride_mcu"
TERRAIN_SKETCH_DIR = ROOT / "firmware/terrain_mcu"
BRIDGE = (
    ROOT
    / "src/safestride_bridge/safestride_bridge/serial_bridge_node.py"
)
ROS_CONFIGS = (
    ROOT / "config/raspberry_pi.yaml",
    ROOT / "src/safestride_bringup/config/safestride.yaml",
)


def constant_expression(text: str, name: str) -> str:
    match = re.search(
        rf"constexpr\s+[^;=]+\s+{re.escape(name)}\s*=\s*([^;]+);",
        text,
    )
    if match is None:
        raise AssertionError(f"missing constant: {name}")
    return re.sub(r"\s+", "", match.group(1))


class TestHardwareIntegrity(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.config = DRIVE_CONFIG.read_text(encoding="utf-8")
        cls.drive = DRIVE_FIRMWARE.read_text(encoding="utf-8")
        cls.terrain = TERRAIN_FIRMWARE.read_text(encoding="utf-8")
        cls.terrain_config = TERRAIN_CONFIG.read_text(encoding="utf-8")
        cls.bridge = BRIDGE.read_text(encoding="utf-8")

    def test_drive_active_pins_are_unique(self):
        names = (
            "LEFT_HALL_PIN",
            "RIGHT_HALL_PIN",
            "MOTOR_PWM_PIN",
            "MOTOR_IN1_PIN",
            "MOTOR_IN2_PIN",
            "PRESSURE_LEFT_PIN",
            "PRESSURE_RIGHT_PIN",
        )
        owners = {
            name: constant_expression(self.config, name) for name in names
        }
        self.assertEqual(
            len(set(owners.values())),
            len(owners),
            f"Drive Uno pin collision: {owners}",
        )
        self.assertNotIn(
            "0U", owners.values(), "D0 is reserved for USB serial"
        )
        self.assertNotIn(
            "1U", owners.values(), "D1 is reserved for USB serial"
        )

    def test_production_sketches_have_one_ino_entry_point(self):
        expected = {
            DRIVE_SKETCH_DIR: "safestride_mcu.ino",
            TERRAIN_SKETCH_DIR: "terrain_mcu.ino",
        }
        for directory, primary_name in expected.items():
            with self.subTest(directory=directory):
                ino_files = sorted(
                    path.name for path in directory.glob("*.ino")
                )
                self.assertEqual(ino_files, [primary_name])

    def test_optional_inputs_do_not_overlap_each_other(self):
        names = (
            "ESTOP_PIN",
            "DRIVER_FAULT_PIN",
            "BATTERY_SENSE_PIN",
            "LEFT_CURRENT_SENSE_PIN",
            "RIGHT_CURRENT_SENSE_PIN",
        )
        pins = [constant_expression(self.config, name) for name in names]
        self.assertEqual(len(set(pins)), len(pins))

    def test_estop_is_unimplemented_and_reports_normal(self):
        self.assertEqual(
            constant_expression(self.config, "ENABLE_ESTOP"), "false"
        )
        self.assertIn("if (!cfg::ENABLE_ESTOP)", self.drive)
        self.assertRegex(
            self.drive,
            r"if\s*\(cfg::ENABLE_ESTOP\)\s*\{\s*"
            r"pinMode\(cfg::ESTOP_PIN,\s*INPUT_PULLUP\);",
        )
        self.assertRegex(
            self.drive,
            r"if\s*\(cfg::ENABLE_ESTOP\)\s*\{\s*"
            r"capabilities\s*\|=\s*CAP_ESTOP;",
        )

    def test_normal_hall_feedback_mode_is_enabled(self):
        self.assertEqual(
            constant_expression(self.config, "HALL_CALIBRATED"),
            "true",
        )
        pulses_per_revolution = int(
            constant_expression(self.config, "HALL_PULSES_PER_WHEEL_REV")
            .removesuffix("UL")
        )
        minimum_pwm = int(
            constant_expression(self.config, "MOTOR_MIN_ACTIVE_PWM")
            .removesuffix("U")
        )
        maximum_pwm = int(
            constant_expression(self.config, "MAX_PWM").removesuffix("U")
        )
        self.assertGreater(pulses_per_revolution, 0)
        self.assertGreater(minimum_pwm, 0)
        self.assertLessEqual(minimum_pwm, maximum_pwm)
        self.assertNotIn("MAGNET_BENCH_MODE", self.config)
        self.assertNotIn("updateMagnetBench", self.drive)
        self.assertIn("COMMAND_WATCHDOG_MAX_MS", self.config)

    def test_ros_blocks_legacy_magnet_bench_mode(self):
        self.assertIn(
            "('command.allow_magnet_bench_mode', False)", self.bridge
        )
        self.assertIn("CAP_MAGNET_BENCH_MODE", self.bridge)
        self.assertIn("STATUS_MAGNET_BENCH_MODE", self.bridge)
        for path in ROS_CONFIGS:
            text = path.read_text(encoding="utf-8")
            self.assertIn("allow_magnet_bench_mode: false", text)

    def test_terrain_uses_i2c_without_gpio_actuator_outputs(self):
        self.assertIn("Wire.begin()", self.terrain)
        self.assertNotIn("analogWrite(", self.terrain)
        self.assertNotRegex(self.terrain, r"\battachInterrupt\s*\(")

    def test_terrain_gps_uses_altsoftserial_pins(self):
        self.assertEqual(
            constant_expression(self.terrain_config, "GPS_RX_PIN"), "8U"
        )
        self.assertEqual(
            constant_expression(self.terrain_config, "GPS_TX_PIN"), "9U"
        )
        self.assertEqual(
            constant_expression(self.terrain_config, "GPS_BAUD"), "9600UL"
        )
        self.assertIn('constexpr bool ENABLE_GPS = true;', self.terrain_config)
        self.assertIn('g_gps.poll()', self.terrain)
        self.assertIn('TYPE_GPS_TELEMETRY', self.terrain)


if __name__ == "__main__":
    unittest.main()
