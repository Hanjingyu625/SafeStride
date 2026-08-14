"""Static checks for production MCU pin ownership and disabled placeholders."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
DRIVE_CONFIG = ROOT / "firmware/safestride_mcu/config.h"
DRIVE_FIRMWARE = ROOT / "firmware/safestride_mcu/safestride_mcu.ino"
TERRAIN_FIRMWARE = ROOT / "firmware/terrain_mcu/terrain_mcu.ino"


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
        self.assertNotIn("0U", owners.values(), "D0 is reserved for USB serial")
        self.assertNotIn("1U", owners.values(), "D1 is reserved for USB serial")

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

    def test_terrain_uses_i2c_without_gpio_actuator_outputs(self):
        self.assertIn("Wire.begin()", self.terrain)
        self.assertNotIn("analogWrite(", self.terrain)
        self.assertNotRegex(self.terrain, r"\battachInterrupt\s*\(")


if __name__ == "__main__":
    unittest.main()
