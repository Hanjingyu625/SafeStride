"""Keep standalone Arduino sensor benches aligned with production config."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONSTANT_PATTERN = re.compile(
    r"constexpr\s+[A-Za-z0-9_:<>]+\s+([A-Za-z0-9_]+)\s*=\s*([^;]+);"
)


def constants(path: Path):
    text = path.read_text(encoding="utf-8")
    return {
        name: re.sub(r"\s+", "", expression)
        for name, expression in CONSTANT_PATTERN.findall(text)
    }


class TestBenchConfigSync(unittest.TestCase):

    def assert_constants_match(self, production, bench, names):
        production_values = constants(ROOT / production)
        bench_values = constants(ROOT / bench)
        for name in names:
            with self.subTest(name=name):
                self.assertIn(name, production_values)
                self.assertIn(name, bench_values)
                self.assertEqual(production_values[name], bench_values[name])

    def test_drive_sensor_constants_match_production(self):
        self.assert_constants_match(
            "firmware/safestride_mcu/config.h",
            "firmware/safestride_sensor_bench_test/"
            "safestride_sensor_bench_test.ino",
            (
                "SERIAL_BAUD",
                "LEFT_HALL_PIN",
                "RIGHT_HALL_PIN",
                "HALL_ACTIVE_LEVEL",
                "HALL_MIN_PULSE_INTERVAL_US",
                "HALL_ZERO_TIMEOUT_US",
                "HALL_PULSES_PER_WHEEL_REV",
                "HALL_CALIBRATED",
                "MOTOR_PWM_PIN",
                "MOTOR_IN1_PIN",
                "MOTOR_IN2_PIN",
                "PRESSURE_LEFT_PIN",
                "PRESSURE_RIGHT_PIN",
                "PRESSURE_SAMPLE_PERIOD_MS",
                "PRESSURE_FILTER_ALPHA",
                "PRESSURE_LEFT_ACTIVE_HIGH",
                "PRESSURE_RIGHT_ACTIVE_HIGH",
                "PRESSURE_LEFT_PRESENT_THRESHOLD",
                "PRESSURE_RIGHT_PRESENT_THRESHOLD",
                "PRESSURE_THRESHOLDS_CALIBRATED",
                "PRESSURE_IMBALANCE_THRESHOLD",
                "PRESSURE_SUDDEN_CHANGE_THRESHOLD",
                "USE_DRIVER_FAULT_PIN",
                "DRIVER_FAULT_PIN",
                "DRIVER_FAULT_ACTIVE_LEVEL",
            ),
        )

    def test_terrain_tof_constants_match_production(self):
        self.assert_constants_match(
            "firmware/terrain_mcu/config.h",
            "firmware/terrain_sensor_bench_test/"
            "terrain_sensor_bench_test.ino",
            (
                "SERIAL_BAUD",
                "TOF_I2C_ADDRESS",
                "TOF_DISTANCE_REGISTER",
                "TOF_SAMPLE_PERIOD_MS",
                "TOF_MIN_VALID_DISTANCE_MM",
                "TOF_MAX_VALID_DISTANCE_MM",
                "TOF_FILTER_ALPHA",
                "TOF_REFERENCE_ALPHA",
                "TOF_ERROR_THRESHOLD_MM",
                "TOF_CHANGE_THRESHOLD_MM",
                "TOF_REQUIRED_FRAMES",
                "TOF_RED_HOLD_MS",
            ),
        )

    def test_drive_bench_only_writes_zero_pwm(self):
        path = ROOT / "firmware/safestride_sensor_bench_test/" \
            "safestride_sensor_bench_test.ino"
        text = path.read_text(encoding="utf-8")
        writes = re.findall(r"analogWrite\(([^;]+)\);", text)
        self.assertTrue(writes)
        for write in writes:
            self.assertRegex(write, r",\s*0\s*$")
        self.assertIn("holdMotorOutputSafe();", text)
        self.assertNotIn("LED_", text)
        self.assertNotIn("RUN ", text)

    def test_terrain_bench_has_no_actuator_output(self):
        path = ROOT / "firmware/terrain_sensor_bench_test/" \
            "terrain_sensor_bench_test.ino"
        text = path.read_text(encoding="utf-8")
        self.assertNotIn("analogWrite(", text)
        self.assertNotIn("RUN ", text)
        self.assertNotIn("LED_", text)

    def test_hall_pulses_match_ros_configs(self):
        production = constants(
            ROOT / "firmware/safestride_mcu/config.h"
        )["HALL_PULSES_PER_WHEEL_REV"]
        expected = int(re.sub(r"[^0-9]", "", production))
        for relative in (
            "config/raspberry_pi.yaml",
            "src/safestride_bringup/config/safestride.yaml",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            match = re.search(
                r"hall_pulses_per_revolution:\s*([0-9]+)", text
            )
            with self.subTest(path=relative):
                self.assertIsNotNone(match)
                self.assertEqual(int(match.group(1)), expected)

    def test_legacy_feedback_and_sensor_led_logic_are_absent(self):
        paths = (
            ROOT / "firmware",
            ROOT / "src/safestride_bridge",
            ROOT / "src/safestride_navigation",
        )
        for root in paths:
            for path in root.rglob("*"):
                if not path.is_file() or path.suffix not in {
                    ".h", ".cpp", ".ino", ".py", ".yaml", ".md"
                }:
                    continue
                text = path.read_text(encoding="utf-8")
                with self.subTest(path=str(path.relative_to(ROOT))):
                    self.assertNotIn("encoder", text.lower())
                    self.assertNotIn("LED_BUILTIN", text)
                    self.assertNotRegex(text, r"\bLED_[A-Z0-9_]+")

    def test_production_uses_one_motor_driver_output(self):
        config = (
            ROOT / "firmware/safestride_mcu/config.h"
        ).read_text(encoding="utf-8")
        controller = (
            ROOT / "firmware/safestride_mcu/motor_control.cpp"
        ).read_text(encoding="utf-8")
        for name in ("MOTOR_PWM_PIN", "MOTOR_IN1_PIN", "MOTOR_IN2_PIN"):
            self.assertIn(name, config)
        self.assertNotIn("LEFT_MOTOR_", config)
        self.assertNotIn("RIGHT_MOTOR_", config)
        writes = re.findall(r"analogWrite\(([^,]+),", controller)
        self.assertTrue(writes)
        self.assertEqual(set(item.strip() for item in writes), {
            "cfg::MOTOR_PWM_PIN"
        })


if __name__ == "__main__":
    unittest.main()
