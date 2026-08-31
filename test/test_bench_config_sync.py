"""Keep standalone Arduino benches aligned with production wiring."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def constant(text: str, name: str) -> str:
    match = re.search(
        rf"constexpr\s+[^;=]+\s+{re.escape(name)}\s*=\s*([^;]+);",
        text,
    )
    if match is None:
        raise AssertionError(f"missing constant: {name}")
    return re.sub(r"\s+", "", match.group(1))


class TestBenchConfigSync(unittest.TestCase):

    def test_drive_bench_matches_single_hall_and_pressure(self):
        production = (ROOT / "firmware/safestride_mcu/config.h").read_text()
        bench = (
            ROOT / "firmware/safestride_sensor_bench_test/"
            "safestride_sensor_bench_test.ino"
        ).read_text()
        self.assertEqual(constant(production, "LEFT_HALL_PIN"), "2U")
        self.assertEqual(constant(production, "HALL_ACTIVE_LEVEL"), "LOW")
        self.assertEqual(
            constant(production, "HALL_PULSES_PER_WHEEL_REV"), "6UL"
        )
        self.assertIn("constexpr uint8_t HALL_PIN = 2U;", bench)
        self.assertIn("constexpr uint8_t PRESSURE_LEFT_PIN = A2;", bench)
        self.assertIn("constexpr uint8_t PRESSURE_RIGHT_PIN = A1;", bench)
        self.assertIn("constexpr uint32_t HALL_PULSES_PER_REV = 6UL;", bench)
        self.assertIn("attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallIsr, FALLING)", bench)
        self.assertIn("constexpr float PRESSURE_THRESHOLD = 80.0F;", bench)
        self.assertIn("digitalWrite(MOTOR_PWM_PIN, LOW);", bench)

    def test_terrain_bench_has_only_installed_i2c_sensors(self):
        bench = (
            ROOT / "firmware/terrain_sensor_bench_test/"
            "terrain_sensor_bench_test.ino"
        ).read_text()
        self.assertIn("TOF_ADDRESS = 0x52U", bench)
        self.assertIn("MPU_ADDRESS_LOW = 0x68U", bench)
        self.assertIn("MPU_ADDRESS_HIGH = 0x69U", bench)
        self.assertNotIn("analogWrite(", bench)

    def test_hall_pulses_match_ros_configs(self):
        for relative in (
            "config/raspberry_pi.yaml",
            "src/safestride_bringup/config/safestride.yaml",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertRegex(text, r"hall_pulses_per_revolution:\s*6\b")


if __name__ == "__main__":
    unittest.main()
