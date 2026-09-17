"""Static checks for the importable SafeStride Foxglove dashboard."""

import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "config/foxglove/safestride.json"
ROS_CONFIGS = (
    ROOT / "config/raspberry_pi.yaml",
    ROOT / "src/safestride_bringup/config/safestride.yaml",
)

REQUIRED_PANELS = {
    "StateTransitions!safestride-state",
    "RawMessages!walker-status",
    "RawMessages!pressure",
    "RawMessages!terrain",
    "DiagnosticsSummary!diagnostics",
    "Plot!speed",
    "Plot!pressure",
    "Plot!tof",
    "Image!camera",
    "Indicator!surface-class",
    "Indicator!surface-valid",
    "Gauge!surface-confidence",
    "Plot!tilt",
}

REQUIRED_TOPICS = {
    "/walker/status",
    "/wheel/hall",
    "/handle/pressure",
    "/terrain/status",
    "/gps/fix",
    "/gps/speed_kmh",
    "/crosswalk/status",
    "/perception/surface_condition",
    "/camera/image/compressed",
    "/diagnostics",
}

REQUIRED_EXPRESSIONS = {
    "/walker/status.state",
    "/walker/status.deadman",
    "/wheel/hall.left_speed_kmh",
    "/walker/status.measured_speed_kmh",
    "/handle/pressure.left_filtered",
    "/handle/pressure.right_filtered",
    "/terrain/status.tof_filtered_m",
    "/terrain/status.tof_reference_m",
    "/terrain/status.tof_error_m",
    "/terrain/status.tof_change_m",
    "/terrain/status.pitch_rad",
    "/terrain/status.roll_rad",
    "/gps/speed_kmh.data",
    "/crosswalk/status.state",
    "/perception/surface_condition.classification",
    "/perception/surface_condition.confidence",
    "/perception/surface_condition.valid",
    "/camera/image/compressed",
}


def panel_ids(node):
    if isinstance(node, str):
        yield node
        return
    yield from panel_ids(node["first"])
    yield from panel_ids(node["second"])


def configured_expressions(layout):
    expressions = set()
    for config in layout["configById"].values():
        for series in config.get("paths", []):
            value = series.get("value")
            if isinstance(value, str) and value.startswith("/"):
                expressions.add(value)
        for key in ("topicPath", "followTopic", "topicToRender", "path"):
            value = config.get(key)
            if isinstance(value, str) and value.startswith("/"):
                expressions.add(value)
        image_topic = config.get("imageMode", {}).get("imageTopic")
        if image_topic:
            expressions.add(image_topic)
    return expressions


class TestFoxgloveLayout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    def test_every_configured_panel_is_placed_once(self):
        placed = list(panel_ids(self.layout["layout"]))
        self.assertEqual(set(placed), REQUIRED_PANELS)
        self.assertEqual(len(placed), len(set(placed)))
        self.assertEqual(set(self.layout["configById"]), REQUIRED_PANELS)

    def test_required_live_fields_are_visible(self):
        expressions = configured_expressions(self.layout)
        self.assertTrue(REQUIRED_EXPRESSIONS.issubset(expressions))

        right = self.layout["layout"]["second"]["second"]
        self.assertEqual(right["first"], "Image!camera")
        self.assertEqual(right["second"]["second"], "Plot!tilt")
        self.assertEqual(set(panel_ids(right["second"]["first"])), {
            "Indicator!surface-class", "Indicator!surface-valid",
            "Gauge!surface-confidence",
        })
        confidence = self.layout["configById"]["Gauge!surface-confidence"]
        self.assertEqual((confidence["minValue"], confidence["maxValue"]), (0, 1))
        self.assertEqual(
            self.layout["configById"]["DiagnosticsSummary!diagnostics"][
                "topicToRender"
            ],
            "/diagnostics",
        )

    def test_hall_and_pressure_calibration_match_runtime(self):
        run_script = (ROOT / "scripts/run.sh").read_text(encoding="utf-8")
        drive_config = (
            ROOT / "firmware/safestride_mcu/config.h"
        ).read_text(encoding="utf-8")
        pressure_paths = self.layout["configById"]["Plot!pressure"]["paths"]

        self.assertIn('SAFESTRIDE_WHEEL_RADIUS_M:-0.115', run_script)
        self.assertIn(
            "/wheel/hall.left_speed_kmh",
            configured_expressions(self.layout),
        )
        self.assertIn("PRESSURE_LEFT_PRESENT_THRESHOLD = 40.0F", drive_config)
        self.assertIn("PRESSURE_RIGHT_PRESENT_THRESHOLD = 40.0F", drive_config)
        self.assertIn("40", {series["value"] for series in pressure_paths})

    def test_bridge_exposes_dashboard_topics_read_only(self):
        for path in ROS_CONFIGS:
            text = path.read_text(encoding="utf-8")
            patterns = re.findall(r"^\s*-\s+(\^.+\$)\s*$", text, re.MULTILINE)
            self.assertTrue(patterns, f"No topic whitelist in {path}")
            for topic in REQUIRED_TOPICS:
                self.assertTrue(
                    any(re.fullmatch(pattern, topic) for pattern in patterns),
                    f"{topic} is not allowed by {path}",
                )
            self.assertIn("service_whitelist: ['(?!)']", text)
            self.assertIn("param_whitelist: ['(?!)']", text)
            self.assertIn("client_topic_whitelist: ['(?!)']", text)


if __name__ == "__main__":
    unittest.main()
