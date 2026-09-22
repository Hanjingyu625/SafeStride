"""Validate the checked-in VisualTFT source against the display contracts."""

import csv
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DISPLAY = ROOT / "display" / "ezhmi"
PROJECT = DISPLAY / "visualtft"


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


class VisualTftProjectTests(unittest.TestCase):
    def test_project_and_pages(self):
        project = ET.parse(PROJECT / "Project.tftprj").getroot()
        self.assertEqual(project.tag, "VisualTFT")
        self.assertEqual(project.attrib["Name"], "SafeStride")
        self.assertEqual(project.attrib["StartupPage"], "Screen0")
        self.assertEqual(project.attrib["DeviceType"], "19005")
        # VisualTFT baud index 4 is 19200, matching terrain_mcu/ezhmi_transport.h.
        self.assertEqual(project.attrib["DeviceBaudRate"], "4")
        self.assertEqual(
            [page.attrib["RelativePath"] for page in project.findall("./Pages/Page")],
            ["Screen0.tft", "Screen1.tft"],
        )

        for number in (0, 1):
            page = ET.parse(PROJECT / f"Screen{number}.tft").getroot()
            self.assertEqual(page.attrib["name"], f"Screen{number}")
            self.assertEqual((page.attrib["width"], page.attrib["height"]), ("480", "272"))

    def test_functional_controls_match_csv(self):
        fonts = {
            item.attrib["id"]: int(item.attrib["height"])
            for item in ET.parse(PROJECT / "font" / "fonts.xml").getroot()
        }
        pages = {
            number: {
                int(item.attrib["id"]): item
                for item in ET.parse(PROJECT / f"Screen{number}.tft").getroot()
            }
            for number in (0, 1)
        }

        for expected in read_csv(DISPLAY / "controls.csv"):
            page = pages[int(expected["screen"])]
            control = page[int(expected["id"])]
            self.assertEqual(control.attrib["type"], expected["type"])
            self.assertEqual(
                tuple(int(control.attrib[key]) for key in ("xOffset", "yOffset", "width", "height")),
                tuple(int(expected[key]) for key in ("x", "y", "width", "height")),
            )
            self.assertEqual(fonts[control.attrib["font"]], int(expected["font_px"]))
            initial = (
                control.attrib["text_state_up"]
                if control.attrib["type"] == "button"
                else control.attrib["text"]
            )
            self.assertEqual(initial, expected["initial_text"])

    def test_control_ids_and_safe_dev_button(self):
        page = ET.parse(PROJECT / "Screen1.tft").getroot()
        controls = {int(item.attrib["id"]): item for item in page}
        self.assertEqual(set(controls), set(range(1, 24)))
        self.assertEqual(len(controls), len(list(page)))
        self.assertEqual(controls[10].attrib["text"], "GPS: UNAVAILABLE")

        button = controls[11]
        self.assertEqual(button.attrib["type"], "button")
        self.assertEqual(button.attrib["notify_disable"], "1")
        for key in ("action", "custom_data_up", "custom_data_down", "external_data_up", "external_data_down"):
            self.assertEqual(button.attrib[key], "")

        for control_id in range(17, 23):
            panel = controls[control_id]
            self.assertEqual(panel.attrib["type"], "text")
            self.assertEqual(panel.attrib["show_bk"], "1")

    def test_modbus_variables_match_csv(self):
        expected = read_csv(DISPLAY / "registers.csv")
        variants = ET.parse(PROJECT / "script.xml").getroot().findall("./variants/variant")
        self.assertEqual(len(variants), len(expected))
        for actual, contract in zip(variants, expected):
            self.assertEqual(actual.attrib["name"], contract["name"])
            self.assertEqual(int(actual.attrib["addr"]), int(contract["address_hex"], 16))
            self.assertEqual(int(actual.attrib["value"]), int(contract["default"]))

    def test_visualtft_lua_matches_canonical_source(self):
        canonical = (DISPLAY / "safestride.lua").read_text(encoding="ascii")
        visualtft = (PROJECT / "main.lua").read_text(encoding="ascii")
        self.assertEqual(visualtft.replace("\r\n", "\n"), canonical.replace("\r\n", "\n"))
        self.assertNotIn("BUILD_TAG", canonical)
        self.assertNotIn("BUILD 3", canonical)
        self.assertIn(
            'local surfaces = {"SMOOTH", "ROUGH", "WET", "GRAVEL", "STEP", "HOLE"}',
            canonical,
        )

        for item in ET.parse(PROJECT / "Screen1.tft").getroot():
            for key in ("text", "text_state_up", "text_state_down"):
                item.attrib.get(key, "").encode("ascii")


if __name__ == "__main__":
    unittest.main()
