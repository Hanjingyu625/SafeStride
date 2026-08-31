import json
import math
import tempfile
import unittest
from pathlib import Path

from safestride_navigation.crosswalk_data import (
    CrosswalkSpatialIndex,
    crosswalk_axis_position,
    load_crosswalks,
    nearest_crosswalk,
)


class TestCrosswalkData(unittest.TestCase):
    def test_loads_v2_converter_fields_and_rejects_bad_records(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'crosswalks.json'
            source.write_text(
                json.dumps(
                    [
                        {
                            'latitude': 37.5,
                            'longitude': 127.0,
                            'et': 12.0,
                            'bt': 4.0,
                            'axis_bearing_deg': 5.0,
                            'itstId': 'A-1',
                        },
                        {'latitude': 200.0, 'longitude': 0.0, 'et': 3.0},
                    ]
                ),
                encoding='utf-8',
            )
            records = load_crosswalks(str(source))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['length_m'], 12.0)
        self.assertEqual(records[0]['width_m'], 4.0)
        self.assertEqual(records[0]['intersection_id'], 'A-1')

    def test_nearest_crosswalk_orients_axis_toward_destination(self):
        crosswalk = {
            'index': 0,
            'latitude': 0.0,
            'longitude': 0.0,
            'length_m': 10.0,
            'width_m': 3.0,
            'axis_bearing_deg': 0.0,
            'intersection_id': '',
        }
        five_metres_south = -5.0 / 111_320.0
        selected = nearest_crosswalk([crosswalk], five_metres_south, 0.0)
        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertEqual(selected['crossing_bearing_deg'], 0.0)
        self.assertLess(selected['edge_distance_m'], 0.05)
        position = crosswalk_axis_position(
            selected,
            five_metres_south,
            0.0,
            selected['crossing_bearing_deg'],
        )
        self.assertTrue(
            math.isclose(position['progress_m'], 0.0, abs_tol=0.05)
        )

    def test_heading_selects_crosswalk_ahead(self):
        north = {
            'index': 1,
            'latitude': 20.0 / 111_320.0,
            'longitude': 0.0,
            'length_m': 5.0,
            'width_m': 3.0,
            'axis_bearing_deg': 0.0,
            'intersection_id': '',
        }
        south = {
            'index': 2,
            'latitude': -10.0 / 111_320.0,
            'longitude': 0.0,
            'length_m': 5.0,
            'width_m': 3.0,
            'axis_bearing_deg': 0.0,
            'intersection_id': '',
        }
        nearest = nearest_crosswalk([north, south], 0.0, 0.0)
        ahead = nearest_crosswalk(
            [north, south],
            0.0,
            0.0,
            heading_deg=0.0,
            maximum_heading_error_deg=75.0,
        )
        self.assertEqual(nearest['index'], 2)
        self.assertEqual(ahead['index'], 1)
        self.assertEqual(ahead['crossing_bearing_deg'], 0.0)
        self.assertLess(ahead['heading_error_deg'], 0.1)

    def test_spatial_index_keeps_long_crosswalk_whose_edge_is_nearby(self):
        crosswalk = {
            'index': 7,
            'latitude': 90.0 / 111_320.0,
            'longitude': 0.0,
            'length_m': 100.0,
            'width_m': 4.0,
            'axis_bearing_deg': 0.0,
            'intersection_id': '',
        }
        index = CrosswalkSpatialIndex([crosswalk])
        selected = index.nearest(
            0.0,
            0.0,
            maximum_distance_m=50.0,
        )
        self.assertIsNotNone(selected)
        self.assertEqual(selected['index'], 7)
        self.assertLess(selected['edge_distance_m'], 41.0)


if __name__ == '__main__':
    unittest.main()
