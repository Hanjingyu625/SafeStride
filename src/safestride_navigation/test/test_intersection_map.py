import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from safestride_navigation.intersection_map import (
    load_intersection_map,
    nearest_intersection,
    normalize_intersections,
    save_intersection_map,
    select_intersection_id,
)


class TestIntersectionMap(unittest.TestCase):
    def test_normalizes_nested_records_and_rejects_bad_coordinates(self):
        records = normalize_intersections(
            {
                'body': {
                    'items': [
                        {
                            'itstId': '1678',
                            'itstNm': 'Konkuk',
                            'mapCtptIntLat': '37.5399365',
                            'mapCtptIntLot': '127.070598',
                        },
                        {
                            'itstId': 'bad',
                            'mapCtptIntLat': '999',
                            'mapCtptIntLot': '127.0',
                        },
                    ]
                }
            }
        )
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['intersection_id'], '1678')
        self.assertEqual(records[0]['name'], 'Konkuk')

    def test_selects_nearest_intersection_within_limit(self):
        intersections = [
            {
                'intersection_id': 'near',
                'name': '',
                'latitude': 10.0 / 111_320.0,
                'longitude': 0.0,
            },
            {
                'intersection_id': 'far',
                'name': '',
                'latitude': 100.0 / 111_320.0,
                'longitude': 0.0,
            },
        ]
        selected = nearest_intersection(
            intersections,
            0.0,
            0.0,
            maximum_distance_m=80.0,
        )
        self.assertEqual(selected['intersection_id'], 'near')
        self.assertLess(selected['distance_m'], 11.0)

    def test_cache_round_trip(self):
        records = [
            {
                'intersection_id': '42',
                'name': 'Guui',
                'latitude': 37.53806,
                'longitude': 127.08583,
            }
        ]
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'map.json'
            save_intersection_map(path, records)
            self.assertEqual(load_intersection_map(path), records)

    def test_dynamic_match_precedes_configured_fallback(self):
        identifier, source, name = select_intersection_id(
            locked_id='',
            crosswalk={'intersection_id': ''},
            nearest={'intersection_id': 'guui', 'name': 'Guui'},
            configured_id='1678',
        )
        self.assertEqual(identifier, 'guui')
        self.assertEqual(source, 'v2x_nearest')
        self.assertEqual(name, 'Guui')

    def test_configured_id_is_only_a_fallback(self):
        identifier, source, name = select_intersection_id(
            locked_id='',
            crosswalk=None,
            nearest=None,
            configured_id='1678',
        )
        self.assertEqual(identifier, '1678')
        self.assertEqual(source, 'configured_fallback')
        self.assertEqual(name, '')


if __name__ == '__main__':
    unittest.main()
