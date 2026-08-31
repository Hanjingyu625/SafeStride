import unittest

from safestride_navigation.intersection_map import (
    nearest_intersection,
    normalize_intersections,
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


if __name__ == '__main__':
    unittest.main()
