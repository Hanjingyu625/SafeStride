import math
import unittest

from safestride_sensors.nmea import parse_fix


def sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f'${body}*{checksum:02X}'


class TestNmeaParser(unittest.TestCase):
    def test_valid_rmc(self):
        fix = parse_fix(
            '$GPRMC,123519,A,4807.038,N,01131.000,E,'
            '022.4,084.4,230394,003.1,W*6A'
        )
        self.assertIsNotNone(fix)
        self.assertTrue(fix.valid)
        self.assertEqual(round(fix.latitude, 6), 48.1173)
        self.assertEqual(round(fix.longitude, 6), 11.516667)
        self.assertEqual(round(fix.speed_mps, 3), 11.524)
        self.assertEqual(fix.course_deg, 84.4)
        self.assertEqual(fix.sentence_type, 'RMC')

    def test_valid_rmc_allows_missing_speed_and_course(self):
        fix = parse_fix(
            sentence(
                'GNRMC,123519,A,3723.2475,N,12158.3416,E,'
                ',,230394,,,A'
            )
        )
        self.assertIsNotNone(fix)
        self.assertTrue(fix.valid)
        self.assertIsNone(fix.speed_mps)
        self.assertIsNone(fix.course_deg)

    def test_valid_gga_exposes_receiver_quality(self):
        fix = parse_fix(
            sentence(
                'GNGGA,040728.00,3732.44128,N,12704.75004,E,'
                '1,06,2.17,31.0,M,18.7,M,,'
            )
        )
        self.assertIsNotNone(fix)
        self.assertTrue(fix.valid)
        self.assertEqual(fix.sentence_type, 'GGA')
        self.assertEqual(fix.fix_quality, 1)
        self.assertEqual(fix.satellites, 6)
        self.assertEqual(fix.hdop, 2.17)
        self.assertEqual(fix.altitude_m, 31.0)

    def test_invalid_gga_keeps_quality_metadata(self):
        fix = parse_fix(
            sentence('GNGGA,040728.00,,,,,0,02,9.73,,,,,,')
        )
        self.assertIsNotNone(fix)
        self.assertFalse(fix.valid)
        self.assertEqual(fix.fix_quality, 0)
        self.assertEqual(fix.satellites, 2)
        self.assertEqual(fix.hdop, 9.73)

    def test_bad_checksum_is_rejected(self):
        self.assertIsNone(parse_fix('$GPRMC,1,V,,,,,,,,,*01'))

    def test_valid_no_fix_sentence_is_published_as_invalid(self):
        fix = parse_fix(sentence('GPRMC,123519,V,,,,,,,230394,,,N'))
        self.assertIsNotNone(fix)
        self.assertFalse(fix.valid)
        self.assertTrue(math.isnan(fix.latitude))
        self.assertTrue(math.isnan(fix.longitude))

    def test_out_of_range_coordinate_is_rejected(self):
        self.assertIsNone(
            parse_fix(
                sentence(
                    'GPRMC,123519,A,9160.000,N,01131.000,E,0.0'
                )
            )
        )


if __name__ == '__main__':
    unittest.main()
