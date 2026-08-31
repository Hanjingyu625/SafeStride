import math

from safestride_sensors.nmea import parse_fix


def sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f'${body}*{checksum:02X}'


def test_valid_rmc():
    fix = parse_fix(
        '$GPRMC,123519,A,4807.038,N,01131.000,E,'
        '022.4,084.4,230394,003.1,W*6A'
    )
    assert fix and fix.valid
    assert round(fix.latitude, 6) == 48.1173
    assert round(fix.longitude, 6) == 11.516667
    assert round(fix.speed_mps, 3) == 11.524
    assert fix.course_deg == 84.4


def test_valid_rmc_allows_missing_speed_and_course():
    fix = parse_fix(
        sentence('GNRMC,123519,A,3723.2475,N,12158.3416,E,,,230394,,,A')
    )
    assert fix and fix.valid
    assert fix.speed_mps is None
    assert fix.course_deg is None


def test_bad_checksum_is_rejected():
    assert parse_fix('$GPRMC,1,V,,,,,,,,,*01') is None


def test_valid_no_fix_sentence_is_published_as_invalid():
    fix = parse_fix(sentence('GPRMC,123519,V,,,,,,,230394,,,N'))
    assert fix and not fix.valid
    assert math.isnan(fix.latitude)
    assert math.isnan(fix.longitude)


def test_out_of_range_coordinate_is_rejected():
    assert parse_fix(
        sentence('GPRMC,123519,A,9160.000,N,01131.000,E,0.0')
    ) is None
