from safestride_sensors.nmea import parse_fix


def test_valid_rmc():
    fix = parse_fix('$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A')
    assert fix and fix.valid
    assert round(fix.latitude, 6) == 48.1173
    assert round(fix.longitude, 6) == 11.516667


def test_bad_checksum_is_rejected():
    assert parse_fix('$GPRMC,1,V,,,,,,,,,*00') is None
