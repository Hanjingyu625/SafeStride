import math

import pytest

from safestride_navigation.heading_fusion import HeadingFusion


def test_gyro_cannot_invent_north_before_gps_anchor():
    fusion = HeadingFusion()
    for i in range(20):
        fusion.gyro(-math.pi / 2, i / 10)
    assert fusion.heading(1.9) is None
    assert fusion.confidence == 0


def test_north_then_clockwise_pivot_gives_east_without_translation():
    fusion = HeadingFusion()
    fusion.gyro(0, 0)
    fusion.gps(0, 0, 0)
    for i in range(1, 11):
        fusion.gyro(-math.pi / 2, i / 10, stationary=True)
        # Stale GPS trajectory during the pivot must not undo the turn.
        fusion.gps(0, i / 10, i / 10)
    assert fusion.heading(1) == pytest.approx(90)
    assert fusion.source == 'gps_gyro'
    assert fusion.bias == 0


def test_wraparound_and_gps_correction_take_short_angle():
    fusion = HeadingFusion()
    fusion.gps(359, 0, 0)
    fusion.gps(1, 1, 1)
    assert fusion.heading(1) == pytest.approx(359.4)


def test_dropout_revokes_heading_and_old_gps_cannot_reanchor():
    fusion = HeadingFusion()
    fusion.gyro(0, 0)
    fusion.gps(90, 0, 0)
    assert fusion.heading(0.4) is None
    fusion.gyro(0, 0.5)
    fusion.gps(90, 0, 0.5)
    assert fusion.heading(0.5) is None
    fusion.gps(100, 0.6, 0.6)
    assert fusion.heading(0.6) == 100


def test_gyro_gap_is_detected_even_without_heading_poll():
    fusion = HeadingFusion()
    fusion.gyro(0, 0)
    fusion.gps(0, 0, 0)
    fusion.gyro(-1, 1)
    assert fusion.heading(1) is None


def test_confidence_decays_and_continuous_imu_cannot_extend_absolute_anchor():
    fusion = HeadingFusion()
    fusion.gyro(0, 0)
    fusion.gps(0, 0, 0)
    for i in range(1, 101):
        fusion.gyro(0, i / 10)
    assert fusion.heading(10) == 0
    assert fusion.confidence == pytest.approx(0.5)
    for i in range(101, 202):
        fusion.gyro(0, i / 10)
    assert fusion.heading(20.1) is None
    assert fusion.confidence == 0


def test_gps_only_mode_expires_and_invalid_gyro_never_integrates():
    fusion = HeadingFusion()
    fusion.gps(45, 0, 0)
    assert fusion.heading(1) == 45
    assert fusion.source == 'gps'
    assert fusion.heading(5.1) is None
    fusion.gyro(float('nan'), 6)
    assert fusion.heading(6) is None


def test_bias_learning_requires_quiet_standstill():
    fusion = HeadingFusion()
    bias = math.radians(0.2)
    for i in range(101):
        fusion.gyro(bias, i / 10, stationary=False)
    assert fusion.bias == 0
    for i in range(101, 301):
        fusion.gyro(bias, i / 10, stationary=True)
    assert 0 < fusion.bias < bias
