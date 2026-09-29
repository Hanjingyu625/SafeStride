import pytest

from safestride_navigation.signal_logic import cached_signal_window


@pytest.mark.parametrize('color', ['stop-And-Remain', 'permissive-Movement-Allowed'])
def test_reported_phase_end_includes_network_delay_and_never_flips_color(color):
    phase = dict(itstId='42', trsmUtcTime=1000000, ntPdsgStatNm=color)
    timing = dict(itstId='42', trsmUtcTime=1000000, ntPdsgRmdrCs=200)
    expected = 0 if color == 'stop-And-Remain' else 5
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1015)[:2] == (expected, True)
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1020)[1] is False
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1100)[1] is False
    assert cached_signal_window(timing, phase, ['nt'], 1015, 1016) is None
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1001) is None


def test_unavailable_red_countdown_cannot_schedule_long_cache():
    phase = dict(itstId='42', trsmUtcTime=1000000, ntPdsgStatNm='stop-And-Remain')
    timing = dict(itstId='42', trsmUtcTime=1000000, ntPdsgRmdrCs=36001)
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1003) is None
    timing['ntPdsgRmdrCs'] = 200
    timing['itstId'] = 'other'
    assert cached_signal_window(timing, phase, ['nt'], 1002, 1003) is None
