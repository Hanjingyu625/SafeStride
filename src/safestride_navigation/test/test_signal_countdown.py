from safestride_navigation.signal_logic import SignalCountdown, SignalApiClient


def records(state='protected-Movement-Allowed', seconds=20, stamp=1000):
    return ({'itstId': '42', 'trsmUtcTime': stamp * 1000,
             'ntPdsgRmdrCs': seconds * 10},
            {'itstId': '42', 'trsmUtcTime': stamp * 1000, 'ntPdsgStatNm': state})


def test_local_countdown_survives_twelve_seconds_but_never_predicts_next_phase():
    timer = SignalCountdown()
    timing, phase = records()
    timer.observe(timing, phase, 1002)
    assert timer.remaining('42', ['nt'], 1002) == 18
    assert timer.state('42', ['nt'], 1019) == (1, True, 'green pedestrian signal')
    assert not timer.state('42', ['nt'], 1020)[1]
    timer.observe(timing, phase, 1021)
    assert not timer.state('42', ['nt'], 1021)[1]


def test_red_time_schedules_poll_but_never_becomes_green_budget():
    timer = SignalCountdown()
    timer.observe(*records('stop-And-Remain', 100), 1002)
    assert timer.remaining('42', ['nt'], 1050) == 50
    assert timer.state('42', ['nt'], 1050) == (0, True, 'red pedestrian signal')
    assert not timer.state('42', ['nt'], 1100)[1]
    timer.observe(*records(stamp=1100), 1101)
    assert timer.state('42', ['nt'], 1101) == (19, True, 'green pedestrian signal')


def test_old_future_mismatched_and_expired_observations_do_not_start_countdown():
    for observed in (1013, 998):
        timer = SignalCountdown()
        timer.observe(*records(), observed)
        assert timer.remaining('42', ['nt'], observed) is None
    timer = SignalCountdown()
    timing, phase = records()
    timing['itstId'] = '43'
    timer.observe(timing, phase, 1001)
    assert not timer.state('42', ['nt'], 1001)[1]
    timer.observe(*records(seconds=1, stamp=1002), 1004)
    assert not timer.state('42', ['nt'], 1004)[1]


def test_missing_red_timing_retries_and_only_keeps_fresh_red():
    timer = SignalCountdown()
    _, phase = records('stop-And-Remain')
    timer.observe(None, phase, 1001)
    assert timer.state('42', ['nt'], 1005) == (0, True, 'red pedestrian signal')
    assert timer.remaining('42', ['nt'], 1005) is None
    assert not timer.state('42', ['nt'], 1013)[1]


def test_selection_changes_and_clock_reversal_do_not_reuse_unrelated_state():
    timer = SignalCountdown()
    timer.observe(*records(), 1001)
    for iid, directions, now in [('43', ['nt'], 1002), ('42', ['st'], 1002),
                                  ('42', ['nt', 'st'], 1002), ('42', ['nt'], 1000)]:
        assert not timer.state(iid, directions, now)[1]


def test_countdown_client_does_not_impose_quota_average_interval(tmp_path):
    import json
    import time
    saved = tmp_path / 'backoff.json'
    saved.write_text(json.dumps({
        'timing': {'until': time.time() + 500, 'reason': 'signal API quota pacing'},
        'phase': {'until': time.time() + 500, 'reason': 'signal API HTTP 429'},
    }))
    client = SignalApiClient(clock=lambda: 1000, minimum_interval_s=3,
        cache_file=str(saved), pace_quota=False,
        fetch=lambda *a, **kw: {'_api_rate_limit': {'reset_s': 24000, 'remaining': 1000}})
    client.get('test', '42', url='timing', timeout_s=3)
    assert client.retry_remaining(['timing']) == 3
    assert client.retry_remaining(['phase']) > 490


def test_countdown_keeps_heads_independent_when_colours_differ():
    timer = SignalCountdown()
    timing, phase = records()
    timing['stPdsgRmdrCs'] = 600
    phase['stPdsgStatNm'] = 'stop-And-Remain'
    timer.observe(timing, phase, 1001)
    assert timer.state('42', ['nt'], 1005) == (15, True, 'green pedestrian signal')
    assert timer.state('42', ['st'], 1005) == (0, True, 'red pedestrian signal')
    assert timer.remaining('42', ['st'], 1005) == 55


def test_zero_red_preserves_colour_without_inventing_a_countdown():
    timer = SignalCountdown()
    timing, phase = records('stop-And-Remain', 0)
    timer.observe(timing, phase, 1001)
    assert timer.state('42', ['nt'], 1005) == (0, True, 'red pedestrian signal')
    assert timer.remaining('42', ['nt'], 1005) is None
    assert timer.zero_retry_count('42', ['nt']) == 0
    timer.observe(timing, phase, 1011)
    assert not timer.state('42', ['nt'], 1012)[1]


def test_zero_green_never_grants_entry_time():
    timer = SignalCountdown()
    timer.observe(*records(seconds=0), 1001)
    assert not timer.state('42', ['nt'], 1001)[1]
    assert timer.zero_retry_count('42', ['nt']) == 0


def test_expired_positive_red_still_preserves_fresh_phase():
    timer = SignalCountdown()
    timer.observe(*records('stop-And-Remain', 1), 1002)
    assert timer.state('42', ['nt'], 1002) == (0, True, 'red pedestrian signal')
    assert timer.zero_retry_count('42', ['nt']) is None
