"""Run production polling methods without ROS/DDS dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Tuple
from unittest.mock import Mock

from safestride_navigation.signal_logic import (
    SignalCountdown, newer_signal_record,
)


def node():
    path = Path(__file__).parents[1] / 'safestride_navigation/crosswalk_controller_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_signal_state', '_request_signal_if_due', '_consume_signal_future')]
    scope = dict(Optional=Optional, Tuple=Tuple, request_signal_bundle=Mock(),
                 SignalCountdown=SignalCountdown, newer_signal_record=newer_signal_record)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    n = SimpleNamespace(_api_key='test', _signal_future=None,
        _last_signal_request=1000, _signal_response_time=1000,
        _last_signal_request_key=('42', 'nt'),
        _signal_refresh=1, _executor=Mock(), _signal_url='timing',
        _phase_url='phase', _combined_url='combined', _signal_request_timeout=3,
        _signal_cache_id='42', _phase_cache=None, _signal_error='old error',
        _signal_countdown=SignalCountdown(), _signal_cache=None, _signal_cache_max_age=12,
        _signal_client=SimpleNamespace(poll_interval_remaining=lambda _: 0.0,
                                       wait_reason=lambda _: ''))
    n._request_signal_if_due = lambda *args: scope['_request_signal_if_due'](n, *args)
    n._signal_state = lambda *args: scope['_signal_state'](n, *args)
    n._consume_signal_future = lambda now: scope['_consume_signal_future'](n, now)
    return n


def test_candidate_changes_do_not_bypass_poll_interval():
    n = node()
    n._request_signal_if_due('42', 1000.5, 'nt')
    n._executor.submit.assert_not_called()
    for identifier, direction in (('43', 'nt'), ('42', 'et')):
        n = node()
        n._request_signal_if_due(identifier, 1000.5, direction)
        n._executor.submit.assert_not_called()
        n._request_signal_if_due(identifier, 1001, direction)
        assert n._signal_future_id == identifier
        assert n._executor.submit.call_args.kwargs['direction'] == direction
        assert n._executor.submit.call_args.kwargs['timing_required']
        assert n._executor.submit.call_args.kwargs['client'] is n._signal_client


def test_worker_start_jitter_defers_poll_without_api_failure():
    n = node()
    n._signal_client.poll_interval_remaining = lambda _: 0.002
    n._request_signal_if_due('42', 1001, 'nt')
    n._executor.submit.assert_not_called()
    n._signal_client.poll_interval_remaining = lambda _: 0.0
    n._request_signal_if_due('42', 1001.2, 'nt')
    n._executor.submit.assert_called_once()


def test_no_overlap_and_no_old_intersection_error_on_new_candidate():
    n = node()
    n._signal_future = object()
    result = n._signal_state('43', 'nt', 1001)
    n._executor.submit.assert_not_called()
    assert result == (None, False, 'awaiting signal for intersection 43')


def test_missing_mapping_and_credentials_are_distinct():
    n = node()
    assert n._signal_state('', 'nt', 1001)[2] == 'no matched V2X intersection'
    n._api_key = ''
    assert n._signal_state('42', 'nt', 1001)[2] == 'signal API key unavailable'


def test_direction_error_is_not_hidden_by_timing_request_error():
    n = node()
    n._phase_cache = {'itstId': '42', 'trsmUtcTime': 1000000,
                      'ntPdsgStatNm': 'protected-Movement-Allowed'}
    n._signal_cache = None
    n._signal_cache_time = 1000
    n._signal_cache_max_age = 12
    n._fresh = lambda *args: True
    result = n._signal_state('42', 'et', 1001)
    assert not result[1]
    assert 'direction=et' in result[2]
    assert 'old error' in result[2]


def test_endpoint_quota_error_remains_visible_after_candidate_switch():
    n = node()
    n._signal_client.poll_interval_remaining = lambda _: 28819
    n._signal_client.wait_reason = lambda _: 'signal API HTTP 429; retry in 28819s'
    result = n._signal_state('2620', 'nt', 1001)
    assert not result[1]
    assert '429' in result[2]
    n._executor.submit.assert_not_called()


def test_both_red_and_green_request_countdown():
    for phase, expected in (
        ('stop-And-Remain', True),
        ('permissive-Movement-Allowed', True),
    ):
        n = node()
        n._phase_cache = {'ntPdsgStatNm': phase}
        n._request_signal_if_due('42', 1001, 'nt', ['nt'])
        assert n._executor.submit.call_args.kwargs['timing_required'] is expected


def test_new_intersection_requests_its_own_phase_and_timing():
    n = node()
    n._phase_cache = {'ntPdsgStatNm': 'permissive-Movement-Allowed'}
    n._signal_cache_id = 'old-intersection'
    n._request_signal_if_due('new-intersection', 1001, 'nt', ['nt'])
    assert n._executor.submit.call_args.kwargs['timing_required']


def test_countdown_defers_requests_past_twelve_seconds_then_refreshes():
    n = node()
    n._signal_countdown.observe(
        {'itstId': '42', 'trsmUtcTime': 1000000, 'ntPdsgRmdrCs': 200},
        {'itstId': '42', 'trsmUtcTime': 1000000,
         'ntPdsgStatNm': 'protected-Movement-Allowed'}, 1001)
    assert n._signal_state('42', 'nt', 1015) == (5, True, 'green pedestrian signal')
    n._executor.submit.assert_not_called()
    assert not n._signal_state('42', 'nt', 1020)[1]
    n._executor.submit.assert_called_once()


def test_ambiguous_mapping_does_not_query_or_accept_consensus():
    n = node()
    n._request_signal_if_due('42', 1001, '', ['nt', 'st'])
    n._executor.submit.assert_not_called()


def test_new_direction_does_not_wait_for_old_direction_countdown():
    n = node()
    n._signal_countdown.observe(
        {'itstId': '42', 'trsmUtcTime': 1000000, 'ntPdsgRmdrCs': 200},
        {'itstId': '42', 'trsmUtcTime': 1000000,
         'ntPdsgStatNm': 'stop-And-Remain'}, 1001)
    n._request_signal_if_due('42', 1005, 'et')
    n._executor.submit.assert_called_once()


def test_worker_observation_latches_once_and_failure_cannot_extend_it():
    from concurrent.futures import Future
    n = node()
    def response(value):
        n._signal_future = Future()
        n._signal_future_id = '42'
        n._signal_future.set_result(value)
    data = {
        'timing': {'itstId': '42', 'trsmUtcTime': 1000000, 'ntPdsgRmdrCs': 200},
        'phase': {'itstId': '42', 'trsmUtcTime': 1000000,
                  'ntPdsgStatNm': 'protected-Movement-Allowed'},
    }
    response(data)
    n._consume_signal_future(1002)
    assert n._signal_state('42', 'nt', 1015)[:2] == (5, True)
    response(data)
    n._consume_signal_future(1016)
    assert n._signal_state('42', 'nt', 1019)[:2] == (1, True)
    response({'timing_error': 'timeout', 'phase_error': 'timeout'})
    n._consume_signal_future(1021)
    assert not n._signal_state('42', 'nt', 1021)[1]


def deliver(n, now, seconds=0, state='stop-And-Remain', stamp=None, identifier='42'):
    from concurrent.futures import Future
    stamp = now if stamp is None else stamp
    n._signal_future = Future()
    n._signal_future_id = identifier
    n._signal_future.set_result({
        'timing': {'itstId': identifier, 'trsmUtcTime': stamp * 1000,
                   'ntPdsgRmdrCs': seconds * 10},
        'phase': {'itstId': identifier, 'trsmUtcTime': stamp * 1000,
                  'ntPdsgStatNm': state},
    })
    n._consume_signal_future(now)


def test_zero_countdown_retries_two_seconds_after_response_at_most_three_times():
    n = node()
    n._signal_refresh = 3
    deliver(n, 1001)
    for attempt, now in enumerate((1003, 1006, 1009), 1):
        n._request_signal_if_due('42', now - .01, 'nt')
        assert n._executor.submit.call_count == attempt - 1
        assert n._signal_state('42', 'nt', now) == (0, True, 'red pedestrian signal')
        assert n._executor.submit.call_count == attempt
        # A one-second response delay: the next retry waits from completion.
        deliver(n, now + 1)
    for now in (1012, 1025, 1100):
        n._request_signal_if_due('42', now, 'nt')
    assert n._executor.submit.call_count == 3
    assert 'zero countdown retry limit' in n._signal_state('42', 'nt', 1100)[2]


def test_duplicate_zero_and_failed_requests_do_not_reset_retry_budget():
    from concurrent.futures import Future
    n = node()
    deliver(n, 1001)
    for now in (1003, 1005, 1007):
        n._request_signal_if_due('42', now, 'nt')
        if now == 1005:
            n._signal_future = Future()
            n._signal_future.set_exception(RuntimeError('timeout'))
            n._consume_signal_future(now)
        else:
            deliver(n, now, stamp=1001)
    n._request_signal_if_due('42', 1030, 'nt')
    assert n._executor.submit.call_count == 3


def test_positive_countdown_recovers_and_next_zero_has_a_new_budget():
    n = node()
    deliver(n, 1001)
    n._request_signal_if_due('42', 1003, 'nt')
    deliver(n, 1003, seconds=10)
    assert n._signal_countdown.zero_retry_count('42', ['nt']) is None
    n._request_signal_if_due('42', 1005, 'nt')
    assert n._executor.submit.call_count == 1
    n._request_signal_if_due('42', 1013, 'nt')
    assert n._executor.submit.call_count == 2
    deliver(n, 1013)
    assert n._signal_countdown.zero_retry_count('42', ['nt']) == 0
    n._request_signal_if_due('42', 1015, 'nt')
    assert n._executor.submit.call_count == 3


def test_switching_signal_heads_does_not_restart_exhausted_zero_budget():
    n = node()
    deliver(n, 1001)
    for now in (1003, 1005, 1007):
        n._request_signal_if_due('42', now, 'nt')
        deliver(n, now)
    n._request_signal_if_due('43', 1010, 'nt')
    assert n._executor.submit.call_count == 4
    deliver(n, 1010, identifier='43', seconds=10)
    n._request_signal_if_due('42', 1013, 'nt')
    assert n._executor.submit.call_count == 4


def test_zero_retries_honour_http_backoff_without_spending_budget():
    n = node()
    deliver(n, 1001)
    n._signal_client.poll_interval_remaining = lambda _: 60
    n._request_signal_if_due('42', 1003, 'nt')
    assert n._signal_countdown.zero_retry_count('42', ['nt']) == 0
    n._executor.submit.assert_not_called()
