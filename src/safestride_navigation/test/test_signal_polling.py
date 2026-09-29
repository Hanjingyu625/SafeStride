"""Run production polling methods without ROS/DDS dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Tuple
from unittest.mock import Mock

from safestride_navigation.signal_logic import (
    evaluate_crosswalk_signal, timing_required_for_phase, cached_signal_window,
)


def node():
    path = Path(__file__).parents[1] / 'safestride_navigation/crosswalk_controller_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_signal_state', '_request_signal_if_due')]
    scope = dict(Optional=Optional, Tuple=Tuple, request_signal_bundle=Mock(),
                 cached_signal_window=cached_signal_window,
                 evaluate_crosswalk_signal=evaluate_crosswalk_signal,
                 timing_required_for_phase=timing_required_for_phase)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    n = SimpleNamespace(_api_key='test', _signal_future=None,
        _last_signal_request=1000, _last_signal_request_key=('42', 'nt'),
        _signal_refresh=1, _executor=Mock(), _signal_url='timing',
        _phase_url='phase', _combined_url='combined', _signal_request_timeout=3,
        _signal_cache_id='42', _phase_cache=None, _signal_error='old error',
        _signal_cache=None, _signal_cache_time=None, _signal_cache_max_age=12,
        _signal_client=SimpleNamespace(poll_interval_remaining=lambda _: 0.0,
                                       wait_reason=lambda _: ''))
    n._request_signal_if_due = lambda *args: scope['_request_signal_if_due'](n, *args)
    n._signal_state = lambda *args: scope['_signal_state'](n, *args)
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


def test_both_phases_request_countdown_for_next_transition():
    for phase, expected in (
        ('stop-And-Remain', True),
        ('permissive-Movement-Allowed', True),
    ):
        n = node()
        n._phase_cache = {'ntPdsgStatNm': phase}
        n._request_signal_if_due('42', 1001, 'nt', ['nt'])
        assert n._executor.submit.call_args.kwargs['timing_required'] is expected


def test_phase_from_previous_intersection_never_triggers_countdown_request():
    n = node()
    n._phase_cache = {'ntPdsgStatNm': 'permissive-Movement-Allowed'}
    n._signal_cache_id = 'old-intersection'
    n._request_signal_if_due('new-intersection', 1001, 'nt', ['nt'])
    assert n._executor.submit.call_args.kwargs['timing_required']


def test_poll_waits_for_phase_deadline_then_resumes():
    n = node()
    n._signal_cache_time = 1002
    n._phase_cache = dict(itstId='42', trsmUtcTime=1000000,
                          ntPdsgStatNm='permissive-Movement-Allowed')
    n._signal_cache = dict(itstId='42', trsmUtcTime=1000000, ntPdsgRmdrCs=200)
    n._request_signal_if_due('42', 1019, 'nt')
    n._executor.submit.assert_not_called()
    n._request_signal_if_due('42', 1020, 'nt')
    n._executor.submit.assert_called_once()
