"""Run production polling methods without ROS/DDS dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Tuple
from unittest.mock import Mock

from safestride_navigation.signal_logic import evaluate_pedestrian_signal


def node():
    path = Path(__file__).parents[1] / 'safestride_navigation/crosswalk_controller_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_signal_state', '_request_signal_if_due')]
    scope = dict(Optional=Optional, Tuple=Tuple, request_signal_bundle=Mock(),
                 evaluate_pedestrian_signal=evaluate_pedestrian_signal)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    n = SimpleNamespace(_api_key='test', _signal_future=None,
        _last_signal_request=1000, _last_signal_request_key=('42', 'nt'),
        _signal_refresh=3, _executor=Mock(), _signal_url='timing',
        _phase_url='phase', _combined_url='combined', _signal_request_timeout=3,
        _signal_cache_id='42', _phase_cache=None, _signal_error='old error')
    n._request_signal_if_due = lambda *args: scope['_request_signal_if_due'](n, *args)
    n._signal_state = lambda *args: scope['_signal_state'](n, *args)
    return n


def test_same_head_throttled_but_new_intersection_or_direction_immediate():
    n = node()
    n._request_signal_if_due('42', 1001, 'nt')
    n._executor.submit.assert_not_called()
    for identifier, direction in (('43', 'nt'), ('42', 'et')):
        n = node()
        n._request_signal_if_due(identifier, 1001, direction)
        assert n._signal_future_id == identifier
        assert n._executor.submit.call_args.kwargs['direction'] == direction


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
