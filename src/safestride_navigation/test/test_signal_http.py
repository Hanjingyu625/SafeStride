import io
import json
from urllib.error import HTTPError
from urllib.parse import parse_qs, urlsplit
from unittest.mock import Mock, patch

import pytest

from safestride_navigation.signal_logic import (
    DEFAULT_TIMING_URL, SignalApiClient, SignalApiError,
    current_signal_url, request_signal_data, retry_after_seconds,
    load_signal_api_key,
)


def test_historical_endpoint_migrates_but_custom_is_preserved():
    old = DEFAULT_TIMING_URL.replace('TimingCurrentInfo', 'TimingInformation')
    assert current_signal_url(old) == DEFAULT_TIMING_URL
    assert current_signal_url('https://example.test/custom') == 'https://example.test/custom'


def test_current_request_uses_official_key_parameter():
    response = Mock()
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    response.read.return_value = json.dumps({'itstId': '42', 'trsmUtcTime': 1000}).encode()
    with patch('urllib.request.urlopen', return_value=response) as opener:
        request_signal_data('secret', '42')
    query = parse_qs(urlsplit(opener.call_args.args[0].full_url).query)
    assert query['apikey'] == ['secret']
    assert 'apiKey' not in query


@pytest.mark.parametrize('status,header,delay', [(429, '60', 60), (429, None, 300), (404, None, 300)])
def test_http_error_backoff_and_secret_redaction(status, header, delay):
    error = HTTPError('https://example.test', status, 'failure',
                      {'Retry-After': header} if header else {}, io.BytesIO(b'secret'))
    with patch('urllib.request.urlopen', side_effect=error):
        with pytest.raises(SignalApiError) as caught:
            request_signal_data('secret', '42')
    assert caught.value.retry_after_s == delay
    assert caught.value.status == status
    assert 'secret' not in str(caught.value)


def test_retry_after_http_date_and_invalid_value():
    assert retry_after_seconds('Thu, 01 Jan 1970 00:01:00 GMT', now=10) == 50
    assert retry_after_seconds('invalid') == 300
    assert retry_after_seconds(None) == 300


def test_backoff_survives_intersection_changes_and_expires():
    clock = Mock(return_value=100)
    fetch = Mock(side_effect=[SignalApiError('HTTP 429', retry_after_s=300), {'ok': True}])
    client = SignalApiClient(clock=clock, fetch=fetch)
    with pytest.raises(SignalApiError):
        client.get('secret', '42', url=DEFAULT_TIMING_URL, timeout_s=3)
    clock.return_value = 103
    with pytest.raises(SignalApiError, match='retry in'):
        client.get('secret', '43', url=DEFAULT_TIMING_URL, timeout_s=3)
    assert fetch.call_count == 1
    clock.return_value = 400
    assert client.get('secret', '43', url=DEFAULT_TIMING_URL, timeout_s=3) == {'ok': True}


def test_normal_polling_allows_one_second_but_not_candidate_bursts():
    clock = Mock(return_value=100)
    fetch = Mock(return_value={'ok': True})
    client = SignalApiClient(clock=clock, fetch=fetch)
    client.get('test', '42', url=DEFAULT_TIMING_URL, timeout_s=3)
    clock.return_value = 100.5
    assert client.poll_interval_remaining([DEFAULT_TIMING_URL]) == 0.5
    with pytest.raises(SignalApiError):
        client.get('test', '43', url=DEFAULT_TIMING_URL, timeout_s=3)
    clock.return_value = 101
    assert client.poll_interval_remaining([DEFAULT_TIMING_URL]) == 0
    client.get('test', '42', url=DEFAULT_TIMING_URL, timeout_s=3)
    assert fetch.call_count == 2


def test_saved_key_loads_without_prompt_and_explicit_missing_path_is_not_ignored(tmp_path, monkeypatch):
    saved = tmp_path / 'key.txt'
    saved.write_text('\ufeffsaved-test-key\n', encoding='utf-8')
    monkeypatch.setenv('SAFESTRIDE_SIGNAL_API_KEY_FILE', str(saved))
    assert load_signal_api_key() == 'saved-test-key'
    assert load_signal_api_key(str(tmp_path / 'missing')) == ''
