"""Read-only current-signal check; never prints the API key or request URL."""
import argparse
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/safestride_navigation'))
from safestride_navigation.signal_logic import latest_signal_record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--key-file', type=Path, required=True)
    parser.add_argument('--intersection', required=True)
    parser.add_argument('--key-param', choices=('apikey', 'apiKey'), default='apikey')
    parser.add_argument('--legacy-check', action='store_true',
                        help='One-off historical endpoint check, never for polling')
    args = parser.parse_args()
    key = args.key_file.read_text(encoding='utf-8-sig').strip()
    endpoints = ('v2xSignalPhaseTimingInformation', 'v2xSignalPhaseInformation') if args.legacy_check else (
        'v2xSignalPhaseTimingCurrentInfo', 'v2xSignalPhaseCurrentInfo')
    for endpoint in endpoints:
        query = urllib.parse.urlencode(dict({args.key_param: key}, itstId=args.intersection,
                                           type='json', pageNo=1, numOfRows=100))
        url = 'https://t-data.seoul.go.kr/apig/apiman-gateway/tapi/' + endpoint + '/1.0?' + query
        try:
            with urllib.request.urlopen(url, timeout=10) as response:
                data = json.loads(response.read().decode('utf-8-sig'))
            record = latest_signal_record(data, args.intersection)
            fields = {k: v for k, v in record.items() if 'Pdsg' in k and v is not None}
            print(json.dumps(dict(endpoint=endpoint, status=200, intersection=record['itstId'],
                  source_age_s=round(time.time()-float(record['trsmUtcTime'])/1000, 2),
                  pedestrian=fields), ensure_ascii=False))
        except urllib.error.HTTPError as error:
            detail = error.read().decode('utf-8', errors='replace').replace(key, '[REDACTED]')
            print(json.dumps(dict(endpoint=endpoint, status=error.code,
                  retry_after=error.headers.get('Retry-After'), detail=detail[:500]), ensure_ascii=False))
        except Exception as error:
            print(json.dumps(dict(endpoint=endpoint, error=type(error).__name__)))


if __name__ == '__main__':
    main()
