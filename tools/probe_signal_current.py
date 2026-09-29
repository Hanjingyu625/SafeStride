"""Bounded check of the production signal client using the saved API key."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/safestride_navigation'))
from safestride_navigation.signal_logic import (
    DEFAULT_PHASE_URL, DEFAULT_TIMING_URL, OPPOSITE_DIRECTION, SignalApiClient,
    evaluate_pedestrian_signal, load_signal_api_key, request_signal_bundle,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--key-file', default='', help='Optional override of automatic key lookup')
    parser.add_argument('--intersection', required=True)
    parser.add_argument('--samples', type=int, choices=range(1, 31), default=1, metavar='1..30')
    args = parser.parse_args()
    key = load_signal_api_key(args.key_file)
    if not key:
        parser.error('No saved signal API key found')
    client = SignalApiClient()
    for sample in range(args.samples):
        started = time.monotonic()
        result = request_signal_bundle(key, args.intersection, url=DEFAULT_TIMING_URL,
                                      phase_url=DEFAULT_PHASE_URL, timeout_s=3, client=client)
        now = time.time()
        timing, phase = result.get('timing'), result.get('phase')
        heads = {}
        for direction in OPPOSITE_DIRECTION:
            if (phase or {}).get(direction + 'PdsgStatNm') is None:
                continue
            remaining, valid, reason = evaluate_pedestrian_signal(timing, phase, direction, now)
            heads[direction] = dict(valid=valid, remaining_s=remaining, reason=reason,
                                    phase=phase[direction + 'PdsgStatNm'])
        print(json.dumps(dict(sample=sample + 1, intersection=args.intersection,
              timing_received=timing is not None, phase_received=phase is not None,
              source_timestamps={name: (result.get(name) or {}).get('trsmUtcTime')
                                 for name in ('timing', 'phase')},
              errors={k: v for k, v in result.items() if k.endswith('_error')},
              pedestrian=heads), ensure_ascii=False), flush=True)
        if any(result.get(name + '_error') for name in ('timing', 'phase')):
            break
        if sample + 1 < args.samples:
            # Small scheduling allowance prevents probing faster than the client limit.
            time.sleep(max(0, 1.05 - (time.monotonic() - started)))


if __name__ == '__main__':
    main()
