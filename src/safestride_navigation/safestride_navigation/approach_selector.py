"""Select approaching polygons from fresh GPS distance history, without a compass gate."""

import math
from collections import deque
from statistics import median

from .crosswalk_data import (
    angular_difference_deg, crosswalk_edge_distance_m, nearest_crosswalk,
)


class ApproachSelector:
    # Metres and seconds; evaluated on distinct accepted GPS fixes, not ROS ticks.
    WINDOW_S = 8.0
    MIN_SPAN_S = 3.0
    MIN_GAIN_M = 2.0
    HOLD_S = 2.0
    AMBIGUITY_M = 1.5
    SWITCH_MARGIN_M = 3.0
    RETREAT_M = 3.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.history = deque(maxlen=100)
        self.selected = None
        self.minimum_distance = math.inf
        self.pending = None
        self.pending_since = None
        self.candidate_count = 0
        self.reason = 'waiting for sustained distance decrease'

    def select(self, index, latitude, longitude, fix_time, *, maximum_distance_m,
               heading_deg=None):
        if self.history and (fix_time < self.history[-1][0]
                             or fix_time - self.history[-1][0] > 2.0):
            self.reset()
        fresh = not self.history or fix_time > self.history[-1][0]
        if fresh:
            self.history.append((fix_time, latitude, longitude))
        while self.history and fix_time - self.history[0][0] > self.WINDOW_S:
            self.history.popleft()
        records = index.nearby(latitude, longitude, maximum_distance_m)
        self.candidate_count = len(records)
        evaluated = {}
        eligible = []
        for record in records:
            item = nearest_crosswalk([record], latitude, longitude,
                                     maximum_distance_m=maximum_distance_m)
            if item is None:
                continue
            distances = [crosswalk_edge_distance_m(record, lat, lon)
                         for _, lat, lon in self.history]
            gain = 0.0
            confirmed = False
            if len(distances) >= 4 and fix_time - self.history[0][0] >= self.MIN_SPAN_S:
                size = max(1, len(distances) // 3)
                gain = median(distances[:size]) - median(distances[-size:])
                # Regression and distributed progress reject a single jump followed
                # by a plateau. Small backwards steps tolerate GPS quantisation.
                times = [t - self.history[0][0] for t, _, _ in self.history]
                mean_t, mean_d = sum(times) / len(times), sum(distances) / len(distances)
                slope = sum((t - mean_t) * (d - mean_d)
                            for t, d in zip(times, distances)) / sum(
                                (t - mean_t) ** 2 for t in times)
                middle = len(distances) // 2
                consistent = sum(b <= a + 0.4 for a, b in zip(distances, distances[1:]))
                confirmed = (gain >= self.MIN_GAIN_M and slope <= -0.2
                             and distances[0] - distances[middle] >= 0.5
                             and distances[middle] - distances[-1] >= 0.5
                             and consistent >= 0.7 * (len(distances) - 1))
            item.update(approach_confirmed=confirmed, approach_gain_m=gain,
                        search_candidate_count=len(records), selection_source='distance_trend')
            # At most one metre of tie-breaking: heading can never exclude a polygon.
            penalty = (angular_difference_deg(heading_deg, item['target_bearing_deg']) / 180.0
                       if heading_deg is not None and math.isfinite(heading_deg) else 0.0)
            item['approach_score'] = item['edge_distance_m'] - min(max(gain, 0.0), 6.0) * 0.5 + penalty
            evaluated[item['index']] = item
            if confirmed:
                eligible.append(item)
        current = evaluated.get(self.selected)
        if current is not None:
            self.minimum_distance = min(self.minimum_distance, current['edge_distance_m'])
            if current['edge_distance_m'] > self.minimum_distance + self.RETREAT_M:
                current = None
        if current is None:
            self.selected = None
            self.minimum_distance = math.inf
        eligible.sort(key=lambda item: (item['approach_score'], item['index']))
        best = eligible[0] if eligible else None
        ambiguous = (len(eligible) > 1 and
                     eligible[1]['approach_score'] - eligible[0]['approach_score'] < self.AMBIGUITY_M)
        if ambiguous:
            best = None
        if best is not None and current is not None and best['index'] != current['index']:
            if current['approach_score'] - best['approach_score'] < self.SWITCH_MARGIN_M:
                best = None
        if best is None or best['index'] == self.selected:
            self.pending = self.pending_since = None
        elif fresh:
            if self.pending != best['index']:
                self.pending, self.pending_since = best['index'], fix_time
            elif fix_time - self.pending_since >= self.HOLD_S:
                self.selected = best['index']
                self.minimum_distance = best['edge_distance_m']
                current = best
                self.pending = self.pending_since = None
        self.reason = ('distance trend selected' if current is not None else
                       'ambiguous approaching crosswalks' if ambiguous else
                       'waiting for sustained distance decrease')
        return current
