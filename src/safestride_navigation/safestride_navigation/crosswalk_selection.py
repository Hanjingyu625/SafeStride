"""Rank nearby crossings without discarding a turn off the pavement."""

import math

from .crosswalk_data import crosswalk_axis_position, nearest_crosswalk


class CrosswalkSelector:
    def __init__(self, hold_s=2.0, margin_m=3.0):
        self.hold_s = hold_s
        self.margin_m = margin_m
        self._key = None
        self._since = None
        self._last_time = None
        self._distances = {}

    def reset(self):
        self._key = self._since = self._last_time = None
        self._distances.clear()

    def select(self, records, latitude, longitude, now, *, maximum_distance_m,
               heading_deg, heading_confidence, heading_tolerance_deg=60.0):
        if self._last_time is not None and not 0 <= now - self._last_time <= 1.0:
            self.reset()
        self._last_time = now
        ranked = []
        distances = {}
        for record in records:
            item = nearest_crosswalk(
                [record], latitude, longitude, heading_deg=heading_deg,
                maximum_heading_error_deg=180.0,
                maximum_distance_m=maximum_distance_m)
            if item is None:
                continue
            position = crosswalk_axis_position(
                item, latitude, longitude, item['crossing_bearing_deg'])
            # Distance to the nearer end of the crossing, not its long side.
            entrance = math.hypot(
                abs(abs(position['along_m']) - item['length_m'] / 2.0),
                position['lateral_error_m'])
            old = self._distances.get(item['index'])
            approach_bonus = 0.0
            if old is not None and now > old[1]:
                approach_bonus = max(-2.0, min(2.0, (old[0] - entrance) / (now - old[1])))
            distances[item['index']] = (entrance, now)
            penalty = 0.0
            if heading_deg is not None:
                penalty = heading_confidence * (
                    12.0 * item['axis_alignment_error_deg'] / 90.0
                    + 12.0 * item['heading_error_deg'] / 180.0)
            item['selection_score_m'] = entrance + penalty - approach_bonus
            item['entrance_distance_m'] = entrance
            ranked.append(item)
        self._distances = distances
        if not ranked:
            self._key = self._since = None
            return None
        ranked.sort(key=lambda item: (item['selection_score_m'], item['index']))
        best = ranked[0]
        gap = (ranked[1]['selection_score_m'] - best['selection_score_m']
               if len(ranked) > 1 else math.inf)
        key = (best['index'], best['crossing_direction'], best['signal_direction'])
        clear = (heading_deg is not None and heading_confidence >= 0.5
                 and best['axis_alignment_error_deg'] <= heading_tolerance_deg
                 and (best['edge_distance_m'] <= 1.0
                      or best['heading_error_deg'] <= heading_tolerance_deg)
                 and gap >= self.margin_m)
        if key != self._key or not clear:
            self._key = key
            self._since = now if clear else None
        if clear and self._since is None:
            self._since = now
        best['selection_confirmed'] = bool(
            clear and now - self._since >= self.hold_s)
        best['selection_margin_m'] = gap
        best['selection_candidate_count'] = len(ranked)
        best['heading_confidence'] = heading_confidence
        return best
