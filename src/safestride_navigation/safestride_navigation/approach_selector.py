"""Select nearby crossings, excluding sustained retreat on distinct GPS fixes."""

from collections import deque

from .crosswalk_data import crosswalk_edge_distance_m, nearest_crosswalk, number


class ApproachSelector:
    WINDOW_S = 6.0
    MIN_SPAN_S = 2.0
    DISTANCE_CHANGE_M = 2.0
    MAX_FIX_GAP_S = 3.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.selected = None
        self.candidate_count = 0
        self.reason = 'no crosswalk within search range'
        self._history = {}
        self._excluded = set()
        self._last_fix = None

    def _trend(self, history, sign):
        if len(history) < 3 or history[-1][0] - history[0][0] < self.MIN_SPAN_S:
            return False
        distances = [sign * distance for _, distance in history]
        middle = len(distances) // 2
        # Progress on both halves rejects a single jump followed by a plateau.
        return (distances[-1] - distances[0] >= self.DISTANCE_CHANGE_M
                and distances[middle] - distances[0] >= 0.5
                and distances[-1] - distances[middle] >= 0.5
                and sum(b >= a - 0.4 for a, b in zip(distances, distances[1:]))
                >= 0.7 * (len(distances) - 1))

    def select(self, index, latitude, longitude, fix_time, *, maximum_distance_m,
               heading_deg=None, heading_tolerance_deg=60.0):
        if self._last_fix is not None and (
                fix_time < self._last_fix or fix_time - self._last_fix > self.MAX_FIX_GAP_S):
            self.reset()
        fresh = self._last_fix is None or fix_time > self._last_fix
        records = [record for record in index.nearby(latitude, longitude, maximum_distance_m)
                   if crosswalk_edge_distance_m(record, latitude, longitude) <= maximum_distance_m]
        self.candidate_count = len(records)
        if fresh:
            present = {record['index'] for record in records}
            self._excluded.intersection_update(present)
            self._history = {key: value for key, value in self._history.items() if key in present}
            for record in records:
                key = record['index']
                history = self._history.setdefault(key, deque(maxlen=100))
                history.append((fix_time, crosswalk_edge_distance_m(record, latitude, longitude)))
                while history and fix_time - history[0][0] > self.WINDOW_S:
                    history.popleft()
                if key in self._excluded:
                    if self._trend(history, -1):
                        self._excluded.remove(key)
                        history.clear()
                elif self._trend(history, 1):
                    self._excluded.add(key)
                    history.clear()
            self._last_fix = fix_time
        heading = number(heading_deg)
        eligible = [record for record in records if record['index'] not in self._excluded]
        selected = nearest_crosswalk(
            eligible,
            latitude, longitude, maximum_distance_m=maximum_distance_m,
            heading_deg=heading, maximum_heading_error_deg=heading_tolerance_deg)
        self.selected = selected['index'] if selected else None
        self.reason = ('nearest heading-aligned crosswalk selected' if selected and heading is not None else
                       'nearest crosswalk selected' if selected else
                       'no crosswalk aligned with heading' if eligible and heading is not None else
                       'all nearby crosswalks receding' if records else
                       'no crosswalk within search range')
        if selected is not None:
            selected.update(selection_source='nearest_heading' if heading is not None else 'nearest_distance',
                            search_candidate_count=self.candidate_count)
        return selected
