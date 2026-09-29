"""Select the nearest polygon on every accepted GPS fix, including at rest."""


class ApproachSelector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.selected = None
        self.candidate_count = 0
        self.reason = 'no crosswalk within search range'

    def select(self, index, latitude, longitude, fix_time, *, maximum_distance_m,
               heading_deg=None):
        # Heading and approach history must not hide a nearby crossing.
        selected = index.nearest(latitude, longitude,
                                 maximum_distance_m=maximum_distance_m)
        self.selected = selected['index'] if selected else None
        self.candidate_count = selected['search_candidate_count'] if selected else 0
        self.reason = ('nearest crosswalk selected' if selected else
                       'no crosswalk within search range')
        if selected is not None:
            selected.update(selection_source='nearest_distance')
        return selected
