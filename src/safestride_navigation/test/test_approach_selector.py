import pytest

from safestride_navigation.approach_selector import ApproachSelector
from safestride_navigation.crosswalk_data import CrosswalkSpatialIndex
from safestride_navigation.crossing_policy import CrossingStateMachine


def crossing(index=1, east=0, north=20):
    return dict(index=index, latitude=north / 111320, longitude=east / 111320,
                length_m=10.0, width_m=3.0, axis_bearing_deg=0.0)


@pytest.mark.parametrize('heading', [None, float('nan'), float('inf')])
def test_missing_or_invalid_heading_selects_nearest(heading):
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1), crossing(2, north=-10)])
    selected = selector.select(index, 0, 0, 1, maximum_distance_m=80, heading_deg=heading)
    assert selected['index'] == 2
    assert selected['selection_source'] == 'nearest_distance'


def test_new_closest_is_selected_immediately_and_out_of_range_clears():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1), crossing(2, north=60)])
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80)['index'] == 1
    assert selector.select(index, 50 / 111320, 0, 2, maximum_distance_m=80)['index'] == 2
    assert selector.select(index, 200 / 111320, 0, 3, maximum_distance_m=80) is None
    assert selector.selected is None


def test_nearest_uses_polygon_edge_not_centre():
    selector = ApproachSelector()
    long = dict(crossing(1, north=35), length_m=60)
    short = crossing(2, north=20)
    index = CrosswalkSpatialIndex([long, short])
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80)['index'] == 1


@pytest.mark.parametrize('state', ['CROSSING', 'CROSSING_URGENT'])
def test_departed_crossing_releases_after_distinct_fixes_even_130_metres_away(state):
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1, north=0), crossing(2, north=145)])
    machine = CrossingStateMachine(clock=lambda: 0)
    machine.lock(selector.select(index, -5 / 111320, 0, 0, maximum_distance_m=80), '2620')
    machine.set_state(state, 'crossing timeout')
    for t in (10, 11):
        selected = selector.select(index, 135 / 111320, 0, t, maximum_distance_m=80)
        machine.reconsider_candidate(selected, 135 / 111320, 0, fix_time=t)
        assert machine.locked_intersection_id == '2620'
    machine.reconsider_candidate(selected, 135 / 111320, 0, fix_time=12)
    assert machine.locked_crosswalk is None
    assert machine.locked_intersection_id == ''
    assert machine.current_crosswalk(selected, 135 / 111320, 0)['index'] == 2
    assert machine.state == 'IDLE'
    assert 'completed' not in machine.reason


def test_one_outlier_or_repeated_tick_cannot_release_on_road_track():
    index = CrosswalkSpatialIndex([crossing(1, north=0), crossing(2, east=20, north=0)])
    machine = CrossingStateMachine(clock=lambda: 100)
    old = index.nearest(-5 / 111320, 0, maximum_distance_m=80)
    machine.lock(old, '42')
    machine.set_state('CROSSING', 'test')
    new = index.nearest(0, 20 / 111320, maximum_distance_m=80)
    for _ in range(30):
        machine.reconsider_candidate(new, 0, 20 / 111320, fix_time=10)
    assert machine.locked_crosswalk['index'] == 1
    machine.reconsider_candidate(old, 0, 0, fix_time=11)
    machine.reconsider_candidate(new, 0, 20 / 111320, fix_time=12)
    assert machine.locked_crosswalk['index'] == 1


def test_departed_track_releases_even_when_no_new_crosswalk_exists():
    machine = CrossingStateMachine(clock=lambda: 0)
    index = CrosswalkSpatialIndex([crossing(1, north=0)])
    machine.lock(index.nearest(-5 / 111320, 0, maximum_distance_m=80), '42')
    machine.set_state('CROSSING_URGENT', 'test')
    for t in (0, 1, 2):
        machine.reconsider_candidate(None, 130 / 111320, 0, fix_time=t)
    assert machine.locked_crosswalk is None


def test_nearest_lock_keeps_resolved_signal_mapping_on_next_fix():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    machine = CrossingStateMachine()
    selected = selector.select(index, 0, 0, 0, maximum_distance_m=80)
    machine.lock(dict(selected, signal_direction='et'), '42')
    machine.set_state('WAIT_AT_CURB', 'test')
    candidate = selector.select(index, 0, 0, 1, maximum_distance_m=80)
    machine.reconsider_candidate(candidate, 0, 0, fix_time=1)
    assert machine.current_crosswalk(candidate, 0, 0)['signal_direction'] == 'et'
    assert machine.locked_intersection_id == '42'


def test_retreat_excludes_even_without_replacement_and_stop_keeps_exclusion():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t, north in enumerate([0, -1.2, -2.4]):
        selected = selector.select(index, north / 111320, 0, t, maximum_distance_m=80)
    assert selected is None
    for t in range(3, 12):
        assert selector.select(index, -2.4 / 111320, 0, t, maximum_distance_m=80) is None
    for t, north in enumerate([-1.2, 0, 1.2, 2.4, 3.6, 4.8], 12):
        selected = selector.select(index, north / 111320, 0, t, maximum_distance_m=80)
    assert selected['index'] == 1


def test_retreat_selects_other_crosswalk():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(), crossing(2, north=-40)])
    for t, north in enumerate([0, -1.2, -2.4]):
        selected = selector.select(index, north / 111320, 0, t, maximum_distance_m=80)
    assert selected['index'] == 2


def test_jitter_single_jump_and_repeated_fix_do_not_exclude():
    for positions in ([0, -.2, .3, -.3, .1], [0, 0, -3, -3, -3]):
        selector = ApproachSelector()
        index = CrosswalkSpatialIndex([crossing()])
        for t, north in enumerate(positions):
            assert selector.select(index, north / 111320, 0, t, maximum_distance_m=80)
    for _ in range(20):
        assert selector.select(index, -10 / 111320, 0, 4, maximum_distance_m=80)


def test_gps_gap_resets_retreat_evidence():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t, north in enumerate([0, -1.2, -2.4]):
        selector.select(index, north / 111320, 0, t, maximum_distance_m=80)
    assert selector.selected is None
    assert selector.select(index, -2.4 / 111320, 0, 10, maximum_distance_m=80)


def test_heading_selects_ahead_instead_of_closer_behind_and_falls_back_when_lost():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1), crossing(2, north=-10)])
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80, heading_deg=0)['index'] == 1
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80, heading_deg=180)['index'] == 2
    selected = selector.select(index, 0, 0, 1, maximum_distance_m=80)
    assert selected['index'] == 2
    assert selected['selection_source'] == 'nearest_distance'


def test_valid_heading_without_aligned_crossing_does_not_choose_sideways():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80, heading_deg=90) is None
    assert selector.select(index, 0, 0, 1, maximum_distance_m=80,
                           heading_deg=90, heading_tolerance_deg=100)['index'] == 1


def test_heading_filter_preserves_retreat_exclusion():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t, north in enumerate([0, -1.2, -2.4]):
        selected = selector.select(index, north / 111320, 0, t,
                                   maximum_distance_m=80, heading_deg=0)
    assert selected is None
