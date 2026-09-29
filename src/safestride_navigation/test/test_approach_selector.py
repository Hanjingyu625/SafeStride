import pytest

from safestride_navigation.approach_selector import ApproachSelector
from safestride_navigation.crosswalk_data import CrosswalkSpatialIndex
from safestride_navigation.crossing_policy import CrossingStateMachine


def crossing(index=1, east=0, north=20):
    return dict(index=index, latitude=north / 111320, longitude=east / 111320,
                length_m=10.0, width_m=3.0, axis_bearing_deg=0.0)


def sample(selector, index, t, north, heading=None):
    return selector.select(index, north / 111320, 0.0, t,
                           maximum_distance_m=80.0, heading_deg=heading)


@pytest.mark.parametrize('heading', [None, 180.0])
def test_approach_without_heading_or_with_wrong_heading(heading):
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t in range(5):
        assert sample(selector, index, t, t, heading) is None
    selected = sample(selector, index, 5, 5, heading)
    assert selected['index'] == 1
    assert selected['approach_confirmed']


def test_stationary_jitter_and_single_jump_do_not_select():
    index = CrosswalkSpatialIndex([crossing()])
    for positions in ([0, .3, -.2, .4, 0, -.4, .2, 0, .1],
                      [0, 0, 0, 4, 4, 4, 4, 4, 4]):
        selector = ApproachSelector()
        for t, north in enumerate(positions):
            assert sample(selector, index, t, north) is None


def test_equal_approaches_remain_ambiguous():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1, -5), crossing(2, 5)])
    for t in range(8):
        assert sample(selector, index, t, t) is None
    assert selector.reason == 'ambiguous approaching crosswalks'
    assert selector.candidate_count == 2


def test_repeated_tick_cannot_create_progress_or_finish_hold():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t in range(4):
        assert sample(selector, index, t, t) is None
    for _ in range(30):
        assert sample(selector, index, 3, 3) is None
    assert len(selector.history) == 4


def test_stop_preserves_selection_retreat_releases_and_gap_clears_history():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t in range(6):
        selected = sample(selector, index, t, t)
    assert selected['index'] == 1
    for t in range(6, 18):
        assert sample(selector, index, t, 5)['index'] == 1
    for t in range(18, 22):
        selected = sample(selector, index, t, 5 - (t - 17))
    assert selected is None
    assert sample(selector, index, 30, 10) is None
    assert len(selector.history) == 1


def test_moving_away_never_selects():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing()])
    for t in range(10):
        assert sample(selector, index, t, -t) is None


def test_closer_receding_crosswalk_loses_to_approaching_crosswalk():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1), crossing(2, north=-5)])
    for t in range(6):
        selected = sample(selector, index, t, t)
    assert selected['index'] == 1


def test_turning_reselects_using_distance_without_heading():
    selector = ApproachSelector()
    index = CrosswalkSpatialIndex([crossing(1, north=30), crossing(2, east=30, north=5)])
    for t in range(6):
        selected = sample(selector, index, t, t)
    assert selected['index'] == 1
    for t in range(6, 23):
        selected = selector.select(index, 5 / 111320, (t - 5) / 111320, t,
                                   maximum_distance_m=80)
    assert selected['index'] == 2


def test_lock_can_change_without_heading_only_before_entry_after_hold():
    now = [0.0]
    machine = CrossingStateMachine(clock=lambda: now[0])
    index = CrosswalkSpatialIndex([crossing(1, 10), crossing(2)])
    old = index.nearest(0, 0, maximum_distance_m=80)
    old['index'] = 1
    candidate = dict(old, index=2, approach_confirmed=True, approach_gain_m=3.0)
    machine.lock(old, '42')
    machine.set_state('APPROACHING', 'test')
    machine.reconsider_candidate(candidate, 0, 0, None)
    assert machine.locked_crosswalk is not None
    now[0] = 2.0
    machine.reconsider_candidate(candidate, 0, 0, None)
    assert machine.locked_crosswalk is None
    machine.lock(old, '42')
    machine.set_state('CROSSING', 'test')
    machine.reconsider_candidate(candidate, 0, 0, None)
    now[0] = 10.0
    machine.reconsider_candidate(candidate, 0, 0, None)
    assert machine.locked_crosswalk is not None
