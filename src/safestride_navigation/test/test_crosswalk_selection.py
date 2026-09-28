from safestride_navigation.crosswalk_selection import CrosswalkSelector
from safestride_navigation.crosswalk_guidance import describe_crosswalk
from safestride_navigation.crossing_policy import CrossingStateMachine


def crossing(index=1, east=0, north=0, axis=0):
    return dict(index=index, latitude=north / 111320, longitude=east / 111320,
                length_m=10.0, width_m=3.0, axis_bearing_deg=axis,
                intersection_id=str(index))


def select(selector, records, now, heading=0, confidence=1):
    return selector.select(records, -8 / 111320, 0, now,
                           maximum_distance_m=80, heading_deg=heading,
                           heading_confidence=confidence)


def stable(selector, records, heading=0):
    result = None
    for tick in range(11):
        result = select(selector, records, tick / 5, heading)
    return result


def update(machine, candidate):
    return machine.update(candidate=candidate, intersection_id=str(candidate['index']),
                          latitude=-8 / 111320, longitude=0,
                          signal_remaining_s=60, signal_valid=True,
                          safe_speed_mps=1, measured_speed_mps=0)


def test_side_crossing_retained_until_turn_then_confirmed_after_hold():
    selector = CrosswalkSelector()
    records = [crossing(east=8, north=-8, axis=90)]
    before = select(selector, records, 0)
    assert before is not None
    assert not before['selection_confirmed']
    assert not select(selector, records, 0.2, 90)['selection_confirmed']
    for tick in range(2, 12):
        after = select(selector, records, tick / 5, 90)
    assert after['selection_confirmed']
    assert after['crossing_bearing_deg'] == 90


def test_unknown_heading_or_equally_good_candidates_never_allow_entry():
    for heading, records in ((None, [crossing()]),
                             (0, [crossing(1), crossing(2)])):
        selector = CrosswalkSelector()
        machine = CrossingStateMachine()
        candidate = stable(selector, records, heading)
        assert not candidate['selection_confirmed']
        update(machine, candidate)
        assert machine.state == 'WAIT_AT_CURB'
        assert not machine.command(1, 0)['entry_allowed']
        assert machine.command(1, 0)['target_speed_mps'] == 0
        assert describe_crosswalk(machine.state, machine.reason, True, True)['state'] == '방향 확인 중'


def test_gps_jump_or_gap_does_not_retain_confirmation():
    selector = CrosswalkSelector()
    assert stable(selector, [crossing()])['selection_confirmed']
    assert not select(selector, [crossing(2)], 2.2)['selection_confirmed']
    assert not select(selector, [crossing(2)], 5)['selection_confirmed']
    assert select(selector, [crossing(north=200)], 5.2) is None


def test_confidence_loss_revokes_prior_entry_and_intersection_before_signal_lookup():
    selector = CrosswalkSelector()
    candidate = stable(selector, [crossing()])
    machine = CrossingStateMachine()
    update(machine, candidate)
    assert machine.state == 'ENTRY_ALLOWED'
    uncertain = select(selector, [crossing()], 2.2, confidence=0.4)
    machine.current_crosswalk(uncertain, -8 / 111320, 0)
    assert machine.locked_intersection_id == ''
    assert not machine.command(1, 0)['entry_allowed']
    update(machine, uncertain)
    assert machine.state == 'WAIT_AT_CURB'


def test_new_crosswalk_or_reverse_direction_clears_previous_grant():
    candidate = stable(CrosswalkSelector(), [crossing()])
    for replacement in (
        dict(candidate, index=2, intersection_id='2'),
        dict(candidate, crossing_direction='st', signal_direction='wt',
             crossing_bearing_deg=180),
    ):
        machine = CrossingStateMachine()
        update(machine, candidate)
        preview = machine.current_crosswalk(replacement, -8 / 111320, 0)
        assert preview['signal_direction'] == replacement['signal_direction']
        assert machine.locked_intersection_id == ''
        assert machine.state == 'IDLE'
        assert not machine.command(1, 0)['entry_allowed']


def test_crossing_keeps_locked_crosswalk_despite_uncertainty_or_new_candidate():
    candidate = stable(CrosswalkSelector(), [crossing()])
    for state in ('CROSSING', 'CROSSING_URGENT', 'EXITING'):
        machine = CrossingStateMachine()
        update(machine, candidate)
        machine.set_state(state, 'test crossing')
        replacement = dict(candidate, index=2, selection_confirmed=False)
        preview = machine.current_crosswalk(replacement, 0, 0)
        assert preview['index'] == 1
        assert machine.locked_intersection_id == '1'
        assert machine.state == state


def test_entry_corridor_is_checked_on_first_lock_not_one_tick_later():
    candidate = stable(CrosswalkSelector(), [crossing(east=6)])
    assert candidate['selection_confirmed']
    machine = CrossingStateMachine()
    update(machine, candidate)
    assert machine.state == 'APPROACHING'
    assert not machine.command(1, 0)['entry_allowed']
