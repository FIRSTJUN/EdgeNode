"""HOLD continues safely before rear passage; observed loss is not recovery."""

import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

from edgenode_planning.lattice_planner import LatticePlanner


SCENES = json.loads((Path(__file__).parent / 'fixtures/s4_continuity_hold_scenes.json').read_text())['cases']


def road(start=0., length=35.):
    return [[start+i*.25, 0., 0.] for i in range(round(length/.25)+1)]


def footprint(front=14., rear=18., low=-.5, high=.5):
    middle = (front+rear)/2
    return [(front, low), (middle, low), (rear, low), (rear, (low+high)/2),
            (rear, high), (middle, high), (front, high), (front, (low+high)/2)]


def approved_planner():
    planner = LatticePlanner(candidate_offsets_m=[-1.8, -2.1, -2.4])
    initial = planner.plan(road(), footprint(), (0., 0.), current_heading=0.)
    assert initial['status'] == 'AVOIDING'
    assert initial['last_approved_offset'] == -1.8
    return planner


def assert_checked(result, planner):
    assert result['evaluated_candidate_reject_reason'] != 'NOT_EVALUATED'
    assert math.isfinite(result['evaluated_candidate_offset'])
    assert math.isfinite(result['evaluated_candidate_clearance'])
    if result['status'] == 'AVOIDING':
        assert result['minimum_clearance_m'] >= 1.2
        assert result['maximum_curvature'] <= planner.usable_curvature_limit


def test_safe_hold_near_approved_target_with_unpassed_rear_and_only_far_observation():
    planner = approved_planner()
    far = footprint(28., 30., 7., 8.)
    result = planner.plan(road(), far, (0., -1.75), current_heading=0., obstacle_groups=[far])
    assert result['status'] == 'AVOIDING'
    assert result['maneuver_phase'] == 'HOLD'
    assert result['last_approved_offset'] == -1.8
    assert result['obstacle_rear_s'] == pytest.approx(18.)
    assert result['points'][0][:2] == pytest.approx([0., -1.75])
    assert all(-1.8-1e-9 <= p[1] <= -1.75+1e-9 for p in result['points'])
    assert_checked(result, planner)


def test_recovery_is_forbidden_until_actual_rear_plus_margin_but_hold_is_allowed():
    planner = approved_planner()
    for position in (16., 18.4, 18.5):
        result = planner.plan(road(position), [], (position, -1.75), current_heading=0.)
        assert result['status'] == 'AVOIDING'
        assert_checked(result, planner)
    after = planner.plan(road(18.6), [], (18.6, -1.75), current_heading=0.)
    assert after['status'] == 'RECOVERING'
    assert after['points'][-1][1] == pytest.approx(0.)


def test_unsafe_hold_falls_back_to_other_safe_feasible_same_direction_candidate():
    planner = approved_planner()
    expanded = footprint(low=-.8, high=.8)
    with patch.object(planner, '_adaptive_transition', wraps=planner._adaptive_transition) as evaluate:
        result = planner.plan(road(), expanded, (0., -1.75), current_heading=0., obstacle_groups=[expanded])
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -2.1
    targets = [call.args[4] for call in evaluate.call_args_list]
    assert targets[0] == -1.8
    assert -2.1 in targets and -2.4 in targets
    assert_checked(result, planner)


def test_all_candidates_fail_after_hold_is_evaluated():
    planner = approved_planner()
    barrier = [(4., i*.25) for i in range(-20, 21)]
    result = planner.plan(road(), barrier, (0., -1.75), current_heading=0., obstacle_groups=[barrier])
    assert result['status'] == 'BLOCKED'
    assert result['blocked_reason'] == 'NO_SAFE_FEASIBLE_CANDIDATE'
    assert_checked(result, planner)


def test_blocked_release_still_needs_three_checked_safe_hold_cycles():
    planner = approved_planner()
    planner._blocked = True
    for i in range(planner.blocked_release_cycles):
        result = planner.plan(road(), [], (0., -1.75), current_heading=0.)
        assert_checked(result, planner)
        assert result['status'] == ('AVOIDING' if i==2 else 'BLOCKED')
        if i < 2:
            assert result['blocked_reason'] == 'RELEASE_PENDING'
            assert result['last_approved_offset'] == -1.8


def test_missing_obstacle_does_not_erase_known_unsafe_occupancy():
    planner = approved_planner()
    barrier = [(14., i*.25) for i in range(-20, 21)]
    # A connected, newly observed occupied region must survive later loss.
    observed = planner.plan(road(), barrier, (0., -1.75), current_heading=0.,
                            obstacle_groups=[barrier])
    assert observed['status'] == 'BLOCKED'
    far = footprint(28., 30., 7., 8.)
    result = planner.plan(road(), far, (0., -1.75), current_heading=0., obstacle_groups=[far])
    assert result['status'] == 'BLOCKED'
    assert result['evaluated_candidate_clearance'] < 1.2
    assert_checked(result, planner)


@pytest.mark.parametrize('scene', SCENES, ids=lambda s: str(s['cycle']))
def test_recorded_departure_and_first_not_passed_cycles_evaluate_safe_paths(scene):
    planner = LatticePlanner()
    state = dict(scene['state_before'])
    memory = state.pop('remembered_obstacle_points')
    planner.__dict__.update(state)
    planner._remembered_obstacle_points = {tuple(p) for p in memory}
    assert planner._maneuver is not None
    assert planner._selected_offset == -1.8
    prior_target = planner._selected_offset
    previous_rear = list(planner._rear_point)
    groups = [scene['obstacles'][i:i+8] for i in range(0, len(scene['obstacles']), 8)]
    for _ in range(planner.blocked_release_cycles):
        result = planner.plan(scene['reference'], scene['obstacles'], scene['position'],
                              current_heading=scene['yaw'], obstacle_groups=groups,
                              road_progress_m=scene['road_progress_m'])
        assert_checked(result, planner)
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == prior_target
    assert result['last_approved_offset'] == prior_target
    assert planner._rear_point == previous_rear or scene['cycle'] == 21
    assert result['points'][0][:2] == pytest.approx(scene['position'], abs=1e-8)
    assert result['obstacle_rear_s'] > 0.
    if scene['cycle'] != 21:
        assert result['maneuver_phase'] == 'HOLD'
        assert result['obstacle_front_s'] == pytest.approx(scene['recorded']['lattice_obstacle_front_s'], abs=2e-6)
        assert result['obstacle_rear_s'] == pytest.approx(scene['recorded']['lattice_obstacle_rear_s'], abs=2e-6)


def test_recorded_negative_projection_roundoff_cannot_prepend_degenerate_segment():
    scene = next(s for s in SCENES if s['cycle']==21)
    planner = LatticePlanner()
    reference = scene['reference']
    arc = planner._arc_lengths(reference)
    projected_s = planner._project_reference(reference, arc, scene['position'])[0]
    assert -1e-9 < projected_s < 0.
    trimmed = planner._ego_reference(reference, scene['position'])
    assert len(trimmed) == len(reference)
    assert all(math.dist(a[:2], b[:2]) > 1e-9 for a,b in zip(trimmed,trimmed[1:]))
    assert math.hypot(*planner._normals(trimmed)[0]) == pytest.approx(1.)
