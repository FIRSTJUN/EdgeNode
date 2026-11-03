"""Offline regressions for the 20 km/h BLOCKED diagnosis and maneuver continuity."""

import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from edgenode_planning.lattice_planner import LatticePlanner


CASES = json.loads((Path(__file__).parent / 'fixtures/s4_20kph_blocked_scenes.json').read_text())['cases']


def road(start=0., length=35., step=.1):
    return [[start+i*step, 0., 0.] for i in range(round(length/step)+1)]


def footprint(front=14., rear=18., low=-.8, high=.8):
    middle = (front+rear)/2
    return [(front, low), (middle, low), (rear, low), (rear, (low+high)/2),
            (rear, high), (middle, high), (front, high), (front, (low+high)/2)]


@pytest.mark.parametrize('case', CASES, ids=lambda c: c['name'])
def test_recorded_first_final_and_terminal_candidates_still_block(case):
    planner = LatticePlanner()
    ref = case['reference']
    arc, normals = planner._arc_lengths(ref), planner._normals(ref)
    # Reproduce the archived algorithm's continuous geometry before adding the
    # measured heading. This locks in the original clearance failure evidence.
    for expected in case['candidates']:
        transition = planner._adaptive_transition(ref, arc, normals, expected['d0'],
                                                  expected['offset'], expected['available'], 5.)
        clearance = planner._minimum_obstacle_distance(transition['points'], case['obstacles'])
        assert clearance == pytest.approx(expected['clearance'], abs=2e-7)
        assert transition['curvature'] == pytest.approx(expected['curvature'], abs=2e-7)
        assert clearance < 1.2 or not transition['feasible']
    planner._front_point, planner._rear_point = case['front_point'], case['rear_point']
    planner._avoidance_active, planner._selected_offset = True, -2.4
    result = planner.plan(ref, case['obstacles'], case['position'], obstacle_groups=case['groups'])
    assert result['status'] == 'BLOCKED'
    assert result['actual_lateral_offset'] == pytest.approx(case['d0'])
    assert result['last_approved_offset'] == -2.4
    assert result['evaluated_candidate_offset'] == 2.4
    assert 'CURVATURE' in result['evaluated_candidate_reject_reason']
    assert result['evaluated_candidate_profile_curvature'] > .1604669205
    assert result['minimum_clearance_m'] == result['evaluated_candidate_clearance']
    # Live boundary replay also fails closed; it need not produce the legacy
    # candidate curvature because it now starts at measured Ego position/yaw.
    planner.reset()
    planner._avoidance_active, planner._selected_offset = True, -2.4
    live = planner.plan(ref, case['obstacles'], case['position'], current_heading=case['yaw'],
                        obstacle_groups=case['groups'])
    assert live['status'] == 'BLOCKED'
    assert live['last_approved_offset'] == -2.4


def test_moving_ego_restricts_approved_profile_and_carries_first_second_derivatives():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    obstacle = footprint()
    result = planner.plan(road(), obstacle, (0., 0.), current_heading=0., road_progress_m=100.)
    assert result['status'] == 'AVOIDING'
    initial_length = result['entry_length']
    initial = result['coefficients']
    previous_progress = 0.
    for progress in (1., 2., 3., 4., initial_length-.3):
        d, first, second = planner._boundary_state(initial, initial_length, progress)
        result = planner.plan(road(progress), obstacle, (progress, d),
                              current_heading=math.atan(first), road_progress_m=100.+progress)
        assert result['status'] == 'AVOIDING'
        assert result['points'][0][:2] == pytest.approx([progress, d], abs=1e-9)
        assert result['entry_length'] == pytest.approx(initial_length-progress, abs=1e-6)
        boundary = planner._boundary_state(result['coefficients'], result['profile_length'], 0.)
        assert boundary == pytest.approx((d, first, second), abs=1e-7)
        # The complete restricted polynomial equals the originally approved suffix.
        for remaining in np.linspace(0., result['entry_length'], 9):
            actual = planner._boundary_state(result['coefficients'], result['profile_length'], remaining)
            expected = planner._boundary_state(initial, initial_length, progress+remaining)
            assert actual == pytest.approx(expected, abs=1e-6)
        assert planner._maneuver_start_global_progress_m == 100.
        assert planner._maneuver_progress_m >= previous_progress
        previous_progress = planner._maneuver_progress_m
        assert result['maximum_curvature'] <= planner.usable_curvature_limit
    assert result['entry_length'] < .31  # no new 5 m entry near the achieved target


@pytest.mark.parametrize('yaw', [-.12, .12])
def test_nonzero_heading_boundary_at_exact_ego_position(yaw):
    planner = LatticePlanner(candidate_offsets_m=[math.copysign(2.4, yaw)])
    result = planner.plan(road(.2), footprint(16., 19.), (0., -.3), current_heading=yaw)
    assert result['status'] == 'AVOIDING'
    assert result['points'][0][:2] == pytest.approx([0., -.3], abs=1e-9)
    state = planner._boundary_state(result['coefficients'], result['profile_length'], 0.)
    assert math.atan(state[1]) == pytest.approx(yaw, abs=1e-10)
    first, following = result['points'][:2]
    assert math.atan2(following[1]-first[1], following[0]-first[0]) == pytest.approx(yaw, abs=.003)


def test_previous_path_is_rechecked_against_new_obstacle_not_blindly_approved():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    first = planner.plan(road(), footprint(), (0., 0.), current_heading=0.)
    assert first['status'] == 'AVOIDING'
    d, slope, _ = planner._boundary_state(first['coefficients'], first['profile_length'], 2.)
    barrier = [(6., i*.25) for i in range(-20, 21)]
    result = planner.plan(road(2.), footprint()+barrier, (2., d), current_heading=math.atan(slope),
                          obstacle_groups=[footprint(), barrier])
    assert result['status'] == 'BLOCKED'
    assert result['last_approved_offset'] == -2.4


def test_near_npc_far_patch_cannot_extend_active_rear_but_still_gets_collision_checked():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    near = footprint()
    far = footprint(28., 30., 7., 8.)
    assert planner.plan(road(), near, (0., 0.), current_heading=0., obstacle_groups=[near])['status'] == 'AVOIDING'
    with patch.object(planner, '_minimum_obstacle_distance', wraps=planner._minimum_obstacle_distance) as collision:
        result = planner.plan(road(1.), near+far, (1., -.1), current_heading=-.1,
                              obstacle_groups=[far, near])
    assert result['obstacle_front_s'] == pytest.approx(13.)
    assert result['obstacle_rear_s'] == pytest.approx(17.)
    assert planner._rear_point[0] == 18.
    assert collision.call_count > 0
    assert all(call.args[1] == near+far for call in collision.call_args_list)
    # A laterally irrelevant object on the actual shifted path must reject it.
    far_on_path = footprint(28., 30., -3.1, -2.7)
    result = planner.plan(road(), near+far_on_path, (0., 0.), current_heading=0.,
                          obstacle_groups=[near, far_on_path])
    assert result['status'] == 'BLOCKED'
    assert 'CLEARANCE' in result['evaluated_candidate_reject_reason']
    assert planner._rear_point[0] == 18.


def test_ungrouped_legacy_far_points_do_not_pollute_near_extent():
    planner = LatticePlanner()
    result = planner.plan(road(), footprint()+footprint(28., 30., 7., 8.))
    assert result['obstacle_rear_s'] == pytest.approx(18.)


@pytest.mark.parametrize('first,second,delta,length', [
    (.15, .03, 2.4, 9.), (-.3, -.08, -2.4, 8.), (.2, -.1, .1, 5.),
    (0., 0., 2.4, 8.), (-.12, .01, -1., 4.),
])
def test_general_profile_curvature_peak_matches_independent_dense_evaluation(first, second, delta, length):
    coefficients = LatticePlanner._boundary_coefficients(0., delta, length, first, second)
    u = np.linspace(0., 1., 20001)
    slope = sum(i*c*u**(i-1) for i, c in enumerate(coefficients) if i >= 1)/length
    second_derivative = sum(i*(i-1)*c*u**(i-2) for i, c in enumerate(coefficients) if i >= 2)/length**2
    dense = np.max(abs(second_derivative)/(1+slope*slope)**1.5)
    peak = LatticePlanner._boundary_maximum_curvature(coefficients, length)
    assert peak == pytest.approx(dense, rel=1e-6, abs=1e-9)


def test_no_length_cannot_erase_nonzero_heading_boundary():
    planner = LatticePlanner(candidate_offsets_m=[2.4])
    ref = road()
    result = planner._adaptive_transition(ref, planner._arc_lengths(ref), planner._normals(ref),
                                          2.4, 2.4, 0., 5., first=.2)
    assert not result['feasible']


def test_road_progress_jitter_preserves_progress_and_remaining_world_boundary():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    result = planner.plan(road(), footprint(), (0., 0.), current_heading=0., road_progress_m=100.)
    initial, length = result['coefficients'], result['profile_length']
    endpoint = planner._maneuver['end_point'][:]
    for actual_progress in (2., 1.95, 2., 2.1):
        d, first, _ = planner._boundary_state(initial, length, actual_progress)
        result = planner.plan(road(actual_progress), footprint(), (actual_progress, d),
                              current_heading=math.atan(first), road_progress_m=100.+max(2., actual_progress))
        assert result['status'] == 'AVOIDING'
        # Small boundary corrections may extend a profile to satisfy curvature;
        # they must not restart the original large transition.
        assert planner._maneuver['end_point'] == pytest.approx(endpoint, abs=.02)
        assert result['entry_length'] < length-1.8
        assert planner._maneuver_start_global_progress_m == 100.
        assert planner._maneuver_progress_m == pytest.approx(max(2., actual_progress))


def test_recovery_suffix_keeps_boundary_below_old_lateral_completion_threshold():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    assert planner.plan(road(), footprint(), (0., 0.), current_heading=0.)['status'] == 'AVOIDING'
    recovery = planner.plan(road(19.), [], (19., -2.4), current_heading=0.)
    assert recovery['status'] == 'RECOVERING'
    length, coefficients = recovery['profile_length'], recovery['coefficients']
    progress = .85*length
    d, first, second = planner._boundary_state(coefficients, length, progress)
    assert abs(d) < planner.recovery_threshold_m
    result = planner.plan(road(19.+progress), [], (19.+progress, d), current_heading=math.atan(first))
    assert result['status'] == 'RECOVERING'
    assert result['points'][0][:2] == pytest.approx([19.+progress, d])
    assert result['recovery_length'] == pytest.approx(length-progress, abs=1e-6)
    assert planner._boundary_state(result['coefficients'], result['profile_length'], 0.) == pytest.approx((d, first, second))
    clear = planner.plan(road(19.+length), [], (19.+length, 0.), current_heading=0.)
    assert clear['status'] == 'CLEAR'


def test_exhausted_avoidance_horizon_cannot_fall_back_to_an_unchecked_path():
    planner = LatticePlanner(candidate_offsets_m=[-2.4])
    planner.plan(road(), footprint(), (0., 0.), current_heading=0.)
    result = planner.plan(road(19., length=.2), [], (19.15, -2.4), current_heading=0.)
    assert result['status'] == 'BLOCKED'
    assert result['blocked_reason'] == 'INVALID_AVOIDANCE_REFERENCE'
