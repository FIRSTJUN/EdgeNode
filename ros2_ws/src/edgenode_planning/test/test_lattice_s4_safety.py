"""S4 collision regressions: recorded paths, actual progress and state continuity."""

import json
import math
from pathlib import Path

import pytest

from edgenode_planning.lattice_planner import LatticePlanner


FIXTURE = json.loads((Path(__file__).parent / 'fixtures/s4_wide_failed_paths.json').read_text())


def path(start=0.0, length=35.0, step=0.25):
    return [[start + i * step, 0.0, 0.0] for i in range(round(length / step) + 1)]


def footprint(near=14.0, far=18.0, low=-0.8, high=0.8):
    middle = (near + far) / 2
    return [(near, low), (middle, low), (far, low), (far, (low + high) / 2),
            (far, high), (middle, high), (near, high), (near, (low + high) / 2)]


@pytest.mark.parametrize('case', FIXTURE['cases'], ids=['9.477s', '9.578s', '11.177s'])
def test_recorded_clearance_safe_path_is_steering_infeasible(case):
    planner = LatticePlanner()
    assert planner.maximum_curvature(case['path']) == pytest.approx(
        case['expected_curvature'], abs=2e-7)
    assert planner._minimum_obstacle_distance(case['path'], case['obstacles']) >= 1.2
    assert not planner.steering_feasible(case['path'])


@pytest.mark.parametrize('case', FIXTURE['cases'], ids=['9.477s', '9.578s', '11.177s'])
def test_recorded_scene_never_approves_an_infeasible_generated_candidate(case):
    result = LatticePlanner().plan(case['reference'], case['obstacles'], case['position'])
    if case['t'] < 9.5:
        # Adaptive entry can find a different feasible path; the recorded
        # original path is still rejected by the test immediately above.
        assert result['status'] == 'AVOIDING'
        assert result['maximum_curvature'] <= result['usable_curvature_limit']
        assert result['minimum_clearance_m'] >= 1.2
        assert result['full_offset_start_s'] < result['obstacle_front_s']
    else:
        assert result['status'] == 'BLOCKED'
        assert result['blocked_reason'] == 'NO_SAFE_FEASIBLE_CANDIDATE'


def test_curvature_limit_matches_existing_control_and_parameterized_factor():
    limit = math.tan(0.70 * 0.610865) / 2.84
    assert LatticePlanner().usable_curvature_limit == pytest.approx(limit)
    assert LatticePlanner(curvature_safety_factor=0.95).usable_curvature_limit == pytest.approx(
        0.95 * limit)


def test_clearance_safe_but_steering_infeasible_is_blocked():
    planner = LatticePlanner(candidate_offsets_m=[2.4])
    result = planner.plan(path(), [(4.0, 0.0)])
    assert result['minimum_clearance_m'] >= 1.2
    assert result['maximum_curvature'] > result['usable_curvature_limit']
    assert result['status'] == 'BLOCKED'


def test_clearance_safe_and_steering_feasible_is_avoiding():
    result = LatticePlanner().plan(path(), footprint())
    assert result['status'] == 'AVOIDING'
    assert result['minimum_clearance_m'] >= 1.2
    assert result['maximum_curvature'] <= result['usable_curvature_limit']


def test_same_side_safe_candidate_wins_over_opposite_side_score():
    planner = LatticePlanner(candidate_offsets_m=[-2.1, 2.1])
    assert planner.plan(path(), footprint())['selected_offset_m'] == -2.1
    # Actual localization crosses slightly left, so a stateless score favors +.
    result = planner.plan(path(), footprint(), current_position=(0.0, 0.1))
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -2.1


def test_opposite_side_requires_blocked_then_stable_safe_near_center_transition():
    planner = LatticePlanner(candidate_offsets_m=[-2.1, 2.1])
    assert planner.plan(path(), footprint())['selected_offset_m'] == -2.1
    obstacle = footprint() + footprint(14, 18, -3.5, -2.2)
    # Only the other side is safe; it cannot reverse in the same frame.
    first = planner.plan(path(), obstacle, current_position=(0, 0))
    assert first['status'] == 'BLOCKED'
    assert first['blocked_reason'] == 'DIRECTION_CHANGE_REQUIRES_STOP'
    for _ in range(planner.blocked_release_cycles - 1):
        assert planner.plan(path(), obstacle, (0, 0))['status'] == 'BLOCKED'
    result = planner.plan(path(), obstacle, (0, 0))
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == 2.1
    assert result['maximum_curvature'] <= result['usable_curvature_limit']


def test_committed_actual_offset_forbids_immediate_opposite_transition():
    planner = LatticePlanner(candidate_offsets_m=[-2.1, 2.1])
    assert planner.plan(path(), footprint())['selected_offset_m'] == -2.1
    obstacle = footprint() + footprint(14, 18, -3.5, -2.2)
    for _ in range(8):
        result = planner.plan(path(), obstacle, (0, -0.8))
        assert result['status'] == 'BLOCKED'


def test_keep_previous_offset_only_while_it_is_collision_safe_and_feasible():
    planner = LatticePlanner(candidate_offsets_m=[-1.8, -2.1, 2.1])
    result = planner.plan(path(), footprint(low=-0.5, high=0.8))
    assert result['selected_offset_m'] == -1.8
    result = planner.plan(path(), footprint(low=-0.8, high=0.8))
    assert result['selected_offset_m'] == -2.1
    assert result['status'] == 'AVOIDING'


def test_avoidance_profile_holds_full_offset_past_entire_obstacle():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    result = planner.plan(path(), footprint())
    assert result['status'] == 'AVOIDING'
    assert result['obstacle_front_s'] == pytest.approx(14.)
    assert result['obstacle_rear_s'] == pytest.approx(18.)
    # Even future path points beyond rear do not start a premature return.
    assert all(p[1] == pytest.approx(2.1) for p in result['points'] if p[0] >= 14)


def test_recovery_requires_actual_rear_plus_margin_progress():
    planner = LatticePlanner(candidate_offsets_m=[2.1], obstacle_rear_margin_m=0.5)
    obstacle = footprint()
    assert planner.plan(path(), obstacle)['status'] == 'AVOIDING'
    before = planner.plan(path(18.4), obstacle, (18.4, 2.1))
    assert before['status'] == 'AVOIDING'
    at_margin = planner.plan(path(18.5), obstacle, (18.5, 2.1))
    assert at_margin['status'] == 'AVOIDING'
    after = planner.plan(path(18.6), obstacle, (18.6, 2.1))
    assert after['status'] == 'RECOVERING'
    assert after['points'][0][1] == pytest.approx(2.1)
    assert after['points'][-1][1] == pytest.approx(0.)
    assert after['maximum_curvature'] <= after['usable_curvature_limit']


def test_shrinking_or_missing_footprint_cannot_erase_rear_memory():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    assert planner.plan(path(), footprint())['status'] == 'AVOIDING'
    shortened = footprint(14, 15)
    result = planner.plan(path(16), shortened, (16, 2.1))
    assert result['status'] == 'AVOIDING'
    assert result['minimum_clearance_m'] >= planner.required_clearance_m
    missing = planner.plan(path(16), [], (16, 2.1))
    assert missing['status'] == 'AVOIDING'
    assert missing['evaluated_candidate_reject_reason'] == ''
    assert planner._rear_point[0] == pytest.approx(18.)
    assert planner.plan(path(18.5), [], (18.5, 2.1))['status'] == 'AVOIDING'
    assert planner.plan(path(19), [], (19, 2.1))['status'] == 'RECOVERING'


def test_one_safe_frame_does_not_release_blocked_and_failures_reset_count():
    planner = LatticePlanner()
    barrier = [(14.0, i * 0.25) for i in range(-20, 21)]
    assert planner.plan(path(), barrier)['status'] == 'BLOCKED'
    assert planner.plan(path(), footprint())['blocked_reason'] == 'RELEASE_PENDING'
    assert planner.plan(path(), barrier)['status'] == 'BLOCKED'
    for _ in range(planner.blocked_release_cycles - 1):
        assert planner.plan(path(), footprint())['status'] == 'BLOCKED'
    assert planner.plan(path(), footprint())['status'] == 'AVOIDING'


def test_failed_or_missing_reference_breaks_release_persistence():
    planner = LatticePlanner()
    planner.plan(path(), [(14.0, i * 0.25) for i in range(-20, 21)])
    planner.plan(path(), footprint())
    assert planner.plan([], footprint()) is None
    for _ in range(planner.blocked_release_cycles - 1):
        assert planner.plan(path(), footprint())['status'] == 'BLOCKED'


@pytest.mark.parametrize('points', [
    [[0, 0], [0, 0], [0, 0]], [[0, 0], [1, 0], [0, 0]],
    [[0, 0], [float('nan'), 0], [2, 0]],
])
def test_invalid_or_reversing_curvature_fails_closed(points):
    assert not LatticePlanner().steering_feasible(points)


def test_duplicate_samples_do_not_change_straight_curvature():
    assert LatticePlanner.maximum_curvature([[0, 0], [1, 0], [1, 0], [2, 0]]) == 0.


@pytest.mark.parametrize('kwargs', [
    {'wheelbase_m': 0}, {'wheelbase_m': math.nan},
    {'max_wheel_angle_rad': math.inf}, {'max_wheel_angle_rad': math.pi},
    {'max_front_steer_normalized': 1.1}, {'max_front_steer_normalized': 0},
    {'curvature_safety_factor': 1.01}, {'curvature_safety_factor': 0},
    {'obstacle_rear_margin_m': -0.1}, {'obstacle_rear_margin_m': math.nan},
    {'blocked_release_cycles': 1}, {'blocked_release_cycles': True},
    {'blocked_release_cycles': 2.5},
])
def test_invalid_safety_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        LatticePlanner(**kwargs)


def test_recovery_is_also_steering_checked():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    planner.plan(path(), footprint())
    result = planner.plan(path(19), [], (19, 4.0))
    assert result['status'] == 'RECOVERING'
    assert result['recovery_length'] > planner.transition_after_m
    assert result['maximum_curvature'] <= result['usable_curvature_limit']


def test_signed_progress_handles_duplicate_endpoint_samples_and_ego_projection():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    planner.plan(path(), footprint())
    reference = path(19)
    reference.insert(0, reference[0][:])
    arc = planner._arc_lengths(reference)
    assert planner._project_reference(reference, arc, (18.0, 0.0))[0] == pytest.approx(-1.0)
    # Local progress may lag the pose; compare extent against Ego arc, not s=0.
    planner.reset()
    planner.plan(path(), footprint())
    assert planner.plan(path(17), footprint(), (19, 2.1))['status'] == 'RECOVERING'
