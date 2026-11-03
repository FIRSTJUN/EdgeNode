"""Adaptive profile regressions from the real stopped 3 km/h S4 scene."""

import json
import math
from pathlib import Path

import pytest

from edgenode_planning.lattice_planner import LatticePlanner


SCENE = json.loads((Path(__file__).parent / 'fixtures/s4_3kph_blocked_scene.json').read_text())


def reference(start=0.0, length=35.0, spacing=0.25):
    return [[start + i * spacing, 0.0, 0.0] for i in range(round(length / spacing) + 1)]


def obstacle(front=14.0, rear=18.0, half_width=0.8):
    middle = (front + rear) / 2
    return [(front, -half_width), (middle, -half_width), (rear, -half_width),
            (rear, 0.0), (rear, half_width), (middle, half_width),
            (front, half_width), (front, 0.0)]


def scene_planner(history=False):
    planner = LatticePlanner()
    planner._front_point = SCENE['remembered_front_point']
    planner._rear_point = SCENE['remembered_rear_point']
    if history:
        planner._avoidance_active = True
        planner._selected_offset = SCENE['previous_selected_offset']
        planner._blocked = True
    return planner


def test_real_old_candidate_reproduces_runtime_curvature_without_relaxing_limit():
    planner = LatticePlanner()
    curvature = planner.maximum_curvature(SCENE['reconstructed_old_best_candidate_path'])
    assert curvature == pytest.approx(0.1983951, abs=5e-7)
    assert curvature == pytest.approx(SCENE['recorded_max_curvature'], abs=5e-8)
    assert not planner.steering_feasible(SCENE['reconstructed_old_best_candidate_path'])
    assert planner.usable_curvature_limit == pytest.approx(0.16046692048050848, abs=1e-14)
    assert SCENE['recorded_front_s'] == pytest.approx(8.0152235, abs=1e-6)
    assert SCENE['recorded_rear_s'] == pytest.approx(10.6054745, abs=1e-6)


def test_real_scene_gets_feasible_entry_before_obstacle_front():
    planner = scene_planner()
    result = planner.plan(SCENE['reference'], SCENE['obstacles'], SCENE['position'])
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -2.1
    assert result['entry_length'] == pytest.approx(7.26918, abs=2e-5)
    assert result['full_offset_start_s'] < result['obstacle_front_s']
    assert result['maximum_curvature'] <= planner.usable_curvature_limit
    assert planner.maximum_curvature(result['points']) <= planner.usable_curvature_limit
    assert result['minimum_clearance_m'] == pytest.approx(1.2285867, abs=2e-6)
    assert result['obstacle_rear_s'] == pytest.approx(10.6054745, abs=1e-6)
    assert math.isnan(result['recovery_length'])
    # The endpoint must actually exist in the published polyline, not only in
    # metadata between two vertices straddling the obstacle front.
    arc = planner._arc_lengths(SCENE['reference'])
    normals = planner._normals(SCENE['reference'])
    sample = next(v for v in planner._profile_reference(
        SCENE['reference'], arc, normals, result['full_offset_start_s'])
        if v[1] == result['full_offset_start_s'])
    point = [sample[0][i] + sample[2][i] * result['selected_offset_m'] for i in range(2)]
    assert any(math.hypot(q[0] - point[0], q[1] - point[1]) < 1e-9 for q in result['points'])
    endpoint_s = planner._project_reference(SCENE['reference'], arc, point)[0]
    assert endpoint_s < result['obstacle_front_s']


def test_real_blocked_history_still_requires_safe_release_and_keeps_direction():
    planner = scene_planner(history=True)
    for _ in range(planner.blocked_release_cycles - 1):
        result = planner.plan(SCENE['reference'], SCENE['obstacles'], SCENE['position'])
        assert result['status'] == 'BLOCKED'
        assert result['blocked_reason'] == 'RELEASE_PENDING'
        assert result['entry_length'] > 0
    result = planner.plan(SCENE['reference'], SCENE['obstacles'], SCENE['position'])
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -2.1


@pytest.mark.parametrize('offset,current,front,half_width,expected', [
    (1.5, 1.3, 12.0, 0.1, 'AVOIDING'),
    (-1.5, -1.3, 12.0, 0.1, 'AVOIDING'),
    (2.4, 0.0, 14.0, 0.8, 'AVOIDING'),
    (-2.4, 0.0, 14.0, 0.8, 'AVOIDING'),
    (2.4, 0.0, 4.0, 0.8, 'BLOCKED'),
    (-2.4, 0.0, 4.0, 0.8, 'BLOCKED'),
    (2.1, 0.0, 24.0, 0.8, 'AVOIDING'),
    (-2.1, 0.0, 24.0, 0.8, 'AVOIDING'),
])
def test_small_large_near_far_left_right_entries(offset, current, front, half_width, expected):
    planner = LatticePlanner(candidate_offsets_m=[offset])
    result = planner.plan(reference(), obstacle(front, front + 2, half_width), (0.0, current))
    assert result['status'] == expected
    if expected == 'AVOIDING':
        assert result['full_offset_start_s'] < front
        assert result['maximum_curvature'] <= planner.usable_curvature_limit
        assert result['minimum_clearance_m'] >= planner.required_clearance_m
        assert all(q[1] == pytest.approx(offset) for q in result['points'] if q[0] >= front)
    else:
        assert result['blocked_reason'] == 'NO_SAFE_FEASIBLE_CANDIDATE'
        assert result['maximum_curvature'] > planner.usable_curvature_limit


def test_far_obstacle_does_not_stretch_entry_until_last_moment():
    results = [LatticePlanner(candidate_offsets_m=[2.1]).plan(
        reference(), obstacle(front, front + 2)) for front in (12.0, 24.0)]
    assert all(r['status'] == 'AVOIDING' for r in results)
    assert results[0]['entry_length'] == pytest.approx(results[1]['entry_length'])
    assert results[1]['full_offset_start_s'] < 12.0


def test_tight_front_deadline_is_not_bypassed_by_larger_horizon():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    needed = planner._required_transition_length(2.1)
    for horizon in (20.0, 60.0):
        result = planner.plan(reference(length=horizon), obstacle(needed - 1e-4, needed + 2))
        assert result['status'] == 'BLOCKED'
    planner.reset()
    result = planner.plan(reference(), obstacle(needed + 1e-4, needed + 2))
    assert result['status'] == 'AVOIDING'
    assert result['full_offset_start_s'] < result['obstacle_front_s']


def test_nominal_continuous_curvature_is_not_hidden_by_sparse_samples():
    planner = LatticePlanner(candidate_offsets_m=[2.4])
    points = [[0., 0., 0.], [20., 0., 0.], [40., 0., 0.]]
    arc = planner._arc_lengths(points)
    transition = planner._adaptive_transition(
        points, arc, planner._normals(points), 0.0, 2.4, 4.0, 5.0)
    assert not transition['feasible']
    assert transition['curvature'] > planner.usable_curvature_limit
    assert transition['required_entry_length'] > 4.0


def test_reference_curvature_can_require_more_than_straight_quintic_length():
    planner = LatticePlanner()
    radius = 12.0
    points = [[radius * math.sin(s / radius), radius * (1 - math.cos(s / radius)), 0.0]
              for s in [i * 0.25 for i in range(121)]]
    arc = planner._arc_lengths(points)
    transition = planner._adaptive_transition(
        points, arc, planner._normals(points), 0., 2.1, 20., 5.)
    assert transition['feasible']
    assert transition['entry_length'] > transition['required_entry_length']
    assert transition['curvature'] <= planner.usable_curvature_limit


def test_recovery_extends_instead_of_using_infeasible_fixed_length():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    assert planner.plan(reference(), obstacle())['status'] == 'AVOIDING'
    result = planner.plan(reference(19), [], (19, 4.0))
    assert result['status'] == 'RECOVERING'
    assert result['recovery_length'] > planner.transition_after_m
    assert result['maximum_curvature'] <= planner.usable_curvature_limit
    assert result['points'][-1][1] == 0.0


def test_recovery_cannot_invent_missing_horizon():
    planner = LatticePlanner(candidate_offsets_m=[2.1])
    planner.plan(reference(), obstacle())
    result = planner.plan(reference(19, length=6), [], (19, 4.0))
    assert result['status'] == 'BLOCKED'
    assert result['blocked_reason'] == 'RECOVERY_UNSAFE'


@pytest.mark.parametrize('sign', [-1, 1])
def test_alongside_obstacle_holds_achieved_safe_offset_without_new_entry(sign):
    planner = LatticePlanner(candidate_offsets_m=[sign * 2.4])
    assert planner.plan(reference(), obstacle())['status'] == 'AVOIDING'
    result = planner.plan(reference(15), obstacle(), (15, sign * 2.37))
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == pytest.approx(sign * 2.37)
    assert result['entry_length'] == 0.0
    assert all(p[1] == pytest.approx(sign * 2.37) for p in result['points'])
    assert result['minimum_clearance_m'] >= planner.required_clearance_m
    assert result['maximum_curvature'] <= planner.usable_curvature_limit


def test_alongside_failed_tracking_does_not_get_an_unsafe_instant_shift():
    planner = LatticePlanner(candidate_offsets_m=[2.4])
    planner.plan(reference(), obstacle())
    result = planner.plan(reference(15), obstacle(), (15, 1.5))
    assert result['status'] == 'BLOCKED'


@pytest.mark.parametrize('delta,length', [(0.2, 5.0), (2.1, 7.3), (-2.4, 8.0), (4., 11.)])
def test_continuous_peak_matches_independent_dense_derivative_evaluation(delta, length):
    import numpy as np
    t = np.linspace(0., 1., 10001)
    first = delta / length * 30 * t**2 * (1 - t)**2
    second = delta / length**2 * 60 * t * (1 - t) * (1 - 2 * t)
    expected = np.max(abs(second) / (1 + first**2)**1.5)
    assert LatticePlanner._quintic_maximum_curvature(delta, length) == pytest.approx(
        expected, rel=1e-6)
