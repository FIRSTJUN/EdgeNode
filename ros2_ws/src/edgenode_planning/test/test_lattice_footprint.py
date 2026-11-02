"""Footprint-aware S4 regressions, including the observed NPC width and legacy API."""

import math

import pytest

from edgenode_planning.lattice_planner import LatticePlanner


def straight_path(length=30.0, spacing=0.25):
    return [[i * spacing, 0.0, 0.0] for i in range(round(length / spacing) + 1)]


def footprint(x0=13.790 + 1.19, x1=14.941 + 1.19, y0=-0.597, y1=1.100):
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    return [(x0, y0), (xm, y0), (x1, y0), (x1, ym),
            (x1, y1), (xm, y1), (x0, y1), (x0, ym)]


def test_width_and_margin_make_required_clearance_1_2m():
    planner = LatticePlanner()
    assert planner.ego_half_width_m == 0.95
    assert planner.obstacle_safety_margin_m == 0.25
    assert planner.required_clearance_m == pytest.approx(1.2)
    assert planner.obstacle_clearance_m == planner.required_clearance_m


def test_legacy_clearance_is_a_floor_and_cannot_silently_reduce_width():
    assert LatticePlanner(obstacle_clearance_m=0.55).required_clearance_m == 1.2
    assert LatticePlanner(obstacle_clearance_m=1.4).required_clearance_m == 1.4


def test_explicit_legacy_point_model_reproduces_small_offset_selection():
    planner = LatticePlanner(candidate_offsets_m=[0, 0.6, -0.6, 0.8, -0.8],
                             obstacle_clearance_m=0.55, ego_half_width_m=0.0,
                             obstacle_safety_margin_m=0.0)
    result = planner.plan(straight_path(), [(15.283, 0.246)])
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -0.6


def test_1_7m_npc_footprint_blocks_center_path():
    planner = LatticePlanner(candidate_offsets_m=[0.0])
    result = planner.plan(straight_path(), footprint())
    assert result['status'] == 'BLOCKED'
    assert result['blocked']


@pytest.mark.parametrize('offset', [0.6, -0.6, 0.8, -0.8, 1.2, -1.2, 1.5, -1.5])
def test_small_offsets_are_unsafe_for_observed_npc(offset):
    result = LatticePlanner(candidate_offsets_m=[offset]).plan(straight_path(), footprint())
    assert result['blocked']


def test_sufficient_right_offset_clears_observed_npc_and_preserves_score():
    result = LatticePlanner().plan(straight_path(), footprint())
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == -1.8
    assert result['minimum_clearance_m'] >= 1.2


@pytest.mark.parametrize('center_y,sign', [(0.7, -1), (-0.7, 1)])
def test_obstacle_lateral_location_selects_smaller_change_on_opposite_side(center_y, sign):
    points = footprint(y0=center_y - 0.85, y1=center_y + 0.85)
    result = LatticePlanner().plan(straight_path(), points)
    assert result['status'] == 'AVOIDING'
    assert math.copysign(1.0, result['selected_offset_m']) == sign
    assert abs(result['selected_offset_m']) == 1.5


def test_centered_1_7m_car_needs_more_than_1_8m_offset():
    result = LatticePlanner().plan(straight_path(), footprint(y0=-0.85, y1=0.85))
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] == 2.1
    assert result['minimum_clearance_m'] >= 1.2


def test_small_cone_footprint_still_has_safe_candidate():
    result = LatticePlanner().plan(straight_path(), footprint(14.9, 15.1, -0.1, 0.1))
    assert result['status'] == 'AVOIDING'
    assert abs(result['selected_offset_m']) == 1.5
    assert result['minimum_clearance_m'] >= 1.2


def test_two_obstacles_can_block_every_candidate():
    points = footprint(y0=-0.85, y1=0.85) + footprint(y0=-3.0, y1=-1.3)
    result = LatticePlanner().plan(straight_path(), points)
    assert result['status'] == 'AVOIDING'
    assert result['selected_offset_m'] > 0.0
    barrier = footprint(y0=-3.0, y1=0.0) + footprint(y0=0.0, y1=3.0)
    result = LatticePlanner().plan(straight_path(), barrier)
    assert result['status'] == 'BLOCKED'
    assert result['blocked']


def test_all_blocked_retains_existing_result_contract():
    planner = LatticePlanner(candidate_offsets_m=[0.0, 0.6, -0.6, 0.8, -0.8])
    result = planner.plan(straight_path(), footprint(), current_position=(0, 0))
    assert result['status'] == 'BLOCKED'
    assert result['blocked']
    assert not result['obstacle_avoid_requested']
    assert result['points'] == straight_path()


def test_obstacle_removed_recovers_then_clears():
    planner = LatticePlanner()
    assert planner.plan(straight_path(), footprint())['status'] == 'AVOIDING'
    result = planner.plan(straight_path(), [], current_position=(0, -1.8))
    assert result['status'] == 'RECOVERING'
    assert result['points'][0][1] == pytest.approx(-1.8)
    assert result['points'][-1][1] == 0.0
    assert planner.plan(straight_path(), [], current_position=(0, 0))['status'] == 'CLEAR'


def test_obstacle_between_sparse_path_points_is_not_missed():
    planner = LatticePlanner()
    assert planner._minimum_obstacle_distance([[0, 0, 0], [10, 0, 0]], [(5, 0.1)]) == 0.1
    assert planner._minimum_obstacle_distance([[0, 0, 0], [0, 0, 0]], [(0, 2)]) == 2.0


def test_side_obstacle_outside_center_influence_also_blocks_shifted_candidate():
    # The on-route NPC activates avoidance; a second footprint outside the 2.5m
    # relevance corridor must still constrain an otherwise safe wide candidate.
    points = footprint() + footprint(y0=-3.5, y1=-3.0)
    result = LatticePlanner(candidate_offsets_m=[-2.4]).plan(straight_path(), points)
    assert result['blocked']


@pytest.mark.parametrize('kwargs', [
    {'ego_half_width_m': -1}, {'ego_half_width_m': math.nan},
    {'obstacle_safety_margin_m': -0.1}, {'obstacle_clearance_m': math.inf},
    {'candidate_offsets_m': []}, {'candidate_offsets_m': [math.nan]},
    {'transition_after_m': 0}, {'transition_after_m': math.nan},
    {'obstacle_influence_m': -1},
])
def test_invalid_footprint_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        LatticePlanner(**kwargs)


def test_wide_offset_default_return_curve_respects_existing_steering_geometry():
    # Dense geometric inspection, not a change to Local Path sampling or Control.
    import numpy as np
    planner = LatticePlanner()
    assert planner.transition_after_m == 10.0
    ref = [[float(x), 0.0, 0.0] for x in np.arange(0.0, 32.001, 0.02)]
    arc = planner._arc_lengths(ref)
    normals = planner._normals(ref)
    xy = np.array(planner._generate_avoidance_candidate(
        ref, arc, normals, 15.283, 0.0, 2.4))[:, :2]
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    dx, dy = np.gradient(xy[:, 0], s), np.gradient(xy[:, 1], s)
    k = np.abs(dx * np.gradient(dy, s) - dy * np.gradient(dx, s))
    k /= (dx * dx + dy * dy) ** 1.5
    command_curvature_limit = math.tan(0.70 * 0.610865) / 2.84
    assert 0.12 < float(np.max(k)) < command_curvature_limit
