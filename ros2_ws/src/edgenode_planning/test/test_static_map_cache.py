"""Persistent map proofs cannot hide new solids or changed motion constraints."""
from dataclasses import replace
from unittest.mock import Mock, patch

import pytest

from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from test_generalized_avoidance import corridor, rect
from test_static_obstacle_planner import setup, enter, plan
from avoidance_world import world


@pytest.mark.parametrize('change', ['boundary', 'road', 'body', 'config', 'network', 'path', 'margin'])
def test_fixed_sweep_map_proof_rechecks_all_geometry_dependencies(change):
    lanes, point, _ = world()
    lanes.boundaries = Mock()
    lanes.boundaries.swept_reason.return_value = ''
    p = StaticObstaclePlanner(lanes)
    p._road_area = corridor()
    path = [point(i*.25, 0.) for i in range(81)]
    assert p._path_safety(path, (), 3.) == ''
    calls = lanes.boundaries.swept_reason.call_count
    assert p._path_safety(path, (), 3.) == ''
    assert lanes.boundaries.swept_reason.call_count == calls
    if change == 'boundary':
        lanes.boundaries = Mock()
        lanes.boundaries.swept_reason.return_value = 'LANE_BOUNDARY_COLLISION:replacement'
    elif change == 'road': p._road_area = corridor([rect(-15., 140., 5., 9.)])
    elif change == 'body': p.body = replace(p.body, width_m=12.)
    elif change == 'config': p.config = replace(p.config, tracking_limit_m=6.)
    elif change == 'margin': p._planning_tracking_reserve = 6.
    elif change == 'network':
        p.network = world()[0]
        p.network.boundaries = lanes.boundaries
    else: path = [point(i*.25, .1) for i in range(81)]
    reason = p._path_safety(path, (), 3.)
    if change == 'boundary': assert reason == 'LANE_BOUNDARY_COLLISION:replacement'
    elif change in ('road', 'body', 'config', 'margin'): assert reason == 'ROAD_AREA_UNPROVEN'
    else:
        assert reason == ''
        assert lanes.boundaries.swept_reason.call_count > calls


def test_equal_lidar_frames_recheck_active_path_solids_but_reuse_map():
    p, point, obstacle = setup()
    enter(p, point, obstacle)
    p.network.boundaries = Mock()
    p.network.boundaries.swept_reason.return_value = ''
    with patch.object(p, '_obstacle_safety', wraps=p._obstacle_safety) as solids:
        assert plan(p, point, [], now=.1)['state'] == 'AVOID_ENTRY'
        calls = solids.call_count
        phases = []
        original = p._map_sweep_reason
        def map_check(*args):
            phases.append(p._check_phase)
            return original(*args)
        with patch.object(p, '_map_sweep_reason', side_effect=map_check):
            assert plan(p, point, [], now=.2)['state'] == 'AVOID_ENTRY'
            assert solids.call_count > calls
            # Actual PP rollout is rechecked, while fixed path sweeps reuse proof.
            assert 'PATH_SWEEP' not in phases and 'PP_ROLLOUT' in phases
        result = plan(p, point, [obstacle(18., 22., 3., 4.)], now=.3)
        assert result['blocked'] and result['reason'] == 'BODY_COLLISION'


def test_pp_rollout_never_reuses_static_proof_even_at_identical_start():
    p, point, obstacle = setup()
    enter(p, point, obstacle)
    with patch.object(p, '_map_sweep_reason', wraps=p._map_sweep_reason) as checks:
        assert p._rollout(p.selected.entry_path, (0., 0., 0.), 3., 0., p.remembered) == ''
        calls = checks.call_count
        assert p._rollout(p.selected.entry_path, (0., 0., 0.), 3., 0., p.remembered) == ''
        assert checks.call_count == calls*2 and calls > 0


def test_fixed_solid_construction_is_reused_without_rounding_path_or_body():
    p, point, _ = setup()
    path = tuple(point(i*.25, 0.) for i in range(81))
    assert p._path_safety(path, (), 3.) == ''
    with patch.object(p, '_body_geometry', wraps=p._body_geometry) as geometry:
        assert p._path_safety(path, (), 3.) == ''
        assert geometry.call_count == 0
        changed = tuple((x, y+1e-7, z) for x, y, z in path)
        assert p._path_safety(changed, (), 3.) == ''
        assert geometry.call_count > 0
        previous = geometry.call_count
        p.body = replace(p.body, width_m=2.)
        assert p._path_safety(path, (), 3.) == ''
        assert geometry.call_count > previous


def test_speed_and_observation_scope_still_apply_to_cached_geometry():
    p, point, _ = setup()
    path = tuple(point(i*.5, 0.) for i in range(81))
    assert p._path_safety(path, (), 3.) == ''
    p._observation_curvature_limit = 0.
    curved = tuple((x, .002*x*x, z) for x, _, z in path)
    assert p._path_safety(curved, (), 3.) == 'OBSERVATION_CURVATURE_SCOPE'
    p._observation_curvature_limit = None
    assert p._path_safety(curved, (), 3.) == ''
    assert p._path_safety(curved, (), 100.) == 'LATERAL_ACCELERATION'


def test_cross_sections_keep_holes_merge_shared_cells_and_respect_rotation():
    road = corridor([rect(-5, 5, -5, -2), rect(-5, 5, 0, 2), rect(-5, 5, 2, 5)])
    assert road.cross_section((0., 0.), (0., 1.)) == ((-5., -2.), (0., 5.))
    assert road.cross_section((0., 0.), (0., -1.)) == ((-5., -0.), (2., 5.))
    assert road.cross_section((6., 0.), (0., 1.)) == ()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_offsets_span_map_width_independently_of_observed_obstacle_width(side):
    lanes, point, obstacle = world(side)
    p = StaticObstaclePlanner(lanes)
    road = corridor([rect(-15., 140., -10., 10.)])
    p._hazards = (tuple(obstacle(35., 39., -.5, .5)),)
    reference = [point(i, 0.) for i in range(81)]
    first = p._road_offsets(road, reference, side)
    p._hazards = (tuple(obstacle(35., 39., -4., 4.)),)
    assert p._road_offsets(road, reference, side) == first
    assert first[0] == .2 and first[-1] > 8. and len(first) <= 9
    # Unsupported near offsets cannot hide a farther supported interval.
    separated = corridor([rect(-15., 140., -2., 2.), rect(-15., 140., 4., 10.),
                          rect(-15., 140., -10., -4.)])
    offsets = p._road_offsets(separated, reference, side)
    assert any(v > 6. for v in offsets)
    assert all(v < 1. or v > 5. for v in offsets)


def test_hold_prefix_stops_before_first_map_hole_and_rechecks_clipped_end():
    p, point, _ = setup()
    p._road_area=corridor([rect(-15., 30., -5.5, 5.5),rect(31., 140., -5.5, 5.5)])
    path=tuple(point(i*.25, 3.5) for i in range(321))
    prefix,reason=p._map_supported_prefix(path)
    assert reason=='ROAD_AREA_UNPROVEN' and prefix and prefix[-1][0]<27.
    assert p._path_safety(prefix,(),3.)==''
    # A solid inside this proven prefix still rejects it.
    assert p._path_safety(prefix,(rect(12.,14.,3.,4.),),3.)=='BODY_COLLISION'


def test_hold_map_prefix_shorter_than_control_horizon_still_stops():
    p, point, obstacle = setup()
    enter(p,point,obstacle)
    p._road_area=corridor([rect(-15.,40.,-5.5,5.5)])
    p.state='AVOID_HOLD'
    result=p._hold((30.,3.5,0.),3.,0.,{})
    assert result['blocked'] and not result['points']
    assert result['reason']=='ROAD_AREA_UNPROVEN'


@pytest.mark.parametrize('corner,blocked', [(40.,True),(90.,False)])
def test_hold_ends_before_native_untrackable_corner_without_relaxing_curvature(corner,blocked):
    from edgenode_planning.lane_geometry import LanePair
    from edgenode_planning.lattice_path import curvatures
    p,point,obstacle=setup()
    enter(p,point,obstacle)
    pair=p.pair
    adjacent=tuple((x,y+(.1 if abs(x-corner)<1e-6 else 0.),z) for x,y,z in pair.adjacent)
    p.pair=LanePair(pair.original,adjacent,pair.ids,pair.side,pair.section_ends)
    p._road_area=corridor();p.state='AVOID_HOLD'
    result=p._hold((30.,3.5,0.),3.,0.,{})
    assert result['blocked'] is blocked
    if blocked:
        assert result['reason']=='CURVATURE' and not result['points']
    else:
        assert result['points'][-1][0]<corner-1.
        native=p._hold_kinematic_points(result['points'],(30.,3.5,0.))
        assert max(map(abs,curvatures(native)))<=p.config.curvature_limit
