"""Measure the unchanged gate and round-trip real/stateful planner snapshots."""
import copy
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner
from edgenode_planning.corridor_diagnostics import inspect_corridor
from edgenode_planning.corridor_scene import (
    capture_scene, json_value, planner_snapshot, replay_scene, write_scene,
)
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork, LanePair, corridor_covers
from test_adjacent_lane_planner import network, obstacle, plan, road


@pytest.mark.parametrize('side,sign', [('left', 1.), ('right', -1.)])
def test_first_and_maximum_failure_have_actual_projections_and_bounds(side, sign):
    pair = network((side,)).pair('O', side, ['O'])
    quads = pair.quads(0., 15.)
    points = [[0., 0.], [3., sign*3.6], [6., sign*4.2], [12., sign*3.5]]
    before = copy.deepcopy((points, quads, pair.original, pair.adjacent))
    report = inspect_corridor(points, quads, pair)
    assert not report['safe']
    assert report['failed_index'] == 1
    assert report['failed_point_count'] == 2
    assert report['max_violation_m'] == pytest.approx(.7)
    first, maximum = report['first_failure'], report['maximum_violation_point']
    assert first['world_xy'] == points[1]
    assert first['violation_m'] == pytest.approx(.1)
    assert maximum['candidate_point_index'] == 2
    assert maximum['original_projection']['world_xy'] == [6., 0.]
    assert maximum['adjacent_projection']['world_xy'] == [6., sign*3.5]
    assert maximum['signed_lateral_m'] == pytest.approx(sign*4.2)
    assert maximum['corridor_lower_m'] == pytest.approx(min(0., sign*3.5))
    assert maximum['corridor_upper_m'] == pytest.approx(max(0., sign*3.5))
    assert maximum['lateral_violation_m'] == pytest.approx(.7)
    assert (points, quads, pair.original, pair.adjacent) == before
    assert not corridor_covers(points, quads)


def test_segment_hole_is_reported_even_with_zero_failed_vertices():
    quads = [[[0., 0.], [4., 0.], [4., 3.5], [0., 3.5]],
             [[6., 0.], [10., 0.], [10., 3.5], [6., 3.5]]]
    report = inspect_corridor([[1., 1.], [9., 1.]], quads)
    assert not report['safe']
    assert report['failed_point_count'] == 0
    assert report['max_violation_m'] == 0.  # Explicitly a vertex metric.
    assert report['reason'] == 'UNCOVERED_SEGMENT_INTERIOR'
    witness = report['first_segment_failure']
    assert witness['segment_start_index'] == 0
    assert witness['world_xy'] == pytest.approx([5., 1.], abs=1e-6)
    assert witness['violation_m'] == pytest.approx(1., abs=1e-6)


def test_tiny_numeric_excursion_and_centimetre_excursion_keep_existing_gate_results():
    quad = [[[0., 0.], [10., 0.], [10., 3.5], [0., 3.5]]]
    tiny = [[1., 1.], [5., 3.5+1e-9], [9., 3.5]]
    cm = [[1., 1.], [5., 3.51], [9., 3.5]]
    assert corridor_covers(tiny, quad)
    assert inspect_corridor(tiny, quad)['safe']
    assert not corridor_covers(cm, quad)
    report = inspect_corridor(cm, quad)
    assert report['max_violation_m'] == pytest.approx(.01)
    assert report['first_failure']['nearest_quad_edges'][2]['effective_gate_tolerance_m'] == pytest.approx(1e-8)


def test_empty_envelope_has_unknown_distance_instead_of_fabricated_zero_violation():
    pair = network(('left',)).pair('O', 'left', ['O'])
    report = inspect_corridor([[0., 0.], [5., 2.]], [], pair)
    assert not report['safe']
    assert report['reason'] == 'EMPTY_OR_INVALID_ENVELOPE'
    assert report['max_violation_m'] is None
    assert report['first_failure']['corridor_lower_m'] is None
    assert report['first_failure']['original_projection']['world_xy'] == [0., 0.]
    json.dumps(report, allow_nan=False)


def test_curved_tangent_discontinuity_no_longer_invalidates_real_boundaries():
    radius = 1000.
    original = [[radius*math.sin(i*.5/radius), radius*(1-math.cos(i*.5/radius)), 0.] for i in range(81)]
    adjacent = [[(radius-3.5)*math.sin(i*.5/radius),
                 radius-(radius-3.5)*math.cos(i*.5/radius), 0.] for i in range(81)]
    pair = LanePair(original, adjacent, [('O', 'L')], 'left', [40.])
    quads = pair.quads(2., 22.)
    # Sampling normal reversals are not crossings of the actual map geometry.
    assert quads
    points = pair.suffix('adjacent', pair.at(2.)['adjacent_s'], pair.at(22.)['adjacent_s'])
    report = inspect_corridor(points, quads, pair, [2., 22.])
    assert report['safe']
    assert report['max_violation_m']==0.
    a,t=pair.at(10.)['adjacent']
    outside=[a[0]-.01*t[1],a[1]+.01*t[0]]
    assert not corridor_covers([outside,outside],quads)


def test_curved_projections_report_distinct_local_normals_and_world_envelope_distance():
    original = [[i*.5, .003*(i*.5)**2, 0.] for i in range(81)]
    adjacent = [[i*.5, 3.5+.006*(i*.5)**2, 0.] for i in range(81)]
    pair = LanePair(original, adjacent, [('O', 'L')], 'left', [40.])
    quads = pair.quads(2., 22.)
    assert quads
    report = inspect_corridor([[3., 1.], [10., 6.], [20., 3.]], quads, pair)
    maximum = report['maximum_violation_point']
    assert maximum['original_projection']['segment_index'] >= 0
    assert abs(maximum['normal_angle_difference_rad']) > .01
    assert maximum['violation_m'] > 1.
    assert maximum['corridor_lower_m'] < maximum['corridor_upper_m']


def _capture(planner, x=0., y=0., yaw=0.):
    inputs = dict(reference_points=road(x), obstacles=obstacle(), current_position=[x, y],
                  current_heading=yaw, obstacle_groups=None, road_progress_m=x,
                  current_link_id='O', route_link_ids=['O'], speed_kmh=20.)
    before = planner_snapshot(planner)
    result = planner.plan(**inputs)
    scene = capture_scene(planner, before, inputs, result, 123456789, 'map')
    return result, scene


@pytest.mark.parametrize('side,sign', [('left', 1.), ('right', -1.)])
@pytest.mark.parametrize('phase', ['ENTRY', 'ADJACENT_LANE_HOLD', 'RECOVERING', 'BLOCKED', 'RELEASE_PENDING'])
def test_json_roundtrip_preserves_candidate_state_and_all_gate_decisions(tmp_path, side, sign, phase):
    planner = AdjacentLanePlanner(network((side,)))
    if phase in ('ADJACENT_LANE_HOLD', 'RECOVERING'):
        plan(planner)
    elif phase == 'RELEASE_PENDING':
        plan(planner, yaw=-sign*.12)
    if phase == 'BLOCKED':
        result, scene = _capture(planner, yaw=-sign*.12)
    elif phase in ('ADJACENT_LANE_HOLD', 'RECOVERING'):
        result, scene = _capture(planner, x=17. if phase == 'ADJACENT_LANE_HOLD' else 22.6, y=sign*3.5)
    else:
        result, scene = _capture(planner)
    assert result['blocked_reason'] == 'RELEASE_PENDING' if phase == 'RELEASE_PENDING' else result['lane_change_phase'] == phase
    after = planner_snapshot(planner)
    first = write_scene(scene, tmp_path)
    second = write_scene(scene, tmp_path)
    assert first != second
    loaded = json.loads(Path(first).read_text())
    reproduced = json_value(replay_scene(loaded))
    assert reproduced == loaded['result']
    assert loaded['canonical_local_path_points'] == (json_value(result['points']) if not result['blocked'] else [])
    assert planner_snapshot(planner) == after  # Dump/replay never touches live state.
    diagnostic = result['corridor_diagnostics']
    assert diagnostic is not None
    assert diagnostic['safe'] == result['drivable_safe']
    if phase == 'BLOCKED':
        assert diagnostic['failed_point_count'] > 0
        assert result['adjacent_lane_points'] == []  # Not an approved lane.
        assert loaded['result']['corridor_evaluation']['adjacent_lane_points']


def test_frozen_s4_input_remains_safe_and_roundtrips_without_external_map(tmp_path):
    fixture = json.loads((Path(__file__).parent/'fixtures/s4_adjacent_lane_scene.json').read_text())
    for name in ('node', 'link'):
        (tmp_path/f'{name}_set.json').write_text(json.dumps(fixture[name+'s']))
    planner = AdjacentLanePlanner(LaneNetwork(DijkstraPlanner(tmp_path)))
    i = fixture['inputs']
    inputs = dict(reference_points=i['reference'], obstacles=i['obstacles'],
                  current_position=i['position'], current_heading=i['yaw'], obstacle_groups=None,
                  road_progress_m=i['road_progress_m'], current_link_id=i['current_link_id'],
                  route_link_ids=i['route_link_ids'], speed_kmh=20.)
    before = planner_snapshot(planner)
    result = planner.plan(**inputs)
    scene = capture_scene(planner, before, inputs, result, 123, 'map')
    loaded = json.loads(Path(write_scene(scene, tmp_path/'dump')).read_text())
    assert result['lane_change_phase'] == 'ENTRY'
    assert result['minimum_clearance_m'] == pytest.approx(2.979798008613521)
    assert result['maximum_curvature'] == pytest.approx(.07346666908408041)
    assert result['evaluated_candidate_offset'] == pytest.approx(-3.4746708960809882)
    assert result['corridor_diagnostics']['failed_index'] == -1
    assert json_value(replay_scene(loaded)) == loaded['result']


def test_no_adjacent_and_missing_scene_are_not_misreported_as_corridor_failures(tmp_path):
    result, scene = _capture(AdjacentLanePlanner(network(())))
    assert result['blocked_reason'] == 'NO_ADJACENT_LANE'
    assert result['corridor_diagnostics'] is None
    assert json_value(replay_scene(scene)) == scene['result']
    with pytest.raises(ValueError, match='No current'):
        write_scene(None, tmp_path)
