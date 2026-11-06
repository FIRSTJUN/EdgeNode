"""Reuse immutable failed-run data without replaying a removed planner."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from edgenode_planning.lattice_path import curvatures
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner,AvoidanceConfig
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork

FIXTURES=Path(__file__).parent/'fixtures'
WIDE=json.loads((FIXTURES/'s4_wide_failed_paths.json').read_text())
BLOCKED=json.loads((FIXTURES/'s4_20kph_blocked_scenes.json').read_text())


@pytest.mark.parametrize('case',WIDE['cases'])
def test_recorded_clearance_approved_but_steering_unsafe_path_stays_infeasible(case):
    maximum=max(map(abs,curvatures(case['path'])))
    assert maximum==pytest.approx(case['expected_curvature'],abs=2e-7)
    assert maximum>AvoidanceConfig().curvature_limit


@pytest.mark.parametrize('case',BLOCKED['cases'])
def test_recorded_unsafe_request_cannot_be_approved_without_lane_or_boundary_evidence(case):
    planner=StaticObstaclePlanner(SimpleNamespace(pair=lambda *args:None))
    result=planner.plan(case['reference'],case['groups'],(*case['position'],case['yaw']),speed_kmh=20.)
    assert result['blocked'] and not result['points']


def test_collinear_reversal_is_not_zero_curvature_feasibility():
    with pytest.raises(ValueError,match='reversal'):
        curvatures([(0.,0.,0.),(2.,0.,0.),(1.,0.,0.)])


def test_real_mgeo_and_frozen_live_initial_input_produce_only_rejected_diagnostic_paths():
    scene=json.loads((FIXTURES/'s4_adjacent_lane_scene.json').read_text())['inputs']
    root=Path(__file__).resolve().parents[4]
    planner=StaticObstaclePlanner(LaneNetwork(DijkstraPlanner(root/'local_data/kcity_mgeo')))
    result=planner.plan(scene['reference'],[scene['obstacles']],(*scene['position'],scene['yaw']),
        speed_kmh=3.,current_link_id=scene['current_link_id'],route_link_ids=scene['route_link_ids'])
    assert result['blocked'] and not result['points']
    assert result['candidates'] and any(c.points for c in result['candidates'])
    assert all(c.rejection for c in result['candidates'])
