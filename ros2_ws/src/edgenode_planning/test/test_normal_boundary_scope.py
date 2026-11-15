"""A measured central-divider contact is diagnostic until S4 is needed."""
from dataclasses import replace
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.hdmap_boundaries import HDMapBoundaries
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner
from avoidance_world import world


ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture(scope='module')
def scene():
    return json.loads((Path(__file__).parent/'fixtures/s4_normal_boundary_start.json').read_text())


@pytest.fixture(scope='module')
def network():
    result = LaneNetwork(DijkstraPlanner(ROOT/'local_data/kcity_mgeo'))
    result.boundaries = HDMapBoundaries(ROOT/'local_data/kcity_hdmap', ROOT/'local_data/kcity_mgeo')
    return result


@pytest.mark.parametrize('roadside', [False, True])
def test_recorded_normal_contact_preserves_base_path_and_has_no_s4_cap(network, scene, roadside):
    planner = StaticObstaclePlanner(network)
    path = scene['reference']
    result = planner.plan(path, scene['groups'] if roadside else [], scene['pose'],
                          current_link_id=scene['current_link_id'])
    assert result['points'] is path
    assert result['state'] == 'NORMAL' and not result['blocked'] and not result['active']
    assert result['reason'] == '' and result['speed_cap_kmh'] is None
    assert result['path_obstacle_count'] == 0
    assert result['boundary_check_mode'] == 'diagnostic'
    assert result['boundary_contact_reason'] == 'LANE_BOUNDARY_COLLISION:B219BS011367'
    assert result['entry_commits'] == result['return_commits'] == 0


def test_markings_cannot_hide_a_path_obstacle_or_its_validation_stop():
    lanes, point, obstacle = world()
    lanes.boundaries = Mock()
    lanes.boundaries.swept_reason.return_value = 'LANE_BOUNDARY_COLLISION:synthetic'
    planner = StaticObstaclePlanner(lanes)
    path = [point(i, 0.) for i in range(81)]
    assert planner._normal_hazards(path, [obstacle()])
    assert planner.needs_validation_stop(path, [obstacle()])


def test_first_contact_at_8_86_seconds_is_diagnostic_in_normal(network, scene):
    planner = StaticObstaclePlanner(network)
    path = scene['first_contact_reference']
    result = planner.plan(path, [], scene['first_contact_pose'])
    assert result['state'] == 'NORMAL' and result['points'] is path and not result['active']
    assert result['boundary_contact_reason'] == 'LANE_BOUNDARY_COLLISION:B219BS011367'


def test_only_confirmed_hazard_activates_the_boundary_stop():
    lanes, point, obstacle = world()
    lanes.boundaries = Mock()
    lanes.boundaries.swept_reason.return_value = 'LANE_BOUNDARY_COLLISION:synthetic'
    planner = StaticObstaclePlanner(lanes, config=replace(AvoidanceConfig(), normal_obstacle_confirmations=2))
    path = [point(i, 0.) for i in range(81)]
    first = planner.plan(path, [obstacle()], (0., 0., 0.), now_sec=0., observation_stamp_ns=1)
    assert first['state'] == 'NORMAL' and first['points'] is path
    assert first['boundary_check_mode'] == 'diagnostic'
    confirmed = planner.plan(path, [obstacle()], (0., 0., 0.), now_sec=.1, observation_stamp_ns=2)
    assert confirmed['blocked'] and confirmed['boundary_check_mode'] == 'avoidance'
    assert confirmed['reason'] == 'LANE_BOUNDARY_COLLISION:synthetic'


@pytest.mark.parametrize('state', ['DETECT', 'AVOID_ENTRY', 'AVOID_HOLD', 'RETURN'])
def test_active_s4_measured_body_keeps_real_central_divider_stop(network, scene, state):
    planner = StaticObstaclePlanner(network)
    planner.state = state
    result = planner.plan(scene['reference'], [], scene['pose'])
    assert result['blocked'] and not result['points']
    assert result['reason'] == 'LANE_BOUNDARY_COLLISION:B219BS011367'


def test_s4_candidate_body_keeps_real_central_divider_stop(network, scene):
    planner = StaticObstaclePlanner(network)
    x, y, yaw = scene['pose']
    import math
    path = [(x+i*.1*math.cos(yaw), y+i*.1*math.sin(yaw), 0.) for i in range(10)]
    assert planner._path_safety(path, (), 3.) == 'LANE_BOUNDARY_COLLISION:B219BS011367'


def test_normal_actual_solid_collision_still_stops():
    lanes, point, obstacle = world()
    planner = StaticObstaclePlanner(lanes)
    path = [point(i, 0.) for i in range(81)]
    assert planner.plan(path, [obstacle(1., 2.)], (0., 0., 0.))['reason'] == 'BODY_COLLISION'
