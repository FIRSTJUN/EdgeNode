"""Recorded K-City entry rejection: validate the original lane before creeping."""
import json
from pathlib import Path

import pytest
import yaml

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.local_path_planner import LocalPathPlanner
from edgenode_planning.route_resume import mission_suffix
from edgenode_planning.static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner


def scene(enabled=True, offset=0.):
    root = Path(__file__).resolve().parents[4]
    data = json.loads((Path(__file__).parent/'fixtures/kcity_entry_wait_scene.json').read_text())
    data['pose'][0] += offset*.014
    data['pose'][1] += offset
    values = dict(data['config']); values['transition_lengths_m'] = tuple(values['transition_lengths_m'])
    if not enabled: values['entry_geometry_wait_distance_m'] = 0.
    planner = DijkstraPlanner(root/'local_data/kcity_mgeo')
    waypoints = yaml.safe_load((root/'local_data/kcity_routes/kcity_demo_route.yaml').read_text())['waypoint_node_ids']
    route = mission_suffix(planner, 'A219BS010380', waypoints, 'A219BS010404')
    avoidance = StaticObstaclePlanner(LaneNetwork(planner, require_explicit_permission=False), config=AvoidanceConfig(**values))
    def tick(step, advance=0., speed=.03, valid=True):
        x, y, yaw = data['pose']; pose = (x + advance*.014, y + advance, yaw)
        reference = LocalPathPlanner().extract(route['points'], *pose[:2], horizon_m=80.)['points']
        result = avoidance.plan(reference, [data['solid']], pose, speed_kmh=speed,
            current_link_id='A219BS010469', route_link_ids=route['link_ids'],
            route_link_index=route['link_ids'].index('A219BS010469'),
            global_reference=route['points'], now_sec=float(step), observation_valid=valid)
        return result, reference
    tick(0, speed=15.)
    assert tick(1, speed=15.)[0]['state'] == 'DETECT'
    return avoidance, tick, data


@pytest.mark.parametrize('offset', [0., -.5])
def test_real_entry_is_not_committed_until_geometry_becomes_feasible_ahead(offset):
    planner, tick, data = scene(offset=offset)
    result, reference = tick(2)
    assert result['reason'] == 'WAIT_ENTRY_GEOMETRY' and not result['blocked']
    assert result['speed_cap_kmh'] == 3. and planner.entry_commits == 0
    assert not planner.needs_validation_stop(reference, [data['solid']])
    assert tick(3, advance=.5, speed=3.)[0]['state'] == 'DETECT'
    result, _ = tick(4, advance=2.1, speed=3.)
    assert result['state'] == 'AVOID_ENTRY' and planner.entry_commits == 1
    assert not planner.selected.rejection


def test_approach_still_stops_for_a_new_intrusion_and_invalid_sensor():
    planner, tick, data = scene()
    _, reference = tick(2)
    x, y, _ = data['pose']
    intrusion = [(x+dx, y+dy) for dx, dy in ((-1., 6.), (1., 6.), (1., 8.), (-1., 8.))]
    assert planner.needs_validation_stop(reference, [data['solid'], intrusion])
    result, _ = tick(3, valid=False)
    assert result['blocked'] and planner.entry_commits == 0 and not result['points']


def test_default_disabled_keeps_the_original_entry_rejection():
    planner, tick, _ = scene(enabled=False)
    result, _ = tick(2)
    assert result['blocked'] and result['reason'] == 'NO_SAFE_ENTRY'
    assert planner.entry_commits == 0
