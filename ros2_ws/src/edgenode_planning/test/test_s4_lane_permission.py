"""Unknown crossing metadata never creates an S4 maneuver or a fallback."""
import pytest

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from avoidance_world import world


def network(*, strict=True):
    lanes, area, point, obstacle = world()
    for link in lanes.links.values():
        link['link_type'] = '6'  # Explicit ordinary-lane metadata, synthetic only.
    lanes.require_explicit_permission = strict
    return lanes, area, point, obstacle


def allow(links, original='O0', adjacent='A0'):
    links[original]['can_move_left_lane'] = True
    links[adjacent]['can_move_right_lane'] = True


def test_missing_permission_keeps_legacy_neighbor_but_blocks_s4():
    lanes, _, _, _ = network(strict=False)
    assert lanes.neighbor('O0', 'left') == 'A0'
    lanes.require_explicit_permission = True
    assert lanes.neighbor('O0', 'left') is None
    assert lanes.permission_reason('O0', 'left') == 'UNKNOWN_LANE_CHANGE_PERMISSION'


@pytest.mark.parametrize('value', [None, 1, 0, 'true', 'false', [], {}])
@pytest.mark.parametrize('which', ['outward', 'return'])
def test_only_literal_true_is_positive_permission(value, which):
    lanes, _, _, _ = network()
    allow(lanes.links)
    record, field = (('O0', 'can_move_left_lane') if which == 'outward'
                     else ('A0', 'can_move_right_lane'))
    lanes.links[record][field] = value
    assert lanes.neighbor('O0', 'left') is None
    assert lanes.permission_reason('O0', 'left') == 'UNKNOWN_LANE_CHANGE_PERMISSION'


def test_explicit_denial_wins_over_unknown_return():
    lanes, _, _, _ = network()
    lanes.links['O0']['can_move_left_lane'] = False
    assert lanes.permission_reason('O0', 'left') == 'LANE_CHANGE_FORBIDDEN'


def test_explicit_true_still_requires_reciprocal_topology_and_direction():
    lanes, _, _, _ = network(); allow(lanes.links)
    assert lanes.neighbor('O0', 'left') == 'A0'
    lanes.links['A0']['right_lane_change_dst_link_idx'] = 'O1'
    assert lanes.neighbor('O0', 'left') is None


@pytest.mark.parametrize('kind', ['1', '4', '5', 'unknown', None])
def test_explicit_true_does_not_authorize_intersection_or_unverified_lane_type(kind):
    lanes, _, _, _ = network(); allow(lanes.links)
    lanes.links['O0']['link_type'] = kind
    assert lanes.neighbor('O0', 'left') is None
    assert lanes.permission_reason('O0', 'left') == 'UNVERIFIED_LANE_TYPE'


def test_each_successor_requires_its_own_permission_without_extrapolation():
    lanes, _, _, _ = network(); allow(lanes.links)
    pair = lanes.pair('O0', 'left', ['O0', 'O1'])
    assert pair.ids == [('O0', 'A0')]
    allow(lanes.links, 'O1', 'A1')
    assert lanes.pair('O0', 'left', ['O0', 'O1']).ids == [('O0', 'A0'), ('O1', 'A1')]


def test_synthetic_verified_road_does_not_override_unknown_crossing_permission():
    lanes, area, point, obstacle = network()
    planner = StaticObstaclePlanner(lanes, drivable_area=area)
    reference = [point(i, 0.) for i in range(81)]
    result = planner.plan(reference, [obstacle()], (0., 0., 0.), speed_kmh=3.,
                          current_link_id='O0', route_link_ids=['O0', 'O1'])
    assert result['blocked'] and not result['points']
    assert result['reason'] == 'NO_VALID_ADJACENT_LANE'
    assert planner.plan(reference, [], (0., 0., 0.))['blocked']


def test_positive_permission_never_substitutes_for_real_road_boundary():
    lanes, _, point, obstacle = network()
    allow(lanes.links); allow(lanes.links, 'O1', 'A1')
    planner = StaticObstaclePlanner(lanes)
    result = planner.plan([point(i, 0.) for i in range(81)], [obstacle()],
                          (0., 0., 0.), speed_kmh=3., current_link_id='O0', route_link_ids=['O0', 'O1'])
    assert result['blocked'] and not result['points']
    assert result['reason'] == 'ROAD_BOUNDARY_UNAVAILABLE'


def test_obstacle_free_baseline_geometry_is_unchanged_in_both_policies():
    for strict in (False, True):
        lanes, _, point, _ = network(strict=strict)
        planner = StaticObstaclePlanner(lanes)
        reference = [point(i, 0.) for i in range(21)]
        result = planner.plan(reference, [], (0., 0., 0.))
        assert result['points'] is reference and result['state'] == 'NORMAL'


def test_actual_kcity_has_no_explicit_crossing_authority():
    from pathlib import Path
    directory = Path(__file__).resolve().parents[4] / 'local_data/kcity_mgeo'
    planner = DijkstraPlanner(directory)
    baseline = LaneNetwork(planner)
    strict = LaneNetwork(planner, require_explicit_permission=True)
    assert len(planner.links_by_id) == 352
    assert any(baseline.neighbor(key, side) for key in baseline.valid for side in ('left', 'right'))
    assert not any(strict.neighbor(key, side) for key in strict.valid for side in ('left', 'right'))


@pytest.mark.parametrize('strict', [False, True])
def test_planning_mode_wires_permission_policy_at_startup(route_editor_map, strict):
    import rclpy
    from unittest.mock import patch
    from edgenode_planning.planning_node import PlanningNode
    args = ['--ros-args', '-r', '__ns:=/stage3_constructor',
            '-p', 'mgeo_dir:=' + str(route_editor_map),
            '-p', 's4_sensor_validation_enabled:=' + str(strict).lower()]
    rclpy.init(args=args)
    node = None
    try:
        with patch.object(PlanningNode, 'create_timer'):
            node = PlanningNode()
        assert node.avoidance_planner.network.require_explicit_permission is strict
        assert not node.get_parameter('enable_drive').value
        assert node.get_parameter('avoidance_boundary_approved_sha256').value == ''
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
