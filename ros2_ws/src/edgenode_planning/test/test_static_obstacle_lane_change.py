"""Phase 4 gate: observations drive behavior; the primitive owns every shift."""
from types import SimpleNamespace
from unittest.mock import patch
import math

import pytest

from edgenode_planning.lane_change_path import generate_shift_path
from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.static_obstacle_lane_change import StaticObstacleLaneChangeBehavior


def network(side='left', length=90., separation=3.5):
    links = {}
    for name, y in [('O', 0.), ('A', separation if side == 'left' else -separation)]:
        links[name] = dict(idx=name, from_node_idx=name+'0', to_node_idx=name+'1',
            max_speed=30., points=[[i*.5, y, 0.] for i in range(round(length*2)+1)],
            left_lane_change_dst_link_idx='A' if name == 'O' and side == 'left' else
                'O' if name == 'A' and side == 'right' else None,
            right_lane_change_dst_link_idx='A' if name == 'O' and side == 'right' else
                'O' if name == 'A' and side == 'left' else None)
    return LaneNetwork(SimpleNamespace(links_by_id=links,
        graph={'x': [{'link_id': name} for name in links]}))


def footprint(front=30., rear=35., low=-.5, high=.5):
    mid = (front+rear)/2
    return [(front, low), (mid, low), (rear, low), (rear, (low+high)/2),
            (rear, high), (mid, high), (front, high), (front, (low+high)/2)]


def plan(behavior, x=0., y=0., yaw=0., obstacles=None, **options):
    obstacles = footprint() if obstacles is None else obstacles
    return behavior.plan([[x+i*.5, 0., 0.] for i in range(41)], obstacles, (x, y),
        current_heading=yaw, obstacle_groups=[obstacles] if obstacles else [],
        current_link_id='O', route_link_ids=['O'], speed_kmh=options.pop('speed_kmh', 3.),
        **options)


@pytest.mark.parametrize('side,offset', [('left', 3.5), ('right', -3.5)])
def test_full_static_obstacle_lifecycle_and_exactly_two_shift_generations(side, offset):
    behavior = StaticObstacleLaneChangeBehavior(network(side))
    phases = [plan(behavior, obstacles=[])['lane_change_phase']]
    with patch('edgenode_planning.lane_change_planner.generate_shift_path',
               wraps=generate_shift_path) as generate:
        entry = plan(behavior)
        phases.append(entry['lane_change_phase'])
        assert behavior.primitive.result.request.reason == 'STATIC_OBSTACLE'
        assert behavior.primitive.result.request.return_required
        fixed = behavior.primitive.entry_path
        for _ in range(20):
            assert plan(behavior)['lane_change_phase'] == 'ENTRY'
            assert behavior.primitive.entry_path is fixed
        for x in (18.5, 18.75, 19.):
            hold = plan(behavior, x, offset)
        assert hold['lane_change_phase'] == 'TARGET_LANE_HOLD'
        phases.append(hold['lane_change_phase'])
        assert plan(behavior, 35.4, offset, obstacles=[])['lane_change_phase'] == 'TARGET_LANE_HOLD'
        assert behavior.primitive.recovery_path is None
        recovery = plan(behavior, 35.6, offset, obstacles=[])
        assert recovery['lane_change_phase'] == 'RECOVERING'
        assert behavior.return_trigger_progress_m > 35.+.5
        phases.append(recovery['lane_change_phase'])
        fixed_return = behavior.primitive.recovery_path
        for _ in range(20):
            assert plan(behavior, 36., offset, obstacles=[])['lane_change_phase'] == 'RECOVERING'
            assert behavior.primitive.recovery_path is fixed_return
        for x in (54.1, 54.35, 54.6):
            complete = plan(behavior, x, 0., obstacles=[])
        assert complete['lane_change_phase'] == 'COMPLETE'
        phases.append(complete['lane_change_phase'])
        assert complete['points'][0][:2] == (0., 0.)
        assert not complete['obstacle_avoid_requested']
        assert complete['minimum_clearance_m'] >= behavior.required_clearance_m
        assert generate.call_count == 2
        assert plan(behavior, 55., 0., obstacles=[])['lane_change_phase'] == 'NORMAL'
    assert phases == ['NORMAL', 'ENTRY', 'TARGET_LANE_HOLD', 'RECOVERING', 'COMPLETE']


@pytest.mark.parametrize('damage', ['absent', 'reverse', 'nan', 'no_reciprocal'])
def test_no_valid_adjacent_lane_stops(damage):
    lanes = network()
    if damage == 'absent': lanes.valid.remove('A')
    elif damage == 'reverse': lanes.links['A']['points'].reverse()
    elif damage == 'nan': lanes.links['A']['points'][10][0] = math.nan
    else: lanes.links['A']['right_lane_change_dst_link_idx'] = None
    result = plan(StaticObstacleLaneChangeBehavior(lanes))
    assert result['blocked'] and result['points'] == []


def test_insufficient_entry_distance_is_a_latched_stop():
    behavior = StaticObstacleLaneChangeBehavior(network())
    result = plan(behavior, obstacles=footprint(15., 20.))
    assert result['blocked_reason'] == 'ENTRY_DEADLINE'
    assert plan(behavior, obstacles=[])['blocked']


def test_insufficient_recovery_distance_stops():
    behavior = StaticObstacleLaneChangeBehavior(network(length=50.))
    assert not plan(behavior)['blocked']
    for x in (18.5, 18.75, 19.): plan(behavior, x, 3.5)
    result = plan(behavior, 35.6, 3.5, obstacles=[])
    assert result['blocked_reason'] == 'INSUFFICIENT_RECOVERY_DISTANCE'


@pytest.mark.parametrize('obstacles', [footprint(low=-.5, high=3.),
    footprint()+footprint(low=3., high=4.)])
def test_clearance_or_unsafe_target_lane_stops(obstacles):
    result = plan(StaticObstacleLaneChangeBehavior(network()), obstacles=obstacles)
    assert result['blocked']
    assert 'CLEARANCE' in result['blocked_reason']


def test_obstacle_update_extends_remembered_rear_and_duplicates_do_not_advance_return():
    behavior = StaticObstacleLaneChangeBehavior(network())
    plan(behavior)
    for x in (18.5, 18.75, 19.): plan(behavior, x, 3.5)
    updated = footprint(rear=39.)
    for _ in range(10):
        result = plan(behavior, 35.6, 3.5, obstacles=updated+updated, road_progress_m=10000.)
        assert result['lane_change_phase'] == 'TARGET_LANE_HOLD'
    assert result['obstacle_rear_progress_m'] == 39.
    assert plan(behavior, 39.4, 3.5, obstacles=[])['lane_change_phase'] == 'TARGET_LANE_HOLD'
    assert plan(behavior, 39.6, 3.5, obstacles=[])['lane_change_phase'] == 'RECOVERING'


def test_updated_obstacle_in_target_lane_stops_without_regenerating_entry():
    behavior = StaticObstacleLaneChangeBehavior(network())
    plan(behavior)
    fixed = behavior.primitive.entry_path
    result = plan(behavior, 1., .01, obstacles=footprint()+footprint(low=3., high=4.))
    assert result['blocked']
    assert behavior.primitive.entry_path is fixed


def test_actual_outside_corridor_and_reverse_heading_fail_closed():
    for y, yaw, reason in [(-.001, 0., 'DRIVABLE_CORRIDOR'), (0., math.pi, 'DIRECTION_CONTINUITY')]:
        behavior = StaticObstacleLaneChangeBehavior(network())
        result = plan(behavior, y=y, yaw=yaw)
        assert result['blocked'] and reason in result['blocked_reason']


def test_planning_node_new_adapter_publishes_path_and_fail_closed_stop():
    # Reuse the existing node harness without changing or collecting its tests.
    import test_planning_node as fixture
    from edgenode_planning.planning_node import PlanningNode
    for valid in (True, False):
        case = fixture.PlanningNodeTest(); case.setUp()
        lanes = network()
        if not valid: lanes.valid.remove('A')
        case.node.lattice_planner = StaticObstacleLaneChangeBehavior(lanes)
        case.node.latest_pose = (0., 0., 0.)
        case.node.latest_obstacles = footprint()
        case.node.latest_obstacle_groups = [footprint()]
        case.node.map_matcher.match.return_value = dict(link_id='O', distance=0., heading_diff=0.)
        case.node.dijkstra_planner.plan.return_value = dict(points=lanes.links['O']['points'], link_ids=['O'])
        PlanningNode.plan(case.node)
        assert case.output('lattice_status') == ('AVOIDING' if valid else 'BLOCKED')
        path = case.node.local_path_pub.publish.call_args.args[0]
        assert bool(path.poses) == valid
        if not valid:
            assert case.output('target_speed') == 0.
            assert case.output('fsm_state') == 'EMERGENCY_STOP'


def test_production_node_constructor_selects_explicit_primitive_mode(route_editor_map):
    import rclpy
    from edgenode_planning.planning_node import PlanningNode
    rclpy.init(args=['--ros-args', '-r', '__ns:=/primitive_mode_constructor',
        '-p', f'mgeo_dir:={route_editor_map}', '-p', 'obstacle_avoidance_mode:=lane_change_primitive'])
    node = None
    try:
        with patch.object(PlanningNode, 'create_timer'):
            node = PlanningNode()
        assert isinstance(node.lattice_planner, StaticObstacleLaneChangeBehavior)
        assert node.lattice_planner.transition_length_m == 20.
        assert node.lattice_planner.required_clearance_m == 1.2
        assert node.get_parameter('enable_drive').value is False
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
