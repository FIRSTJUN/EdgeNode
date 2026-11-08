"""New production integration; isolated ROS topics and mocked outputs, no drive."""
import json
import math
import uuid
from unittest.mock import Mock, patch

import pytest
import rclpy
from geometry_msgs.msg import Pose,PoseArray
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import String

from edgenode_planning.planning_node import PlanningNode
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from avoidance_world import world


PLANNING_TOPICS=('target_error','target_speed','state','current_link','map_match_status',
    'map_match_distance','map_match_heading_diff','global_path','local_path','local_path_index',
    'local_path_progress_m','recommended_speed','speed_mode','speed_curvature','speed_max_curvature',
    'drive_status','fsm_state','runtime_cruise_speed','runtime_curve_speed','runtime_sharp_curve_speed',
    'runtime_max_drive_speed','obstacles','avoidance_state','avoidance_debug','lattice_candidate_markers',
    'original_lane_center','left_lane_center','right_lane_center','avoidance_selected_path',
    'avoidance_return_path','reset_avoidance')


@pytest.fixture
def node(route_editor_map, request):
    prefix='/offline_avoidance_'+uuid.uuid4().hex[:8]
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}','-p','goal_node_id:=D']
    if getattr(request, 'param', False):
        args += ['-p', 's4_sensor_validation_enabled:=true',
                 '-r', '/perception/lidar_status:=' + prefix + '/lidar_status',
                 '-r', '/planning/s4_sensor_state:=' + prefix + '/s4_sensor_state']
    for topic in PLANNING_TOPICS:args+=['-r',f'/planning/{topic}:={prefix}/{topic}']
    args+=['-r',f'/localization/odometry:={prefix}/odometry']
    rclpy.init(args=args)
    value=None
    try:
        with patch.object(PlanningNode,'create_timer'):
            value=PlanningNode()
        for name in list(value.__dict__):
            if name.endswith('_pub'):setattr(value,name,Mock())
        value._avoidance_path_publishers={name:Mock() for name in value._avoidance_path_publishers}
        lanes,area,point,obstacle=world()
        value.avoidance_planner=StaticObstaclePlanner(lanes)
        value.dijkstra_planner.plan=Mock(return_value=dict(points=lanes.links['O0']['points']+
            lanes.links['O1']['points'][1:],link_ids=['O0','O1']))
        value.map_matcher.match=Mock(return_value=dict(link_id='O0',distance=0.,heading_diff=0.))
        if value.distance_safety is not None:
            # Synthetic sensor-only fixture, not K-City visibility approval.
            from edgenode_planning.s4_distance_safety import ObservationEvidence
            value.distance_safety.evidence = ObservationEvidence(21.19, 50., 1., ('O0', 'O1'),
                                                                  'synthetic unit-test world')
        value.latest_pose=(0.,0.,0.);value.latest_speed_kmh=3.
        value.latest_pose_stamp_ns=1
        value.latest_pose_received_ns=value.get_clock().now().nanoseconds
        value.test_area=area;value.test_obstacle=obstacle
        yield value
    finally:
        if value is not None:value.destroy_node()
        rclpy.shutdown()


def output(node,name):
    return getattr(node,name+'_pub').publish.call_args.args[0]


def detect(node,points=None,frame='map',stamp=1.):
    msg=PoseArray();msg.header.frame_id=frame
    msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    for x,y in points if points is not None else node.test_obstacle(21.,25.):
        pose=Pose();pose.position.x=float(x);pose.position.y=float(y);msg.poses.append(pose)
    node.obstacle_callback(msg)


def arm_isolated_node(node):
    assert all(node.get_parameter(name).value==False for name in ('enable_drive',))
    result=node.set_parameters([Parameter('enable_drive',value=True),Parameter('max_drive_speed_kmh',value=20.),
        Parameter('cruise_speed_kmh',value=20.),Parameter('curve_speed_kmh',value=20.),
        Parameter('sharp_curve_speed_kmh',value=20.)])
    assert all(r.successful for r in result)


def test_no_obstacle_keeps_original_global_and_local_path_without_boundary_data(node):
    node.plan()
    assert output(node,'avoidance_state').data=='NORMAL'
    assert output(node,'local_path').poses
    assert output(node,'target_speed').data==0.
    assert output(node,'global_path').poses


def test_unknown_boundary_produces_candidates_diagnostics_empty_path_and_emergency(node):
    arm_isolated_node(node);detect(node);node.plan()
    debug=json.loads(output(node,'avoidance_debug').data)
    assert output(node,'avoidance_state').data=='BLOCKED'
    assert debug['reason']=='ROAD_BOUNDARY_UNAVAILABLE'
    assert debug['candidates'] and all(c['rejection'] for c in debug['candidates'])
    assert not output(node,'local_path').poses
    assert output(node,'target_speed').data==0.
    assert output(node,'fsm_state').data=='EMERGENCY_STOP'
    markers=output(node,'candidate').markers
    assert any(m.points for m in markers) and any('ROAD_BOUNDARY_UNAVAILABLE' in m.text for m in markers)
    global_before=output(node,'global_path')
    detect(node,[],stamp=2.);node.plan()
    assert output(node,'target_speed').data==0.
    assert [p.pose.position for p in output(node,'global_path').poses]==[p.pose.position for p in global_before.poses]
    node.dijkstra_planner.plan.assert_called_once()


def test_verified_synthetic_road_selects_only_local_avoidance_and_caps_high_speed_request(node):
    node.avoidance_planner.drivable_area=node.test_area
    node.set_parameters([Parameter('local_path_length_m',value=50.)])
    detect(node,node.test_obstacle());node.plan()  # Initial geometry validation while disarmed.
    node.latest_pose_received_ns=node.get_clock().now().nanoseconds
    detect(node,node.test_obstacle(),stamp=2.)
    # Test routing/caps independently of CPU contention on the test host.
    # Separate tests below inject slow validation and assert mandatory STOP;
    # closed-loop benchmarks record actual execution time without this mock.
    with patch('edgenode_planning.planning_node.time.perf_counter',return_value=0.):
        arm_isolated_node(node);node.plan()
        assert output(node,'avoidance_state').data=='AVOID_ENTRY'
        assert output(node,'target_speed').data==3.
        assert output(node,'fsm_state').data=='OBSTACLE_AVOID'
        assert all(p.pose.position.y==0 for p in output(node,'global_path').poses)
        assert max(p.pose.position.y for p in output(node,'local_path').poses)>3.
        fixed=node.avoidance_planner.selected
        node.latest_pose=(.01,0.,0.);node.map_matcher.match.return_value['link_id']='A0';node.plan()
    assert node.avoidance_planner.selected is fixed
    node.dijkstra_planner.plan.assert_called_once()


@pytest.mark.parametrize('failure',[None,RuntimeError('test planner fault')])
def test_unavailable_or_faulted_planner_can_never_fall_back_to_normal_path(node,failure):
    arm_isolated_node(node)
    if isinstance(failure,Exception):node.avoidance_planner.plan=Mock(side_effect=failure)
    else:node.avoidance_planner.plan=Mock(return_value=failure)
    node.plan()
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.
    assert output(node,'fsm_state').data=='EMERGENCY_STOP'


@pytest.mark.parametrize('points',[[],[(0.,0.,0.)],[(0.,0.,0.),(math.nan,0.,0.)],
    [(0.,0.,0.),(0.,0.,0.)]])
def test_invalid_result_or_control_window_latches_stop_before_any_motion_publish(node,points):
    arm_isolated_node(node)
    node.avoidance_planner.plan=Mock(return_value=dict(state='AVOID_ENTRY',points=points,
        blocked=False,active=True))
    node.plan()
    assert node.avoidance_planner.state=='BLOCKED'
    assert output(node,'avoidance_state').data=='BLOCKED'
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.
    assert output(node,'fsm_state').data=='EMERGENCY_STOP'


@pytest.mark.parametrize('damage',['frame','incomplete','nan','reordered'])
def test_invalid_obstacle_observation_is_not_silently_converted_to_free_space(node,damage):
    detect(node)
    previous=node.latest_obstacle_groups
    if damage=='frame':detect(node,frame='lidar',stamp=2.)
    elif damage=='incomplete':detect(node,[(1.,0.)],stamp=2.)
    elif damage=='nan':detect(node,[(math.nan,0.)]*8,stamp=2.)
    else:detect(node,stamp=0.)
    assert not node.latest_obstacle_valid and node.latest_obstacle_groups==previous
    node.plan()
    assert output(node,'target_speed').data==0.


def test_reset_requires_disabled_drive_and_cannot_erase_hazard_while_armed(node):
    detect(node);node.plan();assert node.avoidance_planner.state=='BLOCKED'
    arm_isolated_node(node)
    response=node.reset_avoidance(Trigger.Request(),Trigger.Response())
    assert not response.success and node.avoidance_planner.state=='BLOCKED'
    node.set_parameters([Parameter('enable_drive',value=False)])
    response=node.reset_avoidance(Trigger.Request(),Trigger.Response())
    assert response.success and node.avoidance_planner.state=='NORMAL'
    node.plan();assert node.avoidance_planner.state=='BLOCKED'


@pytest.mark.parametrize('known_road',[False,True])
def test_route_edit_cannot_erase_committed_path_obstacle_memory_or_latched_stop(node,known_road):
    if known_road:node.avoidance_planner.drivable_area=node.test_area
    node.set_parameters([Parameter('local_path_length_m',value=50.)])
    detect(node,node.test_obstacle());node.plan()
    assert node.avoidance_planner.state==('AVOID_ENTRY' if known_road else 'BLOCKED')
    remembered=node.avoidance_planner.remembered
    detect(node,[],stamp=2.)
    node.set_parameters([Parameter('goal_node_id',value='E')]);node.plan()
    assert node.avoidance_planner.state=='BLOCKED'
    assert node.avoidance_planner.remembered==remembered
    assert output(node,'avoidance_state').data=='BLOCKED'
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.


def test_normal_cruise_is_not_permanently_reduced_and_match_loss_breaks_confirmation(node):
    arm_isolated_node(node);node.plan()
    assert output(node,'target_speed').data==20.
    node.avoidance_planner.confirmations=2
    node.map_matcher.match.return_value=None;node.plan()
    assert output(node,'target_speed').data==0. and node.avoidance_planner.confirmations==0


def test_safety_parameters_are_read_only_and_cannot_be_loosened_by_live_updates(node):
    results=node.set_parameters([Parameter('avoidance_road_margin_m',value=-1.)])
    assert not results[0].successful


def test_first_malformed_observation_stops_without_prior_detection(node):
    detect(node,[(math.nan,0.)]*8)
    node.plan()
    assert output(node,'avoidance_state').data=='BLOCKED'
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.


def test_degenerate_footprint_remains_latched_after_fresh_empty_observation(node):
    arm_isolated_node(node);detect(node,[(15.,0.)]*8);node.plan()
    assert node.avoidance_planner.state=='BLOCKED'
    detect(node,[],stamp=2.);node.plan()
    assert output(node,'target_speed').data==0. and node.avoidance_planner.state=='BLOCKED'


def test_candidate_search_stops_before_computation_and_requires_distinct_fresh_odometry(node):
    node.avoidance_planner.drivable_area=node.test_area
    node.latest_speed_kmh=0.
    node.set_parameters([Parameter('local_path_length_m',value=50.)])
    arm_isolated_node(node)
    # Isolate freshness semantics from host scheduling; real execution timing
    # is independently recorded by the offline benchmarks, never asserted here.
    with patch('edgenode_planning.planning_node.time.perf_counter',return_value=0.):
        detect(node,node.test_obstacle());node.plan()
        assert output(node,'avoidance_state').data=='VALIDATING'
        assert node.target_speed_pub.publish.call_args_list[0].args[0].data==0.
        for _ in range(3):
            node.latest_pose_received_ns=node.get_clock().now().nanoseconds
            detect(node,node.test_obstacle(),stamp=2.);node.plan()
            assert output(node,'target_speed').data==0.
        for i in range(1,4):
            node.latest_pose_stamp_ns+=1
            node.latest_pose_received_ns=node.get_clock().now().nanoseconds
            detect(node,node.test_obstacle(),stamp=2.+i);node.plan()
            if i<3:assert output(node,'target_speed').data==0.
        assert output(node,'avoidance_state').data=='AVOID_ENTRY'
        assert output(node,'target_speed').data==3.


def test_slow_armed_cycle_cannot_publish_a_motion_path_computed_on_stale_pose(node):
    node.avoidance_planner.drivable_area=node.test_area
    node.set_parameters([Parameter('local_path_length_m',value=50.)])
    detect(node,node.test_obstacle());node.plan()
    node.latest_pose_received_ns=node.get_clock().now().nanoseconds
    detect(node,node.test_obstacle(),stamp=2.)
    arm_isolated_node(node)
    with patch('edgenode_planning.planning_node.time.perf_counter',side_effect=[0.,.1]):
        node.plan()
    assert output(node,'avoidance_state').data=='VALIDATING'
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.
    assert json.loads(output(node,'avoidance_debug').data)['validation_elapsed_ms']==100.


def test_reverse_measured_motion_is_not_reinterpreted_as_forward_speed(node):
    msg=Odometry();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=-1.
    node.localization_callback(msg);node.plan()
    assert node.latest_speed_kmh<0
    assert not output(node,'local_path').poses and output(node,'target_speed').data==0.


def test_baseline_without_s4_mode_ignores_sensor_health_and_keeps_normal_speed(node):
    assert node.sensor_guard is None
    node.plan()
    original = [p.pose.position for p in output(node, 'local_path').poses]
    node.sensor_status_callback(String(data='{invalid status'))
    arm_isolated_node(node)  # Motion outputs are mocks; no live Control exists.
    node.plan()
    assert node.avoidance_planner.state == 'NORMAL'
    assert output(node, 'target_speed').data == 20.
    assert [p.pose.position for p in output(node, 'local_path').poses] == original
    node.dijkstra_planner.plan.assert_called_once()
