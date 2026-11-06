"""Visualization-only tests: exact source geometry, role styling and stale cleanup."""
import copy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from edgenode_planning.planning_visualization_node import PlanningVisualizationNode, PATH_STYLES


@pytest.fixture
def view(monkeypatch, route_editor_map):
    values={};publishers={};subscriptions={}
    def declare(self,name,value):values[name]=str(route_editor_map) if name=='mgeo_dir' else value
    def publisher(self,kind,topic,qos):
        pub=Mock();pub.qos=qos;publishers[topic]=pub;return pub
    def subscribe(self,kind,topic,callback,qos):subscriptions[topic]=(callback,qos)
    monkeypatch.setattr(Node,'__init__',lambda *args,**kwargs:None)
    monkeypatch.setattr(Node,'declare_parameter',declare)
    monkeypatch.setattr(Node,'get_parameter',lambda self,name:SimpleNamespace(value=values[name]))
    monkeypatch.setattr(Node,'create_publisher',publisher)
    monkeypatch.setattr(Node,'create_subscription',subscribe)
    monkeypatch.setattr(Node,'create_timer',Mock())
    monkeypatch.setattr(Node,'get_clock',lambda self:SimpleNamespace(now=lambda:Time(seconds=2.)))
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:0.)
    node=PlanningVisualizationNode();node.test_publishers=publishers;node.test_subscriptions=subscriptions
    return node


def path(points,stamp=2.):
    msg=Path();msg.header.frame_id='map';msg.header.stamp=Time(seconds=stamp).to_msg()
    for x,y,z in points:
        p=PoseStamped();p.header=msg.header
        p.pose.position.x,p.pose.position.y,p.pose.position.z=float(x),float(y),float(z)
        msg.poses.append(p)
    return msg


def test_global_and_control_local_mirrors_preserve_every_point_order_and_duplicates(view):
    # Intentional revisits remain visible; no visualization-only route is calculated.
    points=[(0.,0.,1.),(1.,0.,2.),(1.,0.,2.),(0.,0.,1.),(2.,3.,4.)]
    source=path(points);before=copy.deepcopy(source)
    for key in ('global_path','local_path'):
        view.path_callback(key,source)
        marker=view._path_publishers[key].publish.call_args.args[0]
        assert [(p.x,p.y,p.z) for p in marker.points]==points
        assert marker.lifetime.sec==0 and marker.lifetime.nanosec==0
    assert source==before
    assert PATH_STYLES['global_path'][2]!=PATH_STYLES['local_path'][2]


def test_invalid_input_empty_route_and_source_timeout_delete_markers(view,monkeypatch):
    source=path([(0.,0.,0.),(1.,0.,0.)])
    view.path_callback('global_path',source)
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:3.1)
    view.expire()
    assert view._path_publishers['global_path'].publish.call_args.args[0].action==Marker.DELETE
    view.path_callback('local_path',path([]))
    assert view._path_publishers['local_path'].publish.call_args.args[0].action==Marker.DELETE
    source.header.frame_id='odom'
    view.path_callback('global_path',source)
    assert view._path_publishers['global_path'].publish.call_args.args[0].action==Marker.DELETE


def test_route_replacement_clears_old_local_and_rejects_delayed_old_path(view):
    view.path_callback('global_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.5))
    view.path_callback('local_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.5))
    view.path_callback('global_path',path([(0.,0.,0.),(0.,1.,0.)]))
    assert view._path_publishers['local_path'].publish.call_args.args[0].action==Marker.DELETE
    view.path_callback('local_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.5))
    assert view._path_publishers['local_path'].publish.call_args.args[0].action==Marker.DELETE
    view.state_callback(String(data='GLOBAL_PATH_NOT_FOUND'))
    assert all(pub.publish.call_args.args[0].action==Marker.DELETE for pub in view._path_publishers.values())


def test_current_lane_is_existing_mgeo_geometry_and_phase_cleanup_has_stable_ids(view):
    before=copy.deepcopy(view.map.links_by_id)
    view.current_link_callback(String(data='AB'))
    marker=view.current_pub.publish.call_args.args[0]
    assert [[p.x,p.y,p.z] for p in marker.points]==view.map.links_by_id['AB']['points']
    assert marker.ns=='current_lane' and marker.id==0
    view.current_link_callback(String(data=''))
    assert view.current_pub.publish.call_args.args[0].action==Marker.DELETE
    view.phase_callback(String(data='BLOCKED'))
    for key in ('avoidance_selected_path','avoidance_return_path'):
        assert view._path_publishers[key].publish.call_args.args[0].action==Marker.DELETE
    assert view.map.links_by_id==before


def footprint(copies=1):
    source=PoseArray();source.header.frame_id='map';source.header.stamp=Time(seconds=2.).to_msg()
    for x,y in [(1.,0.),(2.,0.),(3.,0.),(3.,1.),(3.,2.),(2.,2.),(1.,2.),(1.,1.)]*copies:
        p=Pose();p.position.x,p.position.y=x,y;source.poses.append(p)
    return source


def test_footprint_groups_are_closed_diagnostics_with_count_shrink_cleanup(view):
    source=footprint(2);before=copy.deepcopy(source)
    view.obstacles_callback(source)
    markers=view.obstacle_pub.publish.call_args.args[0].markers
    assert [m.id for m in markers]==[0,1]
    assert all(m.action==Marker.ADD and m.lifetime.sec==0 and m.lifetime.nanosec==0 for m in markers)
    assert all(len(m.points)==9 and m.points[0]==m.points[-1] for m in markers)
    assert source==before
    source.poses=source.poses[:8]
    view.obstacles_callback(source)
    markers=view.obstacle_pub.publish.call_args.args[0].markers
    assert [(m.id,m.action) for m in markers]==[(1,Marker.DELETE),(0,Marker.ADD)]
    source.poses=[];view.obstacles_callback(source)
    assert [(m.id,m.action) for m in view.obstacle_pub.publish.call_args.args[0].markers]==[(0,Marker.DELETE)]
    calls=view.obstacle_pub.publish.call_count
    view.obstacles_callback(source)
    assert view.obstacle_pub.publish.call_count==calls


def test_vehicle_is_unchanged_red_arrow_with_freshness_mirror_and_stale_latch_rejection(view,monkeypatch):
    source=Marker();source.header.frame_id='map';source.header.stamp=Time(seconds=2.).to_msg()
    source.type=Marker.ARROW;source.action=Marker.ADD
    source.color.r,source.color.g,source.color.b,source.color.a=1.,.3,.2,1.
    source.pose.position.x=10.;source.scale.x,source.scale.y,source.scale.z=4.,1.5,1.5
    before=copy.deepcopy(source)
    view.vehicle_callback(source)
    mirror=view.vehicle_pub.publish.call_args.args[0]
    assert mirror.pose==source.pose and mirror.color==source.color and mirror.scale==source.scale
    assert mirror.lifetime.sec==0 and mirror.lifetime.nanosec==0
    assert source==before
    calls=view.vehicle_pub.publish.call_count
    source.header.stamp=Time(seconds=0.).to_msg();view.vehicle_callback(source)
    assert view.vehicle_pub.publish.call_count==calls  # Do not erase a newer valid pose.
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:3.1)
    view.expire()
    assert view.vehicle_pub.publish.call_args.args[0].action==Marker.DELETE
    assert view.test_subscriptions['/route_editor/vehicle_marker'][1].durability==DurabilityPolicy.TRANSIENT_LOCAL
    assert all(pub.qos==10 for pub in view.test_publishers.values())


def test_rviz_uses_distinct_roles_ttl_mirrors_and_draft_is_disabled():
    from pathlib import Path
    config=yaml.safe_load((Path(__file__).parents[1]/'rviz/kcity_route_editor.rviz').read_text())
    displays=config['Visualization Manager']['Displays']
    assert len({d['Name'] for d in displays})==len(displays)
    topics=[d['Topic']['Value'] for d in displays]
    assert len(topics)==len(set(topics))
    assert len({style[1] for style in PATH_STYLES.values()})==len(PATH_STYLES)
    assert '/planning/global_path_marker' in topics and '/planning/local_path_marker' in topics
    assert next(d for d in displays if d['Topic']['Value']=='/route_editor/preview_path')['Enabled'] is False
    assert '/planning/ego_vehicle_marker' in topics


def test_ros_clock_rewind_clears_route_epoch_and_accepts_new_live_paths(view,monkeypatch):
    view.path_callback('global_path',path([(0.,0.,0.),(1.,0.,0.)]))
    view.path_callback('local_path',path([(0.,0.,0.),(1.,0.,0.)]))
    monkeypatch.setattr(Node,'get_clock',lambda self:SimpleNamespace(now=lambda:Time(seconds=1.)))
    view.path_callback('global_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.))
    view.path_callback('local_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.))
    assert view._route_epoch_ns==1_000_000_000
    assert view._path_publishers['local_path'].publish.call_args.args[0].action==Marker.ADD


@pytest.mark.parametrize('key',('global_path','local_path','original_lane_center',
                                'left_lane_center','right_lane_center',
                                'avoidance_selected_path','avoidance_return_path'))
def test_delayed_valid_input_does_not_get_a_sub_frame_render_lifetime(view,monkeypatch,key):
    # Before: age .99 s => lifetime .01 s, then expire/delete before next update.
    source=path([(0.,0.,1.),(1.,0.,1.)],stamp=1.01)
    view.path_callback(key,source)
    pub=view._path_publishers[key]
    marker=pub.publish.call_args.args[0]
    assert marker.action==Marker.ADD
    assert marker.lifetime.sec==0 and marker.lifetime.nanosec==0
    pub.publish.reset_mock()
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:.1)
    view.expire()
    assert pub.publish.call_count==0


def test_unchanged_paths_and_current_lane_only_refresh_cached_adds_for_late_joiners(view,monkeypatch):
    source=path([(0.,0.,0.),(1.,0.,0.)])
    view.path_callback('global_path',source)
    view.current_link_callback(String(data='AB'))
    global_pub=view._path_publishers['global_path']
    global_pub.publish.reset_mock();view.current_pub.publish.reset_mock()
    for _ in range(50):
        view.path_callback('global_path',source)
        view.current_link_callback(String(data='AB'))
    assert global_pub.publish.call_count==view.current_pub.publish.call_count==0
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:1.01)
    view.expire()
    assert global_pub.publish.call_count==view.current_pub.publish.call_count==1
    assert global_pub.publish.call_args.args[0].action==Marker.ADD
    assert view.current_pub.publish.call_args.args[0].action==Marker.ADD


def test_delayed_or_reordered_paths_do_not_erase_latest_geometry(view):
    newer=path([(0.,0.,0.),(2.,0.,0.)])
    view.path_callback('global_path',newer)
    pub=view._path_publishers['global_path'];pub.publish.reset_mock()
    view.path_callback('global_path',path([(0.,0.,0.),(1.,0.,0.)],stamp=1.9))
    view.path_callback('global_path',path([],stamp=.5))
    assert pub.publish.call_count==0
    assert [(p.x,p.y) for p in view._markers['global_path'].points]==[(0.,0.),(2.,0.)]


def test_changed_local_geometry_updates_same_id_with_add_only(view):
    view.path_callback('local_path',path([(0.,0.,0.),(1.,0.,0.)]))
    pub=view._path_publishers['local_path'];pub.publish.reset_mock()
    view.path_callback('local_path',path([(1.,0.,0.),(2.,0.,0.)]))
    assert pub.publish.call_count==1
    marker=pub.publish.call_args.args[0]
    assert (marker.ns,marker.id,marker.action)==('local_path_debug',0,Marker.ADD)
    view.path_callback('local_path',path([]))
    count=pub.publish.call_count
    view.path_callback('local_path',path([]))
    assert pub.publish.call_count==count
    assert pub.publish.call_args.args[0].action==Marker.DELETE


def test_blocked_phase_cannot_alternate_delete_and_add_of_failed_paths(view):
    source=path([(0.,0.,0.),(1.,0.,0.)])
    view.path_callback('avoidance_selected_path',source)
    view.phase_callback(String(data='BLOCKED'))
    pubs=[view._path_publishers[key] for key in view._PHASE_PATHS]
    for pub in pubs:pub.publish.reset_mock()
    for _ in range(20):
        view.phase_callback(String(data='BLOCKED'))
        for key in view._PHASE_PATHS:view.path_callback(key,source)
    assert all(pub.publish.call_count==0 for pub in pubs)
    view.phase_callback(String(data='AVOID_ENTRY'))
    view.path_callback('avoidance_selected_path',source)
    assert pubs[0].publish.call_args.args[0].action==Marker.ADD


def test_not_ready_state_does_not_readd_paths_until_ready(view):
    source=path([(0.,0.,0.),(1.,0.,0.)])
    view.path_callback('global_path',source)
    view.state_callback(String(data='MAP_MATCH_LOST'))
    pub=view._path_publishers['global_path'];pub.publish.reset_mock()
    for _ in range(20):
        view.state_callback(String(data='MAP_MATCH_LOST'))
        view.path_callback('global_path',source)
    assert pub.publish.call_count==0
    view.state_callback(String(data='GLOBAL_PATH_READY'))
    view.path_callback('global_path',source)
    assert pub.publish.call_args.args[0].action==Marker.ADD


def test_obstacle_updates_never_clear_retained_ids_and_repeated_empty_is_silent(view):
    source=footprint()
    view.obstacles_callback(source)
    view.obstacle_pub.publish.reset_mock()
    for i in range(4):
        source.poses[0].position.x+=.1
        view.obstacles_callback(source)
        markers=view.obstacle_pub.publish.call_args.args[0].markers
        assert [(m.id,m.action) for m in markers]==[(0,Marker.ADD)]
    count=view.obstacle_pub.publish.call_count
    view.obstacles_callback(source)
    assert view.obstacle_pub.publish.call_count==count
    source.poses=[]
    view.obstacles_callback(source)
    count=view.obstacle_pub.publish.call_count
    view.obstacles_callback(source)
    assert view.obstacle_pub.publish.call_count==count
    assert all(m.action!=Marker.DELETEALL for call in view.obstacle_pub.publish.call_args_list
               for m in call.args[0].markers)


def test_watchdog_deletes_each_lost_source_once_and_does_not_refresh_it(view,monkeypatch):
    view.path_callback('global_path',path([(0.,0.,0.),(1.,0.,0.)]))
    view.current_link_callback(String(data='AB'))
    view.obstacles_callback(footprint(2))
    for pub in view.test_publishers.values():pub.publish.reset_mock()
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:3.01)
    view.expire()
    assert view._path_publishers['global_path'].publish.call_args.args[0].action==Marker.DELETE
    assert view.current_pub.publish.call_args.args[0].action==Marker.DELETE
    assert [(m.id,m.action) for m in view.obstacle_pub.publish.call_args.args[0].markers]==[(0,Marker.DELETE),(1,Marker.DELETE)]
    counts={key:pub.publish.call_count for key,pub in view.test_publishers.items()}
    view.expire();view.expire()
    assert {key:pub.publish.call_count for key,pub in view.test_publishers.items()}==counts


def test_source_heartbeat_extends_watchdog_without_geometry_republish(view,monkeypatch):
    source=path([(0.,0.,0.),(1.,0.,0.)])
    view.path_callback('global_path',source)
    pub=view._path_publishers['global_path'];pub.publish.reset_mock()
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:2.9)
    view.path_callback('global_path',source)
    view.expire()
    assert pub.publish.call_args.args[0].action==Marker.ADD
    pub.publish.reset_mock()
    monkeypatch.setattr('edgenode_planning.planning_visualization_node.time.monotonic',lambda:3.1)
    view.expire()
    assert pub.publish.call_count==0


def test_display_inputs_keep_only_latest_cycle(view):
    for topic,(_,qos) in view.test_subscriptions.items():
        if topic!='/route_editor/vehicle_marker':assert qos==1


def test_startup_is_the_only_obstacle_deleteall(view):
    startup=view.obstacle_pub.publish.call_args.args[0]
    assert len(startup.markers)==1 and startup.markers[0].action==Marker.DELETEALL
    view.obstacle_pub.publish.reset_mock()
    view.obstacles_callback(footprint())
    assert all(m.action==Marker.ADD for m in view.obstacle_pub.publish.call_args.args[0].markers)
