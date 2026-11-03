"""Real diagnostic ROS transport only; all project topics remapped and no driving."""
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from edgenode_planning.planning_visualization_node import PlanningVisualizationNode, PATH_STYLES


def test_real_visualization_transport_copies_path_and_deletes_stale_source(route_editor_map):
    prefix='/planning_viz_test_'+uuid.uuid4().hex[:8]
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}',
          '-p','freshness_sec:=0.15','-p','source_timeout_sec:=0.45']
    topics={'/planning/'+key for key in PATH_STYLES}
    topics.update('/planning/'+style[0] for style in PATH_STYLES.values())
    topics.update('/planning/'+name for name in ('current_link','current_lane_marker','state',
                  'lane_change_state','obstacles','obstacle_footprint_markers','ego_vehicle_marker'))
    topics.add('/route_editor/vehicle_marker')
    for topic in topics:
        args.extend(['-r',f'{topic}:={prefix}/{topic.strip("/").replace("/","_")}'])
    rclpy.init(args=args)
    nodes=[];executor=SingleThreadedExecutor()
    def until(predicate,timeout=3.):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate()
    try:
        visual=PlanningVisualizationNode();nodes.append(visual)
        probe=Node('probe');nodes.append(probe)
        for node in nodes:executor.add_node(node)
        received=[]
        probe.create_subscription(Marker,prefix+'/planning_global_path_marker',received.append,10)
        source=probe.create_publisher(Path,prefix+'/planning_global_path',10)
        until(lambda:source.get_subscription_count()>0 and visual._path_publishers['global_path'].get_subscription_count()>0)
        path=Path();path.header.frame_id='map';path.header.stamp=probe.get_clock().now().to_msg()
        for x,y in ((0.,0.),(1.,2.),(2.,3.)):
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=y;path.poses.append(pose)
        source.publish(path)
        until(lambda:any(m.action==Marker.ADD and len(m.points)==3 for m in received))
        marker=next(m for m in received if m.action==Marker.ADD and len(m.points)==3)
        assert [(p.x,p.y) for p in marker.points]==[(0.,0.),(1.,2.),(2.,3.)]
        assert marker.lifetime.sec==0 and marker.lifetime.nanosec==0
        # Survive beyond the old .15 s render TTL, then expire by watchdog.
        deadline=time.monotonic()+.23
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert received[-1].action==Marker.ADD
        until(lambda:received[-1].action==Marker.DELETE)
    finally:
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.shutdown()


def test_real_volatile_late_joiner_receives_unchanged_cached_path(route_editor_map):
    prefix='/planning_viz_late_'+uuid.uuid4().hex[:8]
    args=['--ros-args','-r','__ns:='+prefix,'-p',f'mgeo_dir:={route_editor_map}',
          '-p','freshness_sec:=0.15','-p','source_timeout_sec:=2.0']
    topics={'/planning/'+key for key in PATH_STYLES}
    topics.update('/planning/'+style[0] for style in PATH_STYLES.values())
    topics.update('/planning/'+name for name in ('current_link','current_lane_marker','state',
                  'lane_change_state','obstacles','obstacle_footprint_markers','ego_vehicle_marker'))
    topics.add('/route_editor/vehicle_marker')
    for topic in topics:
        args.extend(['-r',f'{topic}:={prefix}/{topic.strip("/").replace("/","_")}'])
    rclpy.init(args=args)
    nodes=[];executor=SingleThreadedExecutor()
    def until(predicate,timeout=2.):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate()
    try:
        visual=PlanningVisualizationNode();probe=Node('late_probe');nodes.extend((visual,probe))
        for node in nodes:executor.add_node(node)
        source=probe.create_publisher(Path,prefix+'/planning_global_path',10)
        until(lambda:source.get_subscription_count()>0)
        path=Path();path.header.frame_id='map';path.header.stamp=probe.get_clock().now().to_msg()
        for x,y in ((0.,0.),(1.,2.),(1.,2.),(3.,4.)):
            pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=y;path.poses.append(pose)
        source.publish(path)
        until(lambda:'global_path' in visual._markers)
        # Join after the only source publish. Volatile QoS needs cached ADD refresh.
        received=[]
        probe.create_subscription(Marker,prefix+'/planning_global_path_marker',received.append,10)
        until(lambda:bool(received),timeout=1.5)
        assert all(marker.action==Marker.ADD for marker in received)
        assert [(p.x,p.y) for p in received[-1].points]==[(0.,0.),(1.,2.),(1.,2.),(3.,4.)]
        assert received[-1].lifetime.sec==received[-1].lifetime.nanosec==0
    finally:
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.shutdown()
