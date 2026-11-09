#!/usr/bin/env python3
"""Stage 5C measurements, not vehicle/visibility/map approval.

Run baseline/current in separate processes with identical --source-root layouts.
Production algorithms and, for --kind node, the actual disarmed PlanningNode
callback are called. Node output publishers are memory sinks; no Control node.
Profiled spans are diagnostic (nested inclusive/exclusive), never percentile
samples. Uninstrumented repeats report wall and process CPU independently.
"""
import argparse
from collections import defaultdict
from contextlib import ExitStack, nullcontext
from dataclasses import replace
from functools import wraps
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def dump(path,value):
    path.write_text(json.dumps(clean(value),indent=2,allow_nan=False)+'\n')


def percentile(values,q):
    ordered=sorted(values);p=(len(ordered)-1)*q;i=math.floor(p);j=math.ceil(p)
    return ordered[i]*(j-p)+ordered[j]*(p-i) if i!=j else ordered[i]


def distribution(values):
    return dict(n=len(values),p50=statistics.median(values),p95=percentile(values,.95),
                p99=percentile(values,.99),observed_max=max(values),minimum=min(values))


class Spans:
    def __init__(self):
        self.rows=defaultdict(lambda:dict(calls=0,wall_ns=0,cpu_ns=0,self_wall_ns=0,self_cpu_ns=0))
        self.stack=[];self.context=ExitStack()

    def wrap(self,target,name,label):
        original=getattr(target,name)
        @wraps(original)
        def measured(*args,**kwargs):
            frame=[time.perf_counter_ns(),time.process_time_ns(),0,0];self.stack.append(frame)
            try:return original(*args,**kwargs)
            finally:
                cpu=time.process_time_ns()-frame[1];wall=time.perf_counter_ns()-frame[0]
                self.stack.pop();row=self.rows[label];row['calls']+=1
                row['wall_ns']+=wall;row['cpu_ns']+=cpu
                row['self_wall_ns']+=wall-frame[2];row['self_cpu_ns']+=cpu-frame[3]
                if self.stack:self.stack[-1][2]+=wall;self.stack[-1][3]+=cpu
        self.context.enter_context(patch.object(target,name,measured))

    def __enter__(self):return self
    def __exit__(self,*args):self.context.close()


def load(source_root):
    base=source_root/'ros2_ws/src'
    for directory in ('edgenode_control','edgenode_planning','edgenode_planning/test'):
        sys.path.insert(0,str(base/directory))
    loop=importlib.import_module('synthetic_s4_loop')
    modules={name:importlib.import_module('edgenode_planning.'+name) for name in
        ('static_obstacle_planner','avoidance_geometry','lattice_path','local_path_planner',
         'sensor_readiness','s4_distance_safety','planning_node','map_matcher','dijkstra_planner','speed_planner')}
    return loop,modules


def instrument(spans,m):
    for name,label in [('_normal_hazards','02_hazard_validation'),('_remember','02_obstacle_merge'),
        ('_generate','03_candidate_generation'),('_body_safety','04_vehicle_collision'),
        ('_path_safety','06_path_constraints'),('_rollout','06_control_rollout'),('plan','03_lattice_plan')]:
        spans.wrap(m['static_obstacle_planner'].StaticObstaclePlanner,name,label)
    spans.wrap(m['avoidance_geometry'].DrivableArea,'covers','05_drivable_area')
    spans.wrap(m['avoidance_geometry'],'_half_plane','05_polygon_clipping')
    if hasattr(m['avoidance_geometry'],'_split_half_plane'):
        spans.wrap(m['avoidance_geometry'],'_split_half_plane','05_polygon_clipping')
    for name,label in [('swept_body','04_swept_body'),('polygon_distance','04_obstacle_distance'),
                       ('obstacle_polygons','02_obstacle_conversion'),('curvatures','06_curvature'),
                       ('reference_cost','07_candidate_cost'),('control_local_path','08_control_path'),
                       ('center_suffix','03_center_sampling'),('transition','03_transition')]:
        spans.wrap(m['static_obstacle_planner'],name,label)
    spans.wrap(m['local_path_planner'].LocalPathPlanner,'extract','01_local_extract')
    spans.wrap(m['sensor_readiness'].SensorReadiness,'reason','09_sensor_readiness')
    spans.wrap(m['s4_distance_safety'].DistanceSafety,'evaluate','09_distance_safety')
    for name,label in [('plan','10_planning_callback'),('obstacle_callback','02_obstacle_callback'),
        ('localization_callback','01_pose_callback'),('sensor_status_callback','09_status_callback'),
        ('publish_avoidance','10_diagnostic_publication')]:
        spans.wrap(m['planning_node'].PlanningNode,name,label)
    spans.wrap(m['map_matcher'].MapMatcher,'match','01_map_matching')
    spans.wrap(m['dijkstra_planner'].DijkstraPlanner,'plan','01_global_route')
    spans.wrap(m['speed_planner'].SpeedPlanner,'plan','07_speed_plan')


def result_signature(result):
    from dataclasses import asdict
    values=dict(state=result['state'],reason=result['reason'],points=result['points'],
        candidates=[asdict(c) for c in result['candidates']],selected_side=result['selected_side'],
        return_path=result['return_path'],clearance_m=result['clearance_m'])
    return hashlib.sha256(json.dumps(clean(values),sort_keys=True).encode()).hexdigest()


def make_case(loop,m,case):
    scene=loop.Scene(name=case['name'],radius_m=case.get('radius',0.),side=case.get('side','left'))
    road=loop.Road(scene);pose=(*road.point(5.1,0.)[:2],5.1/scene.radius_m if scene.radius_m else 0.)
    config=replace(loop.AvoidanceConfig(),transition_lengths_m=tuple(case.get('lengths',(12.,16.,20.))))
    planner=loop.StaticObstaclePlanner(road.network,drivable_area=road.area,config=config)
    groups=list(road.obstacles)
    for i in range(case.get('obstacles',1)-1):
        groups.append(road.obstacle(15.+i*.6,1.,.5,0.,lateral=-10.-i*.8))
    local=loop.LocalPathPlanner();spacing=case.get('spacing',2.)
    def prepare():
        normal=local.extract(road.reference,*pose[:2],horizon_m=20.,resample_spacing_m=spacing)['points']
        preview=local.extract(road.reference,*pose[:2],horizon_m=80.,resample_spacing_m=spacing)['points']
        return normal,preview
    policy=loop.DistancePolicy();evidence=loop.ObservationEvidence(50.,20.,config.curvature_limit,
        tuple(road.route_ids),'SYNTHETIC_STAGE5C_BENCHMARK_ONLY')
    safety=loop.DistanceSafety(policy,config,planner.body,evidence)
    def call(reset=True):
        if reset:planner.reset()
        normal,preview=prepare();metrics=safety.evaluate(case.get('speed',3.),.1,1/scene.radius_m if scene.radius_m else 0.,road.route_ids)
        planner.needs_validation_stop(preview,groups)
        result=planner.plan(normal,groups,pose,speed_kmh=case.get('speed',3.),
            actual_curvature=1/scene.radius_m if scene.radius_m else 0.,current_link_id='SYN_O0',
            route_link_ids=road.route_ids,hazard_reference=preview,entry_reserve_m=metrics['entry_reserve_m'],
            stopping_required_m=metrics['stopping_required_m'],entry_observation_m=50.,
            observation_curvature_limit=config.curvature_limit)
        if result['active'] and not result['blocked']:
            loop.control_local_path(result['points'],pose,policy.control_horizon_m)
        return result
    return SimpleNamespace(scene=scene,road=road,pose=pose,planner=planner,groups=groups,call=call,
                           prepare=prepare,evidence=evidence,safety=safety)


class Sink:
    def __init__(self,node,name,events):self.node=node;self.name=name;self.events=events;self.last=None
    def publish(self,msg):
        self.last=msg
        if self.name in ('local_path','target_speed'):
            stamp=msg.header.stamp if hasattr(msg,'header') else None
            self.events.append(dict(topic=self.name,monotonic_ns=time.monotonic_ns(),
                ros_ns=self.node.get_clock().now().nanoseconds,
                message_stamp_ns=(stamp.sec*10**9+stamp.nanosec if stamp is not None else None),
                path_points=len(msg.poses) if hasattr(msg,'poses') else None,
                target_speed=getattr(msg,'data',None)))


class NodeCase:
    """Actual ROS Node + actual callback; disarmed and memory output only."""
    def __init__(self,fixture,m,directory,live_timers=False):
        import rclpy
        from geometry_msgs.msg import Pose,PoseArray
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self.fixture=fixture;self.m=m;self.events=[];self.rclpy=rclpy
        self.Pose=Pose;self.PoseArray=PoseArray;self.Odometry=Odometry;self.String=String
        source=Path(directory);source.mkdir(parents=True,exist_ok=True)
        links=[];nodes={}
        for record in fixture.road.network.links.values():
            record=dict(record);record['points']=[list(p) for p in record['points']]
            links.append(record)
            for key,p in ((record['from_node_idx'],record['points'][0]),(record['to_node_idx'],record['points'][-1])):
                nodes[key]=dict(idx=key,point=p)
        dump(source/'link_set.json',links);dump(source/'node_set.json',list(nodes.values()))
        dump(source/'global_info.json',dict(global_coordinate_system='SYNTHETIC_LOCAL',local_origin_in_global=[0.,0.,0.]))
        args=['--ros-args','-r','__ns:=/stage5c_disarmed','-p',f'mgeo_dir:={source}',
            '-p',f'goal_node_id:={links[-2]["to_node_idx"]}','-p','s4_sensor_validation_enabled:=true',
            '-p','enable_drive:=false','-p','cruise_speed_kmh:=3.0','-p','curve_speed_kmh:=3.0','-p','sharp_curve_speed_kmh:=3.0']
        rclpy.init(args=args)
        with (nullcontext() if live_timers else patch.object(m['planning_node'].PlanningNode,'create_timer')):
            self.node=m['planning_node'].PlanningNode()
        node=self.node
        # No deployment schema/pin, no approval file. Test objects only.
        node.avoidance_planner=fixture.planner
        node.distance_safety=fixture.safety
        self.ros_publishers={name:getattr(node,name) for name in node.__dict__ if name.endswith('_pub')}
        for name in list(node.__dict__):
            if name.endswith('_pub'):
                label='local_path' if name=='local_path_pub' else 'target_speed' if name=='target_speed_pub' else name
                setattr(node,name,Sink(node,label,self.events))
        node._avoidance_path_publishers={k:Sink(node,k,self.events) for k in node._avoidance_path_publishers}
        assert node.get_parameter('enable_drive').value is False

    def feed(self,source_age=.05,reset_guard=True):
        node=self.node;now=node.get_clock().now().nanoseconds;stamp=now-round(source_age*1e9)
        if reset_guard:
            node.sensor_guard=self.m['sensor_readiness'].SensorReadiness()
            node.latest_pose_stamp_ns=None
            self.feed_seq=0
        self.feed_seq=getattr(self,'feed_seq',0)+1
        pose=self.Odometry();pose.header.frame_id='map';pose.header.stamp.sec=stamp//10**9;pose.header.stamp.nanosec=stamp%10**9
        x,y,yaw=self.fixture.pose;pose.pose.pose.position.x=float(x);pose.pose.pose.position.y=float(y)
        pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2)
        pose.twist.twist.linear.x=3./3.6
        pose.twist.twist.angular.z=pose.twist.twist.linear.x/self.fixture.scene.radius_m if self.fixture.scene.radius_m else 0.
        self.pose_message=pose
        node.localization_callback(pose)
        obstacles=self.PoseArray();obstacles.header=pose.header
        points=[p for group in self.fixture.groups for p in group]
        for x,y in points:
            point=self.Pose();point.position.x=float(x);point.position.y=float(y);obstacles.poses.append(point)
        arrival=node.get_clock().now().nanoseconds;mono=time.monotonic_ns();node.obstacle_callback(obstacles)
        status=dict(schema_version=1,clock='ros',session_id='c'*32,status_seq=self.feed_seq,frame_seq=self.feed_seq,fault_seq=0,
            state='READY_OBSTACLES' if self.fixture.groups else 'READY_EMPTY',source_stamp_ns=stamp,frame_received_ns=arrival,published_ns=arrival,
            cluster_count=len(self.fixture.groups),obstacle_digest=self.m['sensor_readiness'].footprint_digest('map',stamp,points))
        node.sensor_status_callback(self.String(data=json.dumps(status)))
        return dict(source_stamp_ns=stamp,arrival_ros_ns=arrival,arrival_monotonic_ns=mono)

    def call(self,reset=True,source_age=.05):
        node=self.node
        if reset:
            node.avoidance_planner.reset();node._route_key=None;node._route_result=None
            node._validation_pending_stamp_ns=None;node._validation_fresh_stamps.clear()
        self.events.clear();timeline=self.feed(source_age)
        timeline.update(start_ros_ns=node.get_clock().now().nanoseconds,start_monotonic_ns=time.monotonic_ns())
        node.plan()
        timeline.update(end_ros_ns=node.get_clock().now().nanoseconds,end_monotonic_ns=time.monotonic_ns(),publications=list(self.events),
            sensor_reason_at_end=node.sensor_reason(node.get_clock().now().nanoseconds),
            final_speed=node.target_speed_pub.last.data,local_points=len(node.local_path_pub.last.poses))
        assert timeline['final_speed']==0. and not node.get_parameter('enable_drive').value
        return timeline

    def close(self):
        self.node.destroy_node();self.rclpy.shutdown()


def cases():
    result=[dict(name=f'{shape}_{side}',radius=radius,side=side) for radius,shape in ((0.,'straight'),(300.,'curve')) for side in ('left','right')]
    result += [dict(name=f'curve_obstacles_{n}',radius=300.,obstacles=n) for n in (4,12)]
    result += [dict(name=f'curve_candidates_{n}',radius=300.,lengths=list(range(12,12+2*n,2))) for n in (1,6)]
    result += [dict(name='curve_dense_preview',radius=300.,spacing=.25)]
    result += [dict(name=f'curve_speed_{v}_analysis',radius=300.,speed=float(v)) for v in (10,20)]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--kind',choices=('algorithm','node'),default='algorithm')
    parser.add_argument('--repeats',type=int,default=40)
    parser.add_argument('--match',default='')
    parser.add_argument('--pin-cpu',type=int)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    if args.pin_cpu is not None:os.sched_setaffinity(0,{args.pin_cpu})
    started=time.perf_counter();loop,m=load(args.source_root.resolve());import_s=time.perf_counter()-started
    output=[]
    for case in cases():
        if args.match and case['name'] not in args.match.split(','):continue
        if (args.kind=='node' and not args.match and
                case['name'] not in ('straight_left','curve_left','curve_right')):continue
        fixture=make_case(loop,m,case);adapter=None
        with tempfile.TemporaryDirectory(prefix='stage5c_disarmed_') as directory:
            if args.kind=='node':adapter=NodeCase(fixture,m,directory)
            def invoke(reset=True):return adapter.call(reset) if adapter else fixture.call(reset)
            records=[];signatures=[];timelines=[]
            for index in range(args.repeats+3):
                wall=time.perf_counter_ns();cpu=time.process_time_ns();value=invoke()
                row=dict(index=index,wall_s=(time.perf_counter_ns()-wall)/1e9,cpu_s=(time.process_time_ns()-cpu)/1e9)
                if index==0:cold=row
                if index>=3:records.append(row)
                if adapter:timelines.append(value)
                else:signatures.append(result_signature(value))
            # Steady means an already committed fixed path. No candidate search
            # or receipt timestamp is reused. It is a separate workload.
            fixture.planner.reset();fixture.call()
            steady=[]
            for index in range(args.repeats):
                wall=time.perf_counter_ns();cpu=time.process_time_ns();invoke(False)
                steady.append(dict(index=index,wall_s=(time.perf_counter_ns()-wall)/1e9,cpu_s=(time.process_time_ns()-cpu)/1e9))
            with Spans() as spans:
                instrument(spans,m);profiled=invoke()
            normal,preview=fixture.prepare()
            row=dict(case=case,kind=args.kind,source_root=str(args.source_root.resolve()),
                module_file=m['static_obstacle_planner'].__file__,cold=cold,warm_search_samples=records,steady_samples=steady,
                warm_search_wall=distribution([r['wall_s'] for r in records]),warm_search_cpu=distribution([r['cpu_s'] for r in records]),
                steady_wall=distribution([r['wall_s'] for r in steady]),steady_cpu=distribution([r['cpu_s'] for r in steady]),
                profile_includes_instrumentation_overhead=True,profile_spans=dict(spans.rows),
                signatures=sorted(set(signatures)),timelines=timelines,input_points=len(normal),preview_points=len(preview),
                candidate_points=[len(c.points) for c in fixture.planner.candidates],candidate_rejections=[c.rejection for c in fixture.planner.candidates],
                candidate_count=len(fixture.planner.candidates),obstacle_count=len(fixture.groups),
                observed_max_is_not_wcet=True)
            if adapter:adapter.close()
        output.append(row);dump(args.out/(case['name']+'.json'),row)
        dump(args.out/'summary.json',dict(import_wall_s=import_s,affinity=sorted(os.sched_getaffinity(0)),results=output))
        print(case['name'],args.kind,'cold',round(cold['wall_s'],4),'warm p95',round(row['warm_search_wall']['p95'],4),flush=True)


if __name__=='__main__':main()
