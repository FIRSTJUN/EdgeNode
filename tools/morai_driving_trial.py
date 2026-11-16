#!/usr/bin/env python3
"""Recorded NORMAL/S4 experiments on the existing, disarmed MORAI stack.

Actor coordinates belong only to this operator and its result files. The only
Planning writes are normal route/speed/session parameters, reset, and arming.
No obstacle publisher, test ID parameter, or synthetic sensor feed is created.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

sys.path[:0] = ['/tmp', '/tmp/edgenode_morai_grpc']
from edgenode_morai_grpc_client import call
from morai_drive import Monitor, rclpy
from edgenode_localization.map_heading import grid_convergence
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import sample, PolylineIndex
from edgenode_planning.route_resume import mission_suffix
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path as NavPath
from pyproj import Proj, Transformer
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
import yaml


def stamp_ns(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


class Recorder(Monitor):
    def __init__(self, output):
        super().__init__()
        self.output = output
        self.paths = {}
        self.path_messages = {}
        self.solids = []
        self.obstacle_stamp = None
        self.pose_history = deque(maxlen=150)
        self.clouds = deque(maxlen=3)
        self.candidate_key = None
        self.marker_key = None
        self.planned_keys = {}
        for topic, kind in [('/localization/grid_convergence_deg', Float32),
                ('/control/target_left', Float32), ('/control/target_forward', Float32),
                ('/control/pure_pursuit_steering', Float32),
                ('/control/lookahead_distance', Float32),
                ('/perception/lidar_diagnostics', String),
                ('/perception/lidar_status', String),('/planning/normal_speed_debug',String)]:
            self.create_subscription(kind, topic, lambda m,t=topic:self.receive(t,m),
                                     qos_profile_sensor_data)
        for key in ('global_path', 'local_path', 'avoidance_selected_path',
                    'avoidance_return_path'):
            self.create_subscription(NavPath, '/planning/'+key,
                lambda m,k=key:self.path(k,m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odometry', self.odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseArray, '/planning/obstacles', self.solid,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points', self.cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/planning/lattice_candidate_markers',
                                 self.markers, qos_profile_sensor_data)

    def receive(self, topic, message):
        super().receive(topic, message)
        if topic == '/ego_vehicle_status':
            self.values[topic]['front_steer_deg'] = message.front_steer_angle
            self.values[topic]['stamp_ns'] = stamp_ns(message)
            self.values[topic]['acceleration_xyz']=[message.acceleration.x,
                message.acceleration.y,message.acceleration.z]
            self.values[topic]['angular_velocity_xyz']=[message.angular_velocity.x,
                message.angular_velocity.y,message.angular_velocity.z]
            self.values[topic]['tire_lateral_forces']=[getattr(message,'tire_lateral_force_'+wheel)
                for wheel in ('fl','fr','rl','rr')]
            self.values[topic]['side_slip_angles']=[getattr(message,'side_slip_angle_'+wheel)
                for wheel in ('fl','fr','rl','rr')]
            self.values[topic]['cornering_stiffness']=[getattr(message,'tire_cornering_stiffness_'+wheel)
                for wheel in ('fl','fr','rl','rr')]
        elif topic == '/ctrl_cmd':
            self.values[topic]['stamp_ns'] = stamp_ns(message)

    def path(self, key, message):
        points = [[p.pose.position.x,p.pose.position.y,p.pose.position.z]
                  for p in message.poses]
        self.paths[key] = points
        self.path_messages[key] = dict(stamp_ns=stamp_ns(message),points=points)
        if key in ('avoidance_selected_path','avoidance_return_path') and points:
            payload=json.dumps(points,separators=(',',':'))
            fingerprint=hashlib.sha256(payload.encode()).hexdigest()
            if self.planned_keys.get(key)!=fingerprint:
                self.planned_keys[key]=fingerprint
                with (self.output/'planned-paths.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(received=time.monotonic(),kind=key,
                        key=fingerprint,points=points))+'\n')

    def solid(self, message):
        self.solids = [[[p.position.x,p.position.y] for p in message.poses[i:i+8]]
                       for i in range(0,len(message.poses),8)]
        self.obstacle_stamp = stamp_ns(message)

    def odom(self, message):
        q=message.pose.pose.orientation;p=message.pose.pose.position
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.values['pose']=[p.x,p.y,yaw]
        self.values['pose_stamp_ns']=stamp_ns(message)
        self.received['pose']=time.monotonic()
        self.pose_history.append(dict(stamp_ns=stamp_ns(message),pose=[p.x,p.y,yaw]))

    def cloud(self, message):
        self.clouds.append(dict(stamp_ns=stamp_ns(message),data=bytes(message.data),
            width=message.width,height=message.height,point_step=message.point_step,
            row_step=message.row_step,is_bigendian=message.is_bigendian,
            fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count)
                    for f in message.fields]))

    def markers(self, message):
        if len(message.markers)<=1:return
        paths=[dict(id=m.id,points=[[p.x,p.y,p.z] for p in m.points])
               for m in message.markers if m.points]
        payload=json.dumps(paths,separators=(',',':'))
        key=hashlib.sha256(payload.encode()).hexdigest()
        if key!=self.marker_key:
            self.marker_key=key
            with (self.output/'candidate-paths.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(received=time.monotonic(),key=key,paths=paths))+'\n')

    def wait(self, condition, seconds=15.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self.spin(.05)
            if condition():return
        raise RuntimeError('readiness condition timeout')

    def reset(self):
        if self.parameters()['enable_drive'] or not self.stopped():
            raise RuntimeError('reset requires verified disarmed stationary vehicle')
        self.wait(lambda:self.values.get('/planning/s4_sensor_state',{}).get('ready'))
        reply=self.call(self.create_client(Trigger,'/planning/reset_avoidance'),Trigger.Request())
        if not reply.success:raise RuntimeError(reply.message)

    def save_clouds(self, label):
        metadata=[]
        for i,cloud in enumerate(self.clouds):
            filename=f'{label}-cloud-{i}.bin'
            (self.output/filename).write_bytes(cloud['data'])
            metadata.append(dict({k:v for k,v in cloud.items() if k!='data'},file=filename))
        (self.output/(label+'-cloud-context.json')).write_text(json.dumps(dict(
            clouds=metadata,pose_history=list(self.pose_history),
            sensor=self.values.get('/perception/lidar_status'),
            diagnostics=self.values.get('/perception/lidar_diagnostics')),indent=2))

    def record(self, start):
        snapshot=self.snapshot()
        debug=snapshot['values'].get('/planning/avoidance_debug',{})
        candidates=debug.get('candidates',[])
        payload=json.dumps(candidates,separators=(',',':'))
        key=hashlib.sha256(payload.encode()).hexdigest()
        if key!=self.candidate_key:
            self.candidate_key=key
            with (self.output/'candidates.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(t=time.monotonic()-start,key=key,candidates=candidates))+'\n')
        snapshot['values']['/planning/avoidance_debug']=dict(
            {k:v for k,v in debug.items() if k!='candidates'},
            candidate_count=len(candidates),candidate_key=key)
        return dict(t=time.monotonic()-start,solids=self.solids,
            obstacle_stamp_ns=self.obstacle_stamp,local_path=self.path_messages.get('local_path'),
            **snapshot)


def restore_ego(node, ego, output):
    desired=ego['transform'];target=json.loads(json.dumps(desired))
    origin=json.loads(Path('/workspace/local_data/kcity_mgeo/global_info.json').read_text())['local_origin_in_global']
    lon,lat=Transformer.from_crs(32652,4326,always_xy=True).transform(
        desired['location']['x']+origin[0],desired['location']['y']+origin[1])
    target['rotation']['z']+=math.degrees(grid_convergence(Proj(32652),lon,lat))
    attempts=[]
    for _ in range(3):
        reply=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=target))
        node.spin(.8);actual=call('actor.Actor','GetActorState',ego['actor_info'])
        error=math.degrees(math.atan2(math.sin(math.radians(desired['rotation']['z']-
            actual['transform']['rotation']['z'])),math.cos(math.radians(desired['rotation']['z']-
            actual['transform']['rotation']['z']))))
        xy=math.hypot(actual['transform']['location']['x']-desired['location']['x'],
                      actual['transform']['location']['y']-desired['location']['y'])
        attempts.append(dict(reply=reply,xy_error_m=xy,yaw_error_deg=error))
        if abs(error)<.02 and xy<.05:
            output['ego_restore']=attempts
            return
        target['rotation']['z']+=error
    raise RuntimeError('Ego restoration did not meet pose tolerance')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--link',required=True)
    parser.add_argument('--station',type=float,required=True)
    parser.add_argument('--goal-node')
    parser.add_argument('--normal-kmh',type=float,default=30.)
    parser.add_argument('--sharp-kmh',type=float,default=30.)
    parser.add_argument('--seconds',type=float,default=90.)
    parser.add_argument('--distance-m',type=float)
    parser.add_argument('--expect',choices=('normal','avoid'),default='normal')
    parser.add_argument('--expected-side',choices=('left','right'))
    parser.add_argument('--obstacle-station',type=float)
    parser.add_argument('--obstacle-lateral-m',type=float,default=0.)
    parser.add_argument('--obstacle-yaw-deg',type=float,default=0.)
    parser.add_argument('--obstacle-model',default='2014_Kia_K7')
    parser.add_argument('--max-error-m',type=float,default=.5)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    numbers=(args.station,args.normal_kmh,args.sharp_kmh,args.seconds,
             args.obstacle_lateral_m,args.obstacle_yaw_deg,args.max_error_m)
    if not all(math.isfinite(x) for x in numbers) or min(args.normal_kmh,args.sharp_kmh,
            args.seconds,args.max_error_m)<=0:parser.error('finite positive speeds/duration/error limit required')
    if args.distance_m is not None and (not math.isfinite(args.distance_m) or args.distance_m<=0):
        parser.error('positive finite distance required')
    if args.obstacle_station is not None and not math.isfinite(args.obstacle_station):
        parser.error('finite obstacle station required')
    args.output.mkdir(parents=True,exist_ok=False)
    def interrupted(*_):raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Recorder(args.output);summary=dict(settings={k:str(v) if isinstance(v,Path) else v
        for k,v in vars(args).items()});rows=[];original=mission=inventory=ego=new_actor=None
    saved=Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml')
    saved_hash=hashlib.sha256(saved.read_bytes()).hexdigest()
    try:
        node.wait(node.fresh);assert node.stop() and node.count_publishers('/ctrl_cmd')==1
        original=node.parameters()
        names=['route_node_ids','route_start_link_id']
        mission=dict(zip(names,map(parameter_value_to_python,node.call(node.getter,
            GetParameters.Request(names=names)).values)))
        inventory=call('actor.Actor','GetAllActorsState',dict(vehicle=True,pedestrian=True,obstacle=True))
        initial_ids={s['actor_info']['id']['value'] for s in inventory['states']}
        ego=next(s for s in inventory['states'] if s['actor_info']['id']['value']=='Ego')
        summary.update(original_parameters=original,original_mission=mission,actors_before=inventory,ego_before=ego)
        (args.output/'recovery-journal.json').write_text(json.dumps(summary,indent=2))
        mapping=DijkstraPlanner('/workspace/local_data/kcity_mgeo')
        saved_route=yaml.safe_load(saved.read_text())
        if args.goal_node:
            route=mapping.plan_route(args.link,[args.goal_node])
            if route is None:raise RuntimeError('unavailable temporary map route')
            waypoints=[args.goal_node]
        else:
            route=mission_suffix(mapping,saved_route.get('start_link_id') or mission['route_start_link_id'],
                                 saved_route['waypoint_node_ids'],args.link)
            waypoints=route['remaining_waypoints']
        points=mapping.links_by_id[args.link]['points'];row=sample(points,args.station)
        if row is None:raise RuntimeError('start outside link geometry')
        xyz,tangent=row
        transform=dict(location=dict(x=xyz[0],y=xyz[1],z=xyz[2]+.35),
                       rotation=dict(z=math.degrees(math.atan2(tangent[1],tangent[0]))))
        obstacle_transform=None
        if args.obstacle_station is not None:
            obstacle_row=sample(points,args.obstacle_station)
            if obstacle_row is None:raise RuntimeError('obstacle outside link geometry')
            p,t=obstacle_row
            obstacle_transform=dict(location=dict(x=p[0]-args.obstacle_lateral_m*t[1],
                y=p[1]+args.obstacle_lateral_m*t[0],z=p[2]+.35),
                rotation=dict(z=math.degrees(math.atan2(t[1],t[0]))+args.obstacle_yaw_deg))
        for actor in inventory['states']:
            if actor['actor_info']['id']['value']=='Ego':continue
            location=actor['transform']['location']
            for target in (transform,obstacle_transform):
                if target is not None and math.hypot(target['location']['x']-location['x'],
                        target['location']['y']-location['y'])<=10.:
                    raise RuntimeError('trial overlaps an original actor')
        if obstacle_transform is not None:
            identifier='s4_integrated_'+str(time.time_ns())
            assert identifier not in initial_ids
            reply=call('actor.Actor','SpawnVehicle',dict(spawn_info=dict(actor_info=dict(
                id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE'),transform=obstacle_transform,
                model_name=args.obstacle_model,label='isolated integrated validation'),velocity=0.,pause=True))
            if reply.get('status')!='STATUS_CODE_SUCCESS' or reply.get('custom_message')!=identifier:
                raise RuntimeError('spawn did not confirm unique trial actor: '+str(reply))
            new_actor=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE')
            summary['obstacle_state']=call('actor.Actor','GetActorState',new_actor)
            assert summary['obstacle_state']['actor_info']==new_actor
        summary.update(target=transform,obstacle_target=obstacle_transform,new_actor=new_actor)
        (args.output/'recovery-journal.json').write_text(json.dumps(summary,indent=2))
        summary['relocation']=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=transform))
        node.wait(lambda:node.fresh() and node.stopped() and node.values.get('/planning/current_link')==args.link
            and 'pose' in node.values and math.dist(node.values['pose'][:2],xyz[:2])<.4,20.)
        node.reset()
        # The complete whitelist of Planning parameter writes. No Actor ID or
        # obstacle coordinate enters this dictionary or any ROS publisher.
        parameters=dict(route_start_link_id=args.link,route_node_ids=waypoints,
            cruise_speed_kmh=args.normal_kmh,sharp_curve_speed_kmh=args.sharp_kmh,
            drive_duration_sec=args.seconds+10.,enable_drive=False)
        node.set_parameters_remote(parameters);node.spin(.8);node.reset()
        node.wait(lambda:node.fresh() and node.values.get('/planning/map_match_status')=='MATCHED'
            and node.values.get('/planning/local_path',{}).get('points',0)>2
            and node.values.get('/planning/s4_sensor_state',{}).get('ready')
            and not node.values['/planning/s4_sensor_state'].get('latched_reason'),30.)
        if node.paths.get('global_path')!=route['points']:
            raise RuntimeError('active Global Path is not exact requested map route')
        if node.values.get('/collision_data')!=0 or not node.stopped():
            raise RuntimeError('preflight collision or stationary check failed')
        summary['planning_parameter_writes']=parameters
        summary['preflight']=node.snapshot();summary['trial_parameters']=node.parameters()
        (args.output/'preflight.json').write_text(json.dumps(summary,indent=2))
        (args.output/'global-path.json').write_text(json.dumps(route['points']))
        node.save_clouds('preflight')
        start=time.monotonic();initial=None;distance=0.;previous=None;seen_return=False;normal_since=None
        node.set_parameters_remote({'enable_drive':True})
        with (args.output/'samples.jsonl').open('a',buffering=1) as stream:
            while time.monotonic()-start<args.seconds:
                node.spin(.05);record=node.record(start);rows.append(record)
                stream.write(json.dumps(record,separators=(',',':'))+'\n')
                values=node.values;debug=values.get('/planning/avoidance_debug',{})
                stage=debug.get('stage',debug.get('state'));state=debug.get('state')
                current=values.get('/ego_vehicle_status',{})
                pose=(current.get('x',0.),current.get('y',0.))
                if previous is not None:distance+=math.dist(previous,pose)
                previous=pose
                if initial is None:initial=pose
                if not node.fresh() or node.count_publishers('/ctrl_cmd')!=1:
                    raise RuntimeError('live freshness or single-control invariant failed')
                if values.get('/collision_data')!=0:raise RuntimeError('MORAI collision')
                sensor=values.get('/planning/s4_sensor_state',{})
                if sensor.get('latched_reason'):raise RuntimeError('sensor latch: '+sensor['latched_reason'])
                if not sensor.get('ready') and values.get('/planning/target_speed',0.)>0.:
                    raise RuntimeError('motion target with unready sensor')
                if state=='BLOCKED':raise RuntimeError('planner rejected: '+debug.get('reason',''))
                if debug.get('boundary_contact_reason'):
                    raise RuntimeError('measured forbidden boundary contact: '+debug['boundary_contact_reason'])
                error=values.get('/control/path_error',math.inf)
                if current.get('speed_kmh',0.)>1. and error>args.max_error_m:
                    raise RuntimeError('experiment path error limit exceeded')
                if args.expect=='normal' and (stage in ('DETECT','AVOID_ENTRY','AVOID_HOLD','RETURN')
                        or debug.get('entry_commits',0)>0 or debug.get('generated_transition_count',0)>0):
                    raise RuntimeError('unexpected S4 intervention in NORMAL experiment')
                if stage=='RETURN':seen_return=True
                returned=seen_return and stage=='NORMAL' and debug.get('return_commits',0)>0
                if returned:
                    normal_since=normal_since or time.monotonic()
                    if args.expected_side and summary.get('selected_side')!=args.expected_side:
                        raise RuntimeError('unexpected avoidance side')
                    if time.monotonic()-normal_since>.5:
                        summary.update(success=True,return_completed=True);break
                if debug.get('selected_side'):summary['selected_side']=debug['selected_side']
                if args.expect=='normal' and args.distance_m is not None and distance>=args.distance_m:
                    summary['success']=True;break
            else:
                if args.expect=='normal':summary['success']=True
                else:raise RuntimeError('avoidance/return did not complete')
        summary.update(driving_time_s=time.monotonic()-start,distance_m=distance)
    except BaseException as error:
        summary['error']=str(error) or type(error).__name__
        node.save_clouds('failure')
    finally:
        try:
            summary['stop_confirmed']=node.stop()
            summary['before_restore']=node.snapshot()
            if not summary['stop_confirmed']:raise RuntimeError('physical stop not confirmed')
            if new_actor is not None:
                assert new_actor['id']['value'] not in initial_ids
                summary['destroy_trial_actor']=call('actor.Actor','DestroyActor',new_actor)
            if ego is not None:restore_ego(node,ego,summary)
            node.spin(2.)
            if original is not None and mission is not None:
                node.set_parameters_remote(dict(mission,enable_drive=False,
                    cruise_speed_kmh=original['cruise_speed_kmh'],sharp_curve_speed_kmh=original['sharp_curve_speed_kmh'],
                    drive_duration_sec=original['drive_duration_sec']))
                node.spin(.8);node.reset()
            summary['final_stop_confirmed']=node.stop()
            summary['final_parameters']=node.parameters();summary['final']=node.snapshot()
            after=call('actor.Actor','GetAllActorsState',dict(vehicle=True,pedestrian=True,obstacle=True))
            summary['actors_after']=after
            current={s['actor_info']['id']['value']:s for s in after['states']}
            errors={}
            if inventory is not None:
                for actor in inventory['states']:
                    key=actor['actor_info']['id']['value'];a=actor['transform'];b=current[key]['transform']
                    xy=math.hypot(a['location']['x']-b['location']['x'],a['location']['y']-b['location']['y'])
                    yaw=abs(math.degrees(math.atan2(math.sin(math.radians(a['rotation']['z']-b['rotation']['z'])),
                                                  math.cos(math.radians(a['rotation']['z']-b['rotation']['z'])))))
                    errors[key]=dict(position_xy_m=xy,yaw_deg=yaw)
                summary['restoration_errors']=errors
                summary['restored']=set(current)==initial_ids and all(v['position_xy_m']<.05
                    and v['yaw_deg']<.02 for v in errors.values())
        except BaseException as error:summary['cleanup_error']=str(error) or type(error).__name__
        summary['saved_route_unchanged']=hashlib.sha256(saved.read_bytes()).hexdigest()==saved_hash
        if rows:
            phases=[]
            for row in rows:
                stage=row['values'].get('/planning/avoidance_debug',{}).get('stage')
                if not phases or phases[-1]!=stage:phases.append(stage)
            errors=[r['values'].get('/control/path_error',0.) for r in rows]
            summary.update(phases=phases,max_control_path_error_m=max(errors),
                mean_control_path_error_m=sum(errors)/len(errors),
                driving_time_s=rows[-1]['t'],distance_m=sum(math.hypot(
                    b['values']['/ego_vehicle_status']['x']-a['values']['/ego_vehicle_status']['x'],
                    b['values']['/ego_vehicle_status']['y']-a['values']['/ego_vehicle_status']['y'])
                    for a,b in zip(rows,rows[1:])),
                peak_speed_kmh=max(r['values'].get('/ego_vehicle_status',{}).get('speed_kmh',0.) for r in rows))
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps({k:v for k,v in summary.items() if k not in ('actors_before','actors_after',
            'ego_before','obstacle_state','preflight','before_restore','final')},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if (summary.get('success') and summary.get('restored') and summary.get('final_stop_confirmed')
                 and summary.get('saved_route_unchanged') and not summary.get('cleanup_error')) else 1


if __name__=='__main__':raise SystemExit(main())
