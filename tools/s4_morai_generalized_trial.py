"""Reuse the checkpoint's stopped, isolated MORAI trial and restore its state.

Run from the sourced Dev Container with the existing stack. No nodes or Docker
containers are launched. Only the temporary actor is removed after the trial.
"""
import argparse,hashlib,json,math,signal,sys,time,subprocess,os,tarfile
from pathlib import Path
sys.path[:0]=['/workspace/tools','/tmp','/tmp/edgenode_morai_grpc']
from edgenode_morai_grpc_client import call
from morai_drive import Monitor,rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Path as NavPath,Odometry
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger
from std_msgs.msg import String,Float32
from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.lane_geometry import sample,PolylineIndex
from edgenode_planning.route_resume import mission_suffix
import yaml
from pyproj import Proj,Transformer
from edgenode_localization.map_heading import grid_convergence
p=argparse.ArgumentParser();p.add_argument('--link',required=True);p.add_argument('--start',type=float,required=True);p.add_argument('--obstacle',type=float);p.add_argument('--obstacle-yaw-deg',type=float,default=0.);p.add_argument('--output',type=Path,required=True)
p.add_argument('--follow-distance-m',type=float,help='Follow this saved-route distance without spawning an obstacle')
p.add_argument('--obstacle-link',help='Place the primary obstacle on this link instead of the start link')
p.add_argument('--obstacle-lateral-m',type=float,default=0.,help='Signed map-tangent lateral displacement, left positive')
p.add_argument('--tracking-only-seconds',type=float,help='Keep Ego disarmed while measuring a moving temporary NPC')
p.add_argument('--actor-motion-kmh',type=float,default=0.)
p.add_argument('--additional-obstacle',action='append',default=[],metavar='LINK:STATION:YAW_DEG',
 help='Additional independent actors; used only for placement and post-run audit')
p.add_argument('--approach-speed-kmh',type=float,default=10.)
p.add_argument('--seconds',type=float,default=100.)
p.add_argument('--record-bag',action='store_true')
p.add_argument('--prepare-before-spawn',action='store_true',
 help='Record the ready native candidate bank at the start pose before placing the obstacle')
a=p.parse_args()
if a.follow_distance_m is None and a.obstacle is None:p.error('--obstacle or --follow-distance-m required')
if a.follow_distance_m is not None and (a.obstacle is not None or not math.isfinite(a.follow_distance_m) or a.follow_distance_m<=0):p.error('positive follow distance without obstacle required')
if not all(math.isfinite(v) for v in (a.start,a.obstacle_yaw_deg,a.obstacle_lateral_m) if v is not None):p.error('finite station, offset and yaw required')
if a.obstacle is not None and not math.isfinite(a.obstacle):p.error('finite obstacle station required')
if a.tracking_only_seconds is not None and (a.obstacle is None or not math.isfinite(a.tracking_only_seconds) or a.tracking_only_seconds<=0 or not math.isfinite(a.actor_motion_kmh) or a.actor_motion_kmh<=0):p.error('tracking requires an obstacle, positive duration and NPC speed')
if a.actor_motion_kmh and a.tracking_only_seconds is None:p.error('moving NPC currently requires --tracking-only-seconds')
if not all(math.isfinite(v) and v>0 for v in (a.approach_speed_kmh,a.seconds)):p.error('positive finite speed and duration required')
a.output.mkdir(exist_ok=False)
class Recorder(Monitor):
 def __init__(self):
  super().__init__();self.paths={};self.solids=[]
  self._candidate_key=None;self._candidate_ref=None;self._candidate_serial=0;self._candidate_counts={}
  self.create_subscription(Float32,'/localization/grid_convergence_deg',lambda m:self.receive('/localization/grid_convergence_deg',m),qos_profile_sensor_data)
  self.create_subscription(String,'/perception/lidar_diagnostics',lambda m:self.receive('/perception/lidar_diagnostics',m),qos_profile_sensor_data)
  for topic,kind in [('/planning/normal_speed_debug',String),('/planning/diagnostics',String),
                     ('/planning/lane_candidates',String),
                     ('/planning/traffic_tracks',String),
                     ('/control/lookahead_distance',Float32),('/control/pure_pursuit_steering',Float32)]:
   self.create_subscription(kind,topic,lambda m,t=topic:self.receive(t,m),qos_profile_sensor_data)
  for key in ('global_path','avoidance_selected_path','avoidance_return_path'):
   self.create_subscription(NavPath,'/planning/'+key,lambda m,k=key:self.path(k,m),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/localization/odometry',self.odom,qos_profile_sensor_data)
  self.create_subscription(PoseArray,'/planning/obstacles',self.solid,qos_profile_sensor_data)
 def receive(self,topic,m):
  super().receive(topic,m)
  if topic=='/ego_vehicle_status':
   self.values[topic]['front_steer_deg']=m.front_steer_angle
   self.values[topic]['source_stamp_ns']=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
  if topic=='/planning/local_path':self.path('control_local_path',m)
 def path(self,k,m):self.paths[k]=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]
 def solid(self,m):self.solids=[[[p.position.x,p.position.y] for p in m.poses[i:i+8]] for i in range(0,len(m.poses),8)]
 def odom(self,m):
  q=m.pose.pose.orientation;p=m.pose.pose.position;self.values['pose']=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))];self.received['pose']=time.monotonic()
 def wait(self,condition,seconds=12.):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   self.spin(.05)
   if condition():return
  raise RuntimeError('readiness condition timeout')
 def reset(self):
  assert not self.parameters()['enable_drive'] and self.stopped()
  self.wait(lambda:self.values.get('/planning/s4_sensor_state',{}).get('ready'))
  r=self.call(self.create_client(Trigger,'/planning/reset_avoidance'),Trigger.Request())
  if not r.success:raise RuntimeError(r.message)
 def record(self,start):
  snapshot=self.snapshot();values=snapshot['values'];debug=dict(values.get('/planning/avoidance_debug',{}))
  candidates=debug.pop('candidates',[])
  key=(debug.get('generated_transition_count'),debug.get('candidate_lane_batches'),
       debug.get('entry_commits'),debug.get('return_commits'),len(candidates),
       debug.get('entry_context',{}).get('current_link_id'))
  if key!=self._candidate_key:
   from collections import Counter
   self._candidate_key=key;self._candidate_ref='c'+str(self._candidate_serial);self._candidate_serial+=1
   self._candidate_counts=dict(Counter(c.get('rejection','') for c in candidates))
   with (a.output/'candidate_sets.jsonl').open('a') as stream:
    stream.write(json.dumps(dict(id=self._candidate_ref,t=time.monotonic()-start,candidates=candidates))+'\n')
  debug.update(candidate_set_id=self._candidate_ref,candidate_count=len(candidates),
               candidate_rejection_counts=self._candidate_counts)
  values['/planning/avoidance_debug']=debug
  return dict(t=time.monotonic()-start,solids=self.solids,
   local_path_points=self.paths.get('control_local_path',[]),**snapshot)
def interrupted(*args):raise KeyboardInterrupt()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Recorder();summary={};rows=[];path_fingerprints={};new_actor=None;spawned_actors=[];ego=None;original=None;mission=None;bag=None;bag_log=None
saved=Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml');saved_hash=hashlib.sha256(saved.read_bytes()).hexdigest()
try:
 n.wait(n.fresh);assert n.stop() and n.count_publishers('/ctrl_cmd')==1
 original=n.parameters();names=['route_node_ids','route_start_link_id'];mission=dict(zip(names,map(parameter_value_to_python,n.call(n.getter,GetParameters.Request(names=names)).values)))
 summary['trial_spec']=dict(link=a.link,start_m=a.start,obstacle_m=a.obstacle,
  obstacle_yaw_deg=a.obstacle_yaw_deg,approach_speed_kmh=a.approach_speed_kmh,seconds=a.seconds,
  follow_distance_m=a.follow_distance_m,obstacle_link=a.obstacle_link or a.link,
  obstacle_lateral_m=a.obstacle_lateral_m,prepare_before_spawn=a.prepare_before_spawn)
 summary['verification']='MORAI_TRACKING_ONLY' if a.tracking_only_seconds else 'MORAI_LIVE'
 names=['clipped_footprints','footprint_tile_size_m','publish_debug_clouds','time_aligned_obstacles','roi_max_x','ground_z_threshold','source_pair_wait_sec','source_pose_max_gap_sec']
 summary['perception_parameters']=dict(zip(names,map(parameter_value_to_python,n.call(
  n.create_client(GetParameters,'/lidar_obstacle_node/get_parameters'),GetParameters.Request(names=names)).values)))
 names=['avoidance_deceleration_mps2','avoidance_memory_merge_extent_m','avoidance_avoid_speed_kmh','s4_control_horizon_m','morai_unmarked_junction_experiment',
        'avoidance_continuous_lane_follow','avoidance_integrated_strategy','morai_surface_offset_experiment','morai_transient_sensor_recovery',
        'avoidance_optimize_entry_onset','avoidance_tracking_limit_m','planning_period_sec','static_avoidance_method']
 summary['s4_parameters']=dict(zip(names,map(parameter_value_to_python,n.call(n.getter,GetParameters.Request(names=names)).values)))
 summary['code_sha']=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
 summary['working_diff_sha256']=hashlib.sha256(subprocess.check_output(['git','diff'])).hexdigest()
 changed=subprocess.check_output(['git','diff','--name-only'],text=True).splitlines()
 changed+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],text=True).splitlines()
 summary['working_files_sha256']={name:hashlib.sha256(Path(name).read_bytes()).hexdigest()
                                for name in changed if Path(name).is_file()}
 with tarfile.open(a.output/'source-snapshot.tar.gz','w:gz') as archive:
  for name in summary['working_files_sha256']:archive.add(name,arcname=name)
 if a.record_bag:
  topics=['/ego_vehicle_status','/ctrl_cmd','/localization/odometry','/control/path_error',
   '/control/lookahead_distance','/collision_data','/lidar/points','/Imu','/gps',
   '/perception/lidar_obstacle_count','/perception/lidar_obstacle_centers',
   '/perception/lidar_diagnostics','/perception/lidar_status','/perception/lidar_frame','/planning/obstacles',
   '/perception/lidar_points_map_xy','/perception/lidar_cluster_points_map_xy',
   '/planning/current_link','/planning/map_match_status','/planning/target_speed',
   '/planning/global_path','/planning/local_path','/planning/avoidance_debug',
   '/planning/s4_sensor_state','/planning/s4_distance_state','/planning/fsm_state',
   '/planning/drive_status','/planning/speed_mode','/localization/grid_convergence_deg',
   '/planning/diagnostics','/planning/normal_speed_debug','/planning/traffic_tracks','/control/pure_pursuit_steering','/planning/lane_candidates']
  bag_log=(a.output/'rosbag-console.log').open('w')
  bag=subprocess.Popen(['ros2','bag','record','-o',str(a.output/'bag'),*topics],stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
  n.spin(2.)
  if bag.poll() is not None:raise RuntimeError('rosbag recorder failed to start')
 summary['original_parameters']=original;summary['original_mission']=mission
 inventory=call('actor.Actor','GetAllActorsState',dict(vehicle=True,pedestrian=True,obstacle=True));summary['actors_before']=inventory
 ego=call('actor.Actor','GetActorState',dict(id={'value':'Ego'},object_type='OBJECT_TYPE_VEHICLE'))
 planner=DijkstraPlanner('/workspace/local_data/kcity_mgeo');points=planner.links_by_id[a.link]['points'];xyz,tangent=sample(points,a.start)
 transform=dict(location=dict(x=xyz[0],y=xyz[1],z=xyz[2]+.35),rotation=dict(z=math.degrees(math.atan2(tangent[1],tangent[0]))))
 obstacle_transform=None
 if a.obstacle is not None:
  obstacle_xyz,obstacle_tangent=sample(planner.links_by_id[a.obstacle_link or a.link]['points'],a.obstacle)
  obstacle_xyz=[obstacle_xyz[0]-a.obstacle_lateral_m*obstacle_tangent[1],
                obstacle_xyz[1]+a.obstacle_lateral_m*obstacle_tangent[0],obstacle_xyz[2]]
  obstacle_transform=dict(location=dict(x=obstacle_xyz[0],y=obstacle_xyz[1],z=obstacle_xyz[2]+.35),rotation=dict(z=math.degrees(math.atan2(obstacle_tangent[1],obstacle_tangent[0]))+a.obstacle_yaw_deg))
 for actor in inventory.get('states',[]):
  if actor['actor_info']['id']['value']=='Ego':continue
  location=actor['transform']['location']
  for target in (transform,obstacle_transform):
   if target is None:continue
   assert math.hypot(target['location']['x']-location['x'],target['location']['y']-location['y'])>10.,'trial overlaps an original actor'
 summary['ego_before']=ego;summary['target']=transform;summary['obstacle_target']=obstacle_transform
 if a.prepare_before_spawn:
  # This is experiment placement only. The planner receives no actor location.
  (a.output/'recovery-journal.json').write_text(json.dumps(dict(summary,new_actors=[]),indent=2))
  summary['preparation_relocation']=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=transform))
  n.wait(lambda:n.fresh() and n.stopped() and n.values.get('/planning/current_link')==a.link and 'pose' in n.values and math.dist(n.values['pose'][:2],xyz[:2])<.4,15.)
  n.reset()
  full=yaml.safe_load(saved.read_text());prepared_route=mission_suffix(planner,full.get('start_link_id') or 'A219BS010380',full['waypoint_node_ids'],a.link)
  n.set_parameters_remote(dict(route_start_link_id=a.link,route_node_ids=prepared_route['remaining_waypoints'],cruise_speed_kmh=a.approach_speed_kmh,sharp_curve_speed_kmh=min(30.,a.approach_speed_kmh),drive_duration_sec=a.seconds+30.,enable_drive=False))
  n.spin(.8);n.reset()
  n.wait(lambda:n.fresh() and n.stopped() and n.values.get('/planning/current_link')==a.link and n.values.get('/planning/fsm_state')=='READY' and n.values.get('/planning/avoidance_debug',{}).get('state') in ('NORMAL','DETECT') and n.values.get('/planning/map_match_status')=='MATCHED' and n.values.get('/planning/local_path',{}).get('points',0)>2 and n.values.get('/planning/s4_sensor_state',{}).get('ready'),30.)
  if summary['s4_parameters'].get('static_avoidance_method')=='lane_candidates':
   n.wait(lambda:n.values.get('/planning/avoidance_debug',{}).get('lane_candidate_bank_ready'),15.)
  summary['candidate_preparation_before_spawn']=n.snapshot()
  summary['candidate_prepared_ros_ns']=n.get_clock().now().nanoseconds
 identifier='s4_generalized_'+str(time.time_ns());initial_ids={x['actor_info']['id']['value'] for x in inventory.get('states',[])};assert identifier not in initial_ids
 spawn=None
 if obstacle_transform is not None:
  summary['spawn_requested_ros_ns']=n.get_clock().now().nanoseconds
  spawn=call('actor.Actor','SpawnVehicle',dict(spawn_info=dict(actor_info=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE'),transform=obstacle_transform,model_name='2014_Kia_K7',label='independent S4 validation'),velocity=0.,pause=True))
  summary['spawn']=spawn
 # The service response contract is inspected before any cleanup ID is used.
 if spawn is None:pass
 elif 'actor_info' in spawn:new_actor=spawn['actor_info']
 elif 'id' in spawn:new_actor=spawn
 elif spawn.get('status')=='STATUS_CODE_SUCCESS' and spawn.get('custom_message')==identifier:
  new_actor=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE')
 else:raise RuntimeError('spawn response did not identify new actor: '+str(spawn))
 if new_actor is not None:
  assert new_actor['id']['value'] not in initial_ids
  assert call('actor.Actor','GetActorState',new_actor)['actor_info']==new_actor
  spawned_actors.append(new_actor)
 summary['additional_obstacles']=[]
 for number,specification in enumerate(a.additional_obstacle):
  key,station,yaw=specification.split(':');station=float(station);yaw=float(yaw)
  if not math.isfinite(station) or not math.isfinite(yaw):raise RuntimeError('finite additional actor station/yaw required')
  position,direction=sample(planner.links_by_id[key]['points'],station)
  target=dict(location=dict(x=position[0],y=position[1],z=position[2]+.35),
   rotation=dict(z=math.degrees(math.atan2(direction[1],direction[0]))+yaw))
  locations=[x['transform']['location'] for x in inventory['states']]
  locations+=[call('actor.Actor','GetActorState',x)['transform']['location'] for x in spawned_actors]
  if any(math.hypot(q['x']-position[0],q['y']-position[1])<10. for q in locations):raise RuntimeError('additional actor overlaps existing scene')
  identifier='s4_additional_'+str(time.time_ns());actor=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE')
  reply=call('actor.Actor','SpawnVehicle',dict(spawn_info=dict(actor_info=actor,transform=target,
    model_name='2014_Kia_K7',label='independent next-lane obstacle'),velocity=0.,pause=True))
  if reply.get('status')!='STATUS_CODE_SUCCESS':raise RuntimeError('additional actor spawn failed: '+str(reply))
  spawned_actors.append(actor);actual=call('actor.Actor','GetActorState',actor)
  summary['additional_obstacles'].append(dict(specification=specification,state=actual))
 summary['ego_before']=ego;summary['target']=transform;summary['obstacle_target']=obstacle_transform
 (a.output/'recovery-journal.json').write_text(json.dumps(dict(summary,new_actor=new_actor,new_actors=spawned_actors),indent=2))
 if not a.prepare_before_spawn:
  summary['relocation']=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=transform))
 n.wait(lambda:n.fresh() and n.stopped() and n.values.get('/planning/current_link')==a.link and 'pose' in n.values and math.dist(n.values['pose'][:2],xyz[:2])<.4,15.)
 n.reset()
 full=yaml.safe_load(saved.read_text());route=mission_suffix(planner,full.get('start_link_id') or 'A219BS010380',full['waypoint_node_ids'],a.link)
 n.set_parameters_remote(dict(route_start_link_id=a.link,route_node_ids=route['remaining_waypoints'],cruise_speed_kmh=a.approach_speed_kmh,sharp_curve_speed_kmh=min(30.,a.approach_speed_kmh),drive_duration_sec=a.seconds+30.,enable_drive=False))
 n.spin(.8);n.reset()
 n.wait(lambda:n.fresh() and n.values.get('/planning/current_link')==a.link and n.values.get('/planning/fsm_state')=='READY' and n.values.get('/planning/avoidance_debug',{}).get('state') in ('NORMAL','DETECT') and n.values.get('/planning/map_match_status')=='MATCHED' and n.values.get('/planning/local_path',{}).get('points',0)>2 and n.values.get('/planning/s4_sensor_state',{}).get('ready') and not n.values['/planning/s4_sensor_state'].get('latched_reason'),30.)
 assert n.count_publishers('/ctrl_cmd')==1 and n.values['/collision_data']==0 and n.stopped()
 assert n.paths['global_path']==route['points'],'runtime path must be exact saved mission suffix'
 summary['preflight']=n.snapshot();summary['parameters_trial']=n.parameters()
 if new_actor is not None:summary['obstacle_state']=call('actor.Actor','GetActorState',new_actor)
 (a.output/'preflight.json').write_text(json.dumps(summary,indent=2))
 (a.output/'global-path.json').write_text(json.dumps(n.paths['global_path']))
 (a.output/'initial-lidar.json').write_text(json.dumps(n.solids))
 route_index=PolylineIndex(route['points'])
 if a.tracking_only_seconds:
  # SetVelocity changes initial velocity only; NPC AI can accelerate again.
  # The SDK specifies this dynamics speed limit in m/s.
  limit=call('actor.Actor','SetVehicleDynamicsSpeedLimit',dict(actor_info=new_actor,
              speed_limiter=True,speed_limit=a.actor_motion_kmh/3.6,reset=False))
  if limit.get('status')!='STATUS_CODE_SUCCESS':raise RuntimeError('NPC speed limit unavailable: '+str(limit))
  summary['npc_motion_setup']=[limit,call('actor.Actor','SetPause',dict(actor_info=new_actor,enable=False))]
 started=time.monotonic();summary['drive_started_ros_ns']=n.get_clock().now().nanoseconds
 phases=[];path_fingerprints={};seen_return=False;seen_avoid=False;sensor_wait_started=None;normal_since=None
 n.set_parameters_remote({'enable_drive':a.tracking_only_seconds is None})
 duration=a.tracking_only_seconds or a.seconds;moving_match_sources=set()
 while time.monotonic()-started<duration:
  n.spin(.05);row=n.record(started);rows.append(row);v=n.values;debug=v.get('/planning/avoidance_debug',{});stage=debug.get('stage');state=debug.get('state')
  if bag is not None and bag.poll() is not None:raise RuntimeError('rosbag recorder exited during trial')
  with (a.output/'samples.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
  if not phases or phases[-1]!=stage:phases.append(stage)
  for key,points in n.paths.items():
   if key in ('global_path','control_local_path') or not points:continue
   payload=json.dumps(points);fingerprint=hashlib.sha256(payload.encode()).hexdigest()
   if fingerprint not in path_fingerprints:path_fingerprints[fingerprint]=dict(kind=key,t=row['t'],stage=stage,points=points)
  if not n.fresh():raise RuntimeError('live topic freshness failed')
  if n.count_publishers('/ctrl_cmd')!=1:raise RuntimeError('duplicate/missing control publisher')
  if v['/collision_data']!=0:raise RuntimeError('MORAI collision')
  sensor=v['/planning/s4_sensor_state']
  if sensor.get('latched_reason') and not a.tracking_only_seconds:raise RuntimeError('latched safety stop: '+sensor['latched_reason'])
  if not sensor.get('ready') and v.get('/planning/target_speed')!=0.:raise RuntimeError('sensor unavailable during motion command')
  if a.tracking_only_seconds:
   if v['/ego_vehicle_status']['speed_kmh']>=.2 or v['/ctrl_cmd']['accel']!=0. or v['/ctrl_cmd']['brake']<.6:
    raise RuntimeError('tracking-only Ego must remain disarmed and physically stopped')
   truth=call('actor.Actor','GetActorState',new_actor);row['actor_truth_audit_only']=truth
   position=truth['transform']['location'];tracks=v.get('/planning/traffic_tracks',{}).get('tracks',[])
   near=[t for t in tracks if math.hypot(t['center_xy'][0]-position['x'],t['center_xy'][1]-position['y'])<5.]
   moving_match_sources.update((t['id'],t['observed_ns']) for t in near if t['motion']=='MOVING' and t['age_s']<=.6)
   continue
  if state=='SENSOR_HOLD' and sensor_wait_started is None:sensor_wait_started=time.monotonic()
  if state not in ('SENSOR_HOLD','VALIDATING') and sensor.get('ready') and v.get('/planning/target_speed',0.)>0.:
   sensor_wait_started=None
  if sensor_wait_started is not None and time.monotonic()-sensor_wait_started>8.:
   raise RuntimeError('sensor recovery did not stabilize within trial supervision window')
  if state=='BLOCKED':raise RuntimeError('planner rejected: '+debug.get('reason',''))
  if debug.get('boundary_check_mode')=='avoidance' and debug.get('boundary_contact_reason'):
   raise RuntimeError('measured boundary contact: '+debug['boundary_contact_reason'])
  if a.follow_distance_m is not None:
   summary['followed_distance_m']=route_index.project(v['pose'][:2])-a.start
   if v.get('/control/path_error',0.)>1.5:raise RuntimeError('normal tracking error exceeds 1.5m')
   if summary['followed_distance_m']>=a.follow_distance_m:
    summary['success']=True;break
  if stage=='RETURN':seen_return=True
  if stage in ('AVOID_ENTRY','AVOID_HOLD'):seen_avoid=True
  resumed=(seen_avoid and stage=='NORMAL' and (debug.get('return_commits',0)>0 or debug.get('natural_rejoins',0)>0)
   and v.get('/planning/fsm_state')=='NORMAL_DRIVE' and v.get('/planning/target_speed',0.)>0.
   and v.get('/ego_vehicle_status',{}).get('speed_kmh',0.)>1. and sensor.get('ready'))
  normal_since=(normal_since or time.monotonic()) if resumed else None
  if normal_since is not None and time.monotonic()-normal_since>=1.:
   summary['normal_resumed_s']=time.monotonic()-normal_since;summary['success']=True;break
 else:
  if a.tracking_only_seconds:
   first=summary['obstacle_state']['transform']['location'];last=rows[-1]['actor_truth_audit_only']['transform']['location']
   distance=math.hypot(last['x']-first['x'],last['y']-first['y'])
   summary['tracking_measurement']=dict(npc_displacement_m=distance,moving_match_frames=len(moving_match_sources))
   summary['success']=distance>1. and len(moving_match_sources)>=5
   if not summary['success']:raise RuntimeError('moving NPC tracking did not meet measurement criterion')
  else:raise RuntimeError('avoid/return timeout')
 summary['phases']=phases;summary['candidate_and_selected_paths']=path_fingerprints
except BaseException as error:
 summary['error']=str(error) or type(error).__name__
finally:
 try:summary['stop_confirmed']=n.stop()
 except Exception as error:summary['stop_error']=str(error)
 summary['before_restore']=n.snapshot();summary['candidate_and_selected_paths']=path_fingerprints
 if rows:
  summary['peak_speed_kmh']=max(r['values'].get('/ego_vehicle_status',{}).get('speed_kmh',0.) for r in rows)
  summary['max_control_path_error_m']=max(r['values'].get('/control/path_error',0.) for r in rows)
  summary['phases']=[v for i,v in enumerate([r['values'].get('/planning/avoidance_debug',{}).get('stage') for r in rows]) if i==0 or v!=rows[i-1]['values'].get('/planning/avoidance_debug',{}).get('stage')]
 try:
  if not summary.get('stop_confirmed'):raise RuntimeError('restoration requires confirmed stationary Ego')
  summary['trial_actors_cleanup']=[]
  for actor in spawned_actors:
   actual=call('actor.Actor','GetActorState',actor);reply=call('actor.Actor','DestroyActor',actor)
   summary['trial_actors_cleanup'].append(dict(state=actual,destroy=reply))
  if ego is not None:
   desired=ego['transform'];target=json.loads(json.dumps(desired))
   origin=json.load(open('/workspace/local_data/kcity_mgeo/global_info.json'))['local_origin_in_global']
   lon,lat=Transformer.from_crs(32652,4326,always_xy=True).transform(desired['location']['x']+origin[0],desired['location']['y']+origin[1])
   target['rotation']['z']+=math.degrees(grid_convergence(Proj(32652),lon,lat))
   restoration=[]
   for attempt in range(3):
    reply=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=target));n.spin(.8)
    actual=call('actor.Actor','GetActorState',ego['actor_info'])
    error=math.degrees(math.atan2(math.sin(math.radians(desired['rotation']['z']-actual['transform']['rotation']['z'])),math.cos(math.radians(desired['rotation']['z']-actual['transform']['rotation']['z']))))
    restoration.append(dict(reply=reply,yaw_error_deg=error,actual=actual))
    actual_location=actual['transform']['location']
    position_error=math.hypot(actual_location['x']-desired['location']['x'],actual_location['y']-desired['location']['y'])
    if abs(error)<.02 and position_error<.05:break
    target['rotation']['z']+=error
   if abs(error)>=.02 or position_error>=.05:raise RuntimeError('Ego pose restoration not confirmed')
   summary['restore_ego']=restoration
  n.spin(3.)
  if original is not None and mission is not None:
   n.reset();n.set_parameters_remote(dict(mission,enable_drive=False,cruise_speed_kmh=original['cruise_speed_kmh'],sharp_curve_speed_kmh=original['sharp_curve_speed_kmh'],drive_duration_sec=original['drive_duration_sec']));n.spin(.8);n.reset()
  summary['final_stop_confirmed']=n.stop()
  n.wait(n.fresh);summary['final_parameters']=n.parameters();summary['final']=n.snapshot();summary['actors_after']=call('actor.Actor','GetAllActorsState',dict(vehicle=True,pedestrian=True,obstacle=True));summary['restored']={x['actor_info']['id']['value'] for x in summary['actors_after']['states']}==initial_ids
  after={x['actor_info']['id']['value']:x for x in summary['actors_after']['states']}
  errors={}
  for actor in inventory['states']:
   key=actor['actor_info']['id']['value'];before=actor['transform'];current=after[key]['transform']
   xy=math.hypot(before['location']['x']-current['location']['x'],before['location']['y']-current['location']['y'])
   yaw=abs(math.degrees(math.atan2(math.sin(math.radians(before['rotation']['z']-current['rotation']['z'])),math.cos(math.radians(before['rotation']['z']-current['rotation']['z'])))))
   errors[key]=dict(position_xy_m=xy,yaw_deg=yaw)
  summary['restoration_errors']=errors
  summary['restored']=summary['restored'] and summary['final_stop_confirmed'] and all(v['position_xy_m']<.05 and v['yaw_deg']<.02 for v in errors.values())
 except Exception as error:summary['restore_error']=str(error)
 summary['saved_route_unchanged']=saved_hash==hashlib.sha256(saved.read_bytes()).hexdigest()
 if bag is not None:
  if bag.poll() is None:
   os.killpg(bag.pid,signal.SIGINT)
   try:bag.wait(timeout=15.)
   except subprocess.TimeoutExpired:
    bag.terminate();bag.wait(timeout=5.)
  summary['rosbag_exit_code']=bag.returncode
  summary['rosbag_metadata_present']=(a.output/'bag'/'metadata.yaml').exists()
 if bag_log is not None:bag_log.close()
 (a.output/'summary.json').write_text(json.dumps(summary,indent=2));(a.output/'samples.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
 print(json.dumps({k:v for k,v in summary.items() if k not in ('actors_before','actors_after','preflight','candidate_and_selected_paths','before_restore','final','ego_before','obstacle_state')}),flush=True)
 n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if summary.get('success') and summary.get('restored') else 1)
