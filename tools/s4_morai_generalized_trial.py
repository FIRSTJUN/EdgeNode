"""Reuse the checkpoint's stopped, isolated MORAI trial and restore its state.

Run from the sourced Dev Container with the existing stack. No nodes or Docker
containers are launched. Only the temporary actor is removed after the trial.
"""
import argparse,hashlib,json,math,signal,sys,time,subprocess,os
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
from edgenode_planning.lane_geometry import sample
from edgenode_planning.route_resume import mission_suffix
import yaml
from pyproj import Proj,Transformer
from edgenode_localization.map_heading import grid_convergence
p=argparse.ArgumentParser();p.add_argument('--link',required=True);p.add_argument('--start',type=float,required=True);p.add_argument('--obstacle',type=float,required=True);p.add_argument('--obstacle-yaw-deg',type=float,default=0.);p.add_argument('--output',type=Path,required=True)
p.add_argument('--approach-speed-kmh',type=float,default=10.)
p.add_argument('--seconds',type=float,default=100.)
p.add_argument('--record-bag',action='store_true')
a=p.parse_args()
if not all(math.isfinite(v) for v in (a.start,a.obstacle,a.obstacle_yaw_deg)):p.error('finite station and yaw required')
if not all(math.isfinite(v) and v>0 for v in (a.approach_speed_kmh,a.seconds)):p.error('positive finite speed and duration required')
a.output.mkdir(exist_ok=False)
class Recorder(Monitor):
 def __init__(self):
  super().__init__();self.paths={};self.solids=[]
  self.create_subscription(Float32,'/localization/grid_convergence_deg',lambda m:self.receive('/localization/grid_convergence_deg',m),qos_profile_sensor_data)
  self.create_subscription(String,'/perception/lidar_diagnostics',lambda m:self.receive('/perception/lidar_diagnostics',m),qos_profile_sensor_data)
  for key in ('global_path','avoidance_selected_path','avoidance_return_path'):
   self.create_subscription(NavPath,'/planning/'+key,lambda m,k=key:self.path(k,m),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/localization/odometry',self.odom,qos_profile_sensor_data)
  self.create_subscription(PoseArray,'/planning/obstacles',self.solid,qos_profile_sensor_data)
 def receive(self,topic,m):
  super().receive(topic,m)
  if topic=='/ego_vehicle_status':self.values[topic]['front_steer_deg']=m.front_steer_angle
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
 def record(self,start):return dict(t=time.monotonic()-start,solids=self.solids,**self.snapshot())
def interrupted(*args):raise KeyboardInterrupt()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Recorder();summary={};rows=[];path_fingerprints={};new_actor=None;ego=None;original=None;mission=None;bag=None;bag_log=None
saved=Path('/workspace/local_data/kcity_routes/kcity_demo_route.yaml');saved_hash=hashlib.sha256(saved.read_bytes()).hexdigest()
try:
 n.wait(n.fresh);assert n.stop() and n.count_publishers('/ctrl_cmd')==1
 original=n.parameters();names=['route_node_ids','route_start_link_id'];mission=dict(zip(names,map(parameter_value_to_python,n.call(n.getter,GetParameters.Request(names=names)).values)))
 summary['trial_spec']=dict(link=a.link,start_m=a.start,obstacle_m=a.obstacle,
  obstacle_yaw_deg=a.obstacle_yaw_deg,approach_speed_kmh=a.approach_speed_kmh,seconds=a.seconds)
 names=['clipped_footprints','footprint_tile_size_m','publish_debug_clouds','time_aligned_obstacles','roi_max_x','ground_z_threshold']
 summary['perception_parameters']=dict(zip(names,map(parameter_value_to_python,n.call(
  n.create_client(GetParameters,'/lidar_obstacle_node/get_parameters'),GetParameters.Request(names=names)).values)))
 names=['avoidance_deceleration_mps2','avoidance_memory_merge_extent_m','avoidance_avoid_speed_kmh','s4_control_horizon_m','morai_unmarked_junction_experiment']
 summary['s4_parameters']=dict(zip(names,map(parameter_value_to_python,n.call(n.getter,GetParameters.Request(names=names)).values)))
 summary['code_sha']=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
 summary['working_diff_sha256']=hashlib.sha256(subprocess.check_output(['git','diff'])).hexdigest()
 if a.record_bag:
  topics=['/ego_vehicle_status','/ctrl_cmd','/localization/odometry','/control/path_error',
   '/control/lookahead_distance','/collision_data','/lidar/points','/Imu','/gps',
   '/perception/lidar_obstacle_count','/perception/lidar_obstacle_centers',
   '/perception/lidar_diagnostics','/perception/lidar_status','/planning/obstacles',
   '/perception/lidar_points_map_xy','/perception/lidar_cluster_points_map_xy',
   '/planning/current_link','/planning/map_match_status','/planning/target_speed',
   '/planning/global_path','/planning/local_path','/planning/avoidance_debug',
   '/planning/s4_sensor_state','/planning/s4_distance_state','/planning/fsm_state',
   '/planning/drive_status','/planning/speed_mode','/localization/grid_convergence_deg']
  bag_log=(a.output/'rosbag-console.log').open('w')
  bag=subprocess.Popen(['ros2','bag','record','-o',str(a.output/'bag'),*topics],stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
  n.spin(2.)
  if bag.poll() is not None:raise RuntimeError('rosbag recorder failed to start')
 summary['original_parameters']=original;summary['original_mission']=mission
 inventory=call('actor.Actor','GetAllActorsState',dict(vehicle=True,pedestrian=True,obstacle=True));summary['actors_before']=inventory
 ego=call('actor.Actor','GetActorState',dict(id={'value':'Ego'},object_type='OBJECT_TYPE_VEHICLE'))
 planner=DijkstraPlanner('/workspace/local_data/kcity_mgeo');points=planner.links_by_id[a.link]['points'];xyz,tangent=sample(points,a.start)
 obstacle_xyz,obstacle_tangent=sample(points,a.obstacle)
 transform=dict(location=dict(x=xyz[0],y=xyz[1],z=xyz[2]+.35),rotation=dict(z=math.degrees(math.atan2(tangent[1],tangent[0]))))
 obstacle_transform=dict(location=dict(x=obstacle_xyz[0],y=obstacle_xyz[1],z=obstacle_xyz[2]+.35),rotation=dict(z=math.degrees(math.atan2(obstacle_tangent[1],obstacle_tangent[0]))))
 obstacle_transform['rotation']['z']+=a.obstacle_yaw_deg
 for actor in inventory.get('states',[]):
  if actor['actor_info']['id']['value']=='Ego':continue
  location=actor['transform']['location']
  for target in (transform,obstacle_transform):
   assert math.hypot(target['location']['x']-location['x'],target['location']['y']-location['y'])>10.,'trial overlaps an original actor'
 identifier='s4_generalized_'+str(time.time_ns());initial_ids={x['actor_info']['id']['value'] for x in inventory.get('states',[])};assert identifier not in initial_ids
 spawn=call('actor.Actor','SpawnVehicle',dict(spawn_info=dict(actor_info=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE'),transform=obstacle_transform,model_name='2014_Kia_K7',label='independent S4 validation'),velocity=0.,pause=True))
 summary['spawn']=spawn
 # The service response contract is inspected before any cleanup ID is used.
 if 'actor_info' in spawn:new_actor=spawn['actor_info']
 elif 'id' in spawn:new_actor=spawn
 elif spawn.get('status')=='STATUS_CODE_SUCCESS' and spawn.get('custom_message')==identifier:
  new_actor=dict(id={'value':identifier},object_type='OBJECT_TYPE_VEHICLE')
 else:raise RuntimeError('spawn response did not identify new actor: '+str(spawn))
 assert new_actor['id']['value'] not in initial_ids
 assert call('actor.Actor','GetActorState',new_actor)['actor_info']==new_actor
 summary['ego_before']=ego;summary['target']=transform;summary['obstacle_target']=obstacle_transform
 (a.output/'recovery-journal.json').write_text(json.dumps(dict(summary,new_actor=new_actor),indent=2))
 summary['relocation']=call('actor.Actor','SetTransform',dict(actor_info=ego['actor_info'],transform=transform))
 n.wait(lambda:n.fresh() and n.stopped() and n.values.get('/planning/current_link')==a.link and 'pose' in n.values and math.dist(n.values['pose'][:2],xyz[:2])<.4,15.)
 n.reset()
 full=yaml.safe_load(saved.read_text());route=mission_suffix(planner,full.get('start_link_id') or 'A219BS010380',full['waypoint_node_ids'],a.link)
 n.set_parameters_remote(dict(route_start_link_id=a.link,route_node_ids=route['remaining_waypoints'],cruise_speed_kmh=a.approach_speed_kmh,sharp_curve_speed_kmh=min(30.,a.approach_speed_kmh),drive_duration_sec=a.seconds+30.,enable_drive=False))
 n.spin(.8);n.reset()
 n.wait(lambda:n.fresh() and n.values.get('/planning/map_match_status')=='MATCHED' and n.values.get('/planning/local_path',{}).get('points',0)>2 and n.values.get('/planning/s4_sensor_state',{}).get('ready') and not n.values['/planning/s4_sensor_state'].get('latched_reason'),30.)
 assert n.count_publishers('/ctrl_cmd')==1 and n.values['/collision_data']==0 and n.stopped()
 assert n.paths['global_path']==route['points'],'runtime path must be exact saved mission suffix'
 summary['preflight']=n.snapshot();summary['parameters_trial']=n.parameters();summary['obstacle_state']=call('actor.Actor','GetActorState',new_actor)
 (a.output/'preflight.json').write_text(json.dumps(summary,indent=2))
 (a.output/'global-path.json').write_text(json.dumps(n.paths['global_path']))
 (a.output/'initial-lidar.json').write_text(json.dumps(n.solids))
 started=time.monotonic();phases=[];path_fingerprints={};seen_return=False;sensor_wait_started=None;normal_since=None;n.set_parameters_remote({'enable_drive':True})
 while time.monotonic()-started<a.seconds:
  n.spin(.05);row=n.record(started);rows.append(row);v=n.values;debug=v.get('/planning/avoidance_debug',{});stage=debug.get('stage');state=debug.get('state')
  if bag is not None and bag.poll() is not None:raise RuntimeError('rosbag recorder exited during trial')
  with (a.output/'samples.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
  if not phases or phases[-1]!=stage:phases.append(stage)
  for key,points in n.paths.items():
   if key=='global_path' or not points:continue
   payload=json.dumps(points);fingerprint=hashlib.sha256(payload.encode()).hexdigest()
   if fingerprint not in path_fingerprints:path_fingerprints[fingerprint]=dict(kind=key,t=row['t'],stage=stage,points=points)
  if not n.fresh():raise RuntimeError('live topic freshness failed')
  if n.count_publishers('/ctrl_cmd')!=1:raise RuntimeError('duplicate/missing control publisher')
  if v['/collision_data']!=0:raise RuntimeError('MORAI collision')
  sensor=v['/planning/s4_sensor_state']
  if sensor.get('latched_reason'):raise RuntimeError('latched safety stop: '+sensor['latched_reason'])
  if not sensor.get('ready') and v.get('/planning/target_speed')!=0.:raise RuntimeError('sensor unavailable during motion command')
  if state=='SENSOR_HOLD' and sensor_wait_started is None:sensor_wait_started=time.monotonic()
  if state not in ('SENSOR_HOLD','VALIDATING') and sensor.get('ready') and v.get('/planning/target_speed',0.)>0.:
   sensor_wait_started=None
  if sensor_wait_started is not None and time.monotonic()-sensor_wait_started>8.:
   raise RuntimeError('sensor recovery did not stabilize within trial supervision window')
  if state=='BLOCKED':raise RuntimeError('planner rejected: '+debug.get('reason',''))
  if debug.get('boundary_check_mode')=='avoidance' and debug.get('boundary_contact_reason'):
   raise RuntimeError('measured boundary contact: '+debug['boundary_contact_reason'])
  if stage=='RETURN':seen_return=True
  resumed=(seen_return and stage=='NORMAL' and debug.get('return_commits',0)>0
   and v.get('/planning/fsm_state')=='NORMAL_DRIVE' and v.get('/planning/target_speed',0.)>0.
   and v.get('/ego_vehicle_status',{}).get('speed_kmh',0.)>1. and sensor.get('ready'))
  normal_since=(normal_since or time.monotonic()) if resumed else None
  if normal_since is not None and time.monotonic()-normal_since>=1.:
   summary['normal_resumed_s']=time.monotonic()-normal_since;summary['success']=True;break
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
  if new_actor is not None:
   summary['obstacle_state_after']=call('actor.Actor','GetActorState',new_actor)
   summary['destroy_trial_actor']=call('actor.Actor','DestroyActor',new_actor)
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
