"""Bounded MORAI-only test. Uses Planning's enable/lease gate, never CtrlCmd publishing."""
import argparse
import collections
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import parameter_value_to_python
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32, Float64
from nav_msgs.msg import Path as PathMessage, Odometry
from sensor_msgs.msg import Imu
from morai_ros2_msgs.msg import EgoVehicleStatus, CtrlCmd

OUT = Path(__file__).resolve().parents[2] / 'local_data' / 'kcity_trials'
RUNNER_LOCK_PATH = Path('/tmp/edgenode_kcity_runner.lock')
PROFILE = dict(cruise_speed_kmh=60., curve_speed_kmh=40., sharp_curve_speed_kmh=30.)
MAX_SPEED_KMH = 60.
OVERSPEED_MARGIN_KMH = 1.5
SAFETY_SETTINGS = dict(speed_long_preview_enabled=True,
    speed_plan_lateral_accel_mps2=1.5, speed_plan_speed_timeout_sec=.2,
    speed_long_preview_distance_m=80., speed_plan_deceleration_mps2=1.5,
    speed_plan_reaction_time_sec=.5, speed_plan_distance_margin_m=5.,
    speed_plan_acceleration_mps2=1., speed_plan_recovery_acceleration_mps2=2.,
    speed_plan_recovery_clear_sec=1.)


def require(condition, message):
    # Runtime gates must also work under python -O.
    if not condition: raise RuntimeError(message)


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--prepare',action='store_true',help='configure while drive stays disabled')
    mode.add_argument('--run',action='store_true',help='explicitly start a bounded MORAI test')
    parser.add_argument('--seconds',type=float,default=120.)
    parser.add_argument('--max-speed-kmh',type=float,default=20.,
        help='independent target ceiling, >0 and <=60; defaults to low-speed 20')
    parser.add_argument('--legacy-profile', action='store_true', help='retain 60/30/25 for isolated steering trials')
    parser.add_argument('--max-lateral-accel-mps2',type=float,
        help='optional sustained v*IMU yaw-rate abort threshold; does not change the speed ceiling')
    parser.add_argument('--lateral-budget-mps2',type=float,
        help='explicit MORAI planning-budget experiment, <=2.0; above 1.5 requires IMU abort guard <=3.0')
    parser.add_argument('--output-dir',type=Path,default=OUT)
    args=parser.parse_args(argv)
    if not math.isfinite(args.max_speed_kmh) or not 0 < args.max_speed_kmh <= MAX_SPEED_KMH:
        parser.error('--max-speed-kmh must be finite, >0 and <=60')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('--seconds must be finite and between 1 and 120')
    if args.max_lateral_accel_mps2 is not None and (
            not math.isfinite(args.max_lateral_accel_mps2) or not 0 < args.max_lateral_accel_mps2 <= 10):
        parser.error('--max-lateral-accel-mps2 must be finite, >0 and <=10')
    if args.lateral_budget_mps2 is not None:
        if not math.isfinite(args.lateral_budget_mps2) or not 0 < args.lateral_budget_mps2 <= 2.:
            parser.error('--lateral-budget-mps2 must be finite, >0 and <=2.0')
        if args.lateral_budget_mps2>1.5 and (args.max_lateral_accel_mps2 is None
                                            or args.max_lateral_accel_mps2>3.):
            parser.error('increased lateral budget requires an independent IMU abort guard <=3.0')
    return args


def preparation_parameters(max_speed, legacy=False, lateral_budget=None):
    require(math.isfinite(max_speed) and 0 < max_speed <= MAX_SPEED_KMH,'invalid speed ceiling')
    # Defaults preserve conservative budgets. A monitored lateral experiment
    # is explicit; braking, freshness and the independent speed ceiling stay.
    profile = dict(PROFILE)
    if legacy: profile.update(curve_speed_kmh=30., sharp_curve_speed_kmh=25.)
    values=dict(profile, speed_long_preview_enabled=True, enable_drive=False,
                max_drive_speed_kmh=float(max_speed),drive_duration_sec=120.)
    if lateral_budget is not None:
        require(not isinstance(lateral_budget,bool) and math.isfinite(lateral_budget)
                and 0<lateral_budget<=2.,'invalid lateral budget experiment')
        values['speed_plan_lateral_accel_mps2']=float(lateral_budget)
    return values


def verify_parameters(parameters,max_speed,legacy=False,lateral_budget=None):
    expected=preparation_parameters(max_speed,legacy,lateral_budget)
    expected.pop('drive_duration_sec')
    expected.update(local_path_length_m=20.)
    for name,value in expected.items():
        require(parameters.get(name)==value,'runtime setting mismatch: '+name)
    lower_bounds={'speed_long_preview_distance_m','speed_plan_reaction_time_sec',
                  'speed_plan_distance_margin_m','speed_plan_recovery_clear_sec'}
    for name,baseline in SAFETY_SETTINGS.items():
        if isinstance(baseline,bool):continue
        value=parameters.get(name)
        if name=='speed_plan_lateral_accel_mps2' and lateral_budget is not None:
            baseline=lateral_budget
        require(isinstance(value,(int,float)) and not isinstance(value,bool)
                and math.isfinite(value) and value>0,'invalid safety setting: '+name)
        require(value>=baseline if name in lower_bounds else value<=baseline,
                'safety setting outside conservative experiment bounds: '+name)


def lateral_accel_estimate(latest):
    speed=latest.get('/ego_vehicle_status',{}).get('speed',math.nan)
    rate=latest.get('/Imu',{}).get('gyro_z',math.nan)
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for v in (speed,rate)) or speed < 0:
        return None
    estimate=speed/3.6*rate
    return estimate if math.isfinite(estimate) else None


def immediate_faults(latest,times,now,max_speed,max_lateral=None):
    problems=[]
    for topic,limit in [('/ego_vehicle_status',.3),('/planning/target_speed',.5),
                        ('/ctrl_cmd',.3),('/control/path_error',.3),
                        ('/planning/long_speed_plan',.3),('/planning/map_match_status',.3)]:
        if now-times.get(topic,0)>limit:problems.append('stream missing: '+topic)
    speed=latest.get('/ego_vehicle_status',{}).get('speed',math.nan)
    target=latest.get('/planning/target_speed',math.nan)
    error=latest.get('/control/path_error',math.nan)
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (speed,target,error)):
        problems.append('nonfinite speed, target or path error')
    if target>max_speed+.001:problems.append('target exceeds selected ceiling')
    if abs(error)>1.5:problems.append('path error >1.5m')
    for topic in ('/localization/odometry', '/localization/status', '/planning/local_path'):
        limit = 1.5 if topic == '/localization/status' else .3
        if now-times.get(topic,0)>limit:problems.append('stream missing: '+topic)
    if latest.get('/localization/status') != 'EKF_READY':problems.append('localization not ready')
    for topic in ('/ego_vehicle_status', '/localization/odometry'):
        age = latest.get(topic, {}).get('age', math.inf)
        if not math.isfinite(age) or not -.05 <= age <= .3:
            problems.append('stale/future sensor stamp: '+topic)
    if max_lateral is not None:
        if (now-times.get('/Imu',0) > .2 or lateral_accel_estimate(latest) is None
                or not -.05 <= latest.get('/Imu',{}).get('age',math.inf) <= .2):
            problems.append('lateral guard requires fresh finite IMU and speed')
    return problems


def wait_for_graph(session,timeout=10.):
    """Keep the existing identity/single-publisher gates; allow DDS discovery time.

    A fresh observer may receive samples before remote node graph metadata.
    Never configure or arm while waiting, and still fail closed on timeout.
    """
    expected={'/planning/target_speed':'planning_node','/ctrl_cmd':'control_node',
              '/ego_vehicle_status':'MoraiSimNode'}
    deadline=time.monotonic()+timeout
    while True:
        nodes=session.node.get_node_names_and_namespaces()
        publishers={t:[p.node_name for p in session.node.get_publishers_info_by_topic(t)]for t in expected}
        if (all(nodes.count((name,'/')) == 1 for name in
                ('MoraiSimNode','planning_node','control_node','localization_node')) and
                all(publishers[t]==[name] for t,name in expected.items())):
            return dict(nodes=nodes,publishers=publishers)
        require(time.monotonic()<deadline,'ROS graph readiness timeout: '+
                json.dumps(dict(nodes=nodes,publishers=publishers)))
        session.spin(.1)


class Session:
    def __init__(self, name, output_dir=OUT):
        self.output_dir=Path(output_dir)
        self.output_dir.mkdir(parents=True,exist_ok=True)
        self.node = Node('codex_bounded_morai_test')
        self.latest = {}; self.times = {}; self.records = []
        self.log = (self.output_dir / (name + '.jsonl')).open('x')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topics = {t: String for t in ['/planning/drive_status', '/planning/fsm_state',
            '/planning/map_match_status', '/planning/current_link', '/planning/speed_mode',
            '/planning/long_speed_plan', '/localization/status']}
        topics.update({t: Float32 for t in ['/planning/target_speed', '/planning/recommended_speed',
            '/control/path_error', '/control/pure_pursuit_steering',
            '/control/lookahead_distance', '/control/target_left', '/control/target_forward',
            '/control/target_smoothing_offset',
            '/control/speed_pid_d_raw', '/control/speed_pid_d_filtered', '/control/speed_pid_effort',
            '/control/speed_pid_i',
            '/localization/gps_speed', '/planning/map_match_distance',
            '/planning/map_match_heading_diff']})
        topics.update({'/planning/local_path_progress_m': Float64,
            '/ego_vehicle_status': EgoVehicleStatus, '/ctrl_cmd': CtrlCmd,
            '/localization/odometry': Odometry, '/planning/local_path': PathMessage})
        topics['/Imu']=Imu
        self.subs = [self.node.create_subscription(typ, t, self.callback(t), q) for t,typ in topics.items()]
        self.set_client = self.node.create_client(SetParametersAtomically, '/planning_node/set_parameters_atomically')
        self.get_client = self.node.create_client(GetParameters, '/planning_node/get_parameters')
        self.activate_client = self.node.create_client(Trigger, '/route_editor/activate')

    def callback(self, topic):
        def receive(msg):
            now = time.monotonic()
            if isinstance(msg, EgoVehicleStatus):
                value = dict(speed=3.6*math.hypot(msg.velocity.x,msg.velocity.y,msg.velocity.z),
                    x=msg.position.x,y=msg.position.y,heading=msg.heading,
                    acceleration=[msg.acceleration.x,msg.acceleration.y,msg.acceleration.z],
                    angular_velocity=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                    age=self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
            elif isinstance(msg, Imu):
                value=dict(gyro_z=msg.angular_velocity.z,
                    linear_acceleration=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
                    frame=msg.header.frame_id,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                    age=self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
            elif isinstance(msg, CtrlCmd):
                value = dict(accel=msg.accel,brake=msg.brake,steer=msg.front_steer,
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            elif isinstance(msg, Odometry):
                q = msg.pose.pose.orientation
                value = dict(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                    speed_mps=msg.twist.twist.linear.x,
                    covariance=list(msg.pose.covariance),
                    age=self.node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)
            elif isinstance(msg, PathMessage):
                value = dict(frame=msg.header.frame_id,
                    points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
            elif topic == '/planning/long_speed_plan': value=json.loads(msg.data)
            else: value=msg.data
            self.latest[topic]=value; self.times[topic]=now
            row=dict(t=now,wall=time.time(),topic=topic,value=value)
            self.log.write(json.dumps(row)+'\n'); self.records.append(row)
        return receive

    def spin(self, duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.02)

    def call(self, client, request, timeout=5.):
        if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('service unavailable: '+client.srv_name)
        future=client.call_async(request);end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.02)
        if not future.done():raise RuntimeError('service timeout: '+client.srv_name)
        return future.result()

    def set(self, **values):
        response=self.call(self.set_client,SetParametersAtomically.Request(parameters=[
            Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]))
        if not response.result.successful:raise RuntimeError('parameter rejected: '+response.result.reason)
        print('ATOMIC_PARAMETERS',json.dumps(values),flush=True)

    def get(self):
        names=['enable_drive','max_drive_speed_kmh','drive_duration_sec','cruise_speed_kmh',
            'curve_speed_kmh','sharp_curve_speed_kmh','speed_long_preview_enabled',
            'local_path_length_m','route_node_ids','global_path_frame_id','mgeo_dir']
        names=list(dict.fromkeys(names+list(SAFETY_SETTINGS)))
        values=self.call(self.get_client,GetParameters.Request(names=names)).values
        return dict(zip(names,map(parameter_value_to_python,values)))

    def control_settings(self):
        names=['control_rate_hz','lookahead_min_m','lookahead_max_m',
            'lookahead_preview_time_sec','wheelbase_m','max_wheel_angle_rad',
            'steering_sign','max_front_steer_normalized',
            'steering_filter_time_constant_sec','steering_max_rate_per_sec',
            'target_smoothing_sigma_m','target_smoothing_max_deviation_m',
            'target_smoothing_reversal_hold_m',
            'speed_kp','speed_ki','speed_kd','speed_derivative_filter_time_constant_sec',
            'speed_pid_anti_windup_enabled','speed_integral_limit']
        client=self.node.create_client(GetParameters,'/control_node/get_parameters')
        values=self.call(client,GetParameters.Request(names=names)).values
        return dict(zip(names,map(parameter_value_to_python,values)))

    def capture_global_route(self):
        """Inspect one full path while disarmed; avoid a heavy run-time subscription."""
        captured=[]
        def receive(msg):
            if captured:return
            points=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in msg.poses]
            encoded=json.dumps(points).encode()
            digest=hashlib.sha256(encoded).hexdigest()
            path_file=self.output_dir/('global_'+digest+'.json')
            if not path_file.exists():path_file.write_bytes(encoded)
            captured.append(dict(points=len(points),frame=msg.header.frame_id,
                sha256=digest,points_file=str(path_file),
                length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))))
        sub=self.node.create_subscription(PathMessage,'/planning/global_path',receive,1)
        try:
            end=time.monotonic()+3.
            while not captured and time.monotonic()<end:self.spin(.02)
            require(captured and captured[0]['points']>=2 and captured[0]['frame']=='map'
                    and math.isfinite(captured[0]['length_m']) and captured[0]['length_m']>0,
                    'valid map-frame global route not received')
            return captured[0]
        finally:self.node.destroy_subscription(sub)

    def reanchor_stopped(self):
        client=self.node.create_client(Trigger,'/planning_node/reset_local_progress')
        try:
            if not client.wait_for_service(timeout_sec=.3):return dict(available=False)
            response=self.call(client,Trigger.Request())
            require(response.success,'stationary progress reset refused: '+response.message)
            return dict(available=True,projection=json.loads(response.message))
        finally:self.node.destroy_client(client)

    def brief(self):
        d=self.latest
        return dict(speed=d.get('/ego_vehicle_status',{}).get('speed'),
            target=d.get('/planning/target_speed'),error=d.get('/control/path_error'),
            progress=d.get('/planning/local_path_progress_m'),fsm=d.get('/planning/fsm_state'),
            drive=d.get('/planning/drive_status'),long=d.get('/planning/long_speed_plan',{}).get('status'),
            long_age=d.get('/planning/long_speed_plan',{}).get('speed_age_sec'))

    def ready(self):
        d=self.latest;now=time.monotonic()
        return (d.get('/planning/fsm_state')=='READY' and d.get('/planning/drive_status')=='DISABLED'
            and d.get('/planning/map_match_status')=='MATCHED'
            and d.get('/localization/status')=='EKF_READY'
            and now-self.times.get('/localization/status',0)<1.5
            and now-self.times.get('/localization/odometry',0)<.3
            and now-self.times.get('/planning/local_path',0)<.3
            and d.get('/planning/local_path',{}).get('frame')=='map'
            and len(d.get('/planning/local_path',{}).get('points',[]))>=2
            and d.get('/planning/long_speed_plan',{}).get('status')=='READY'
            # Same 1m boundary as the active monitor. After stopping, the
            # monotonic local-path start may remain slightly ahead of the car;
            # the reported nearest-segment distance includes that along-track gap.
            and abs(d.get('/control/path_error',math.inf))<1.
            and d.get('/ego_vehicle_status',{}).get('speed',math.inf)<.3
            and d.get('/planning/target_speed')==0
            and d.get('/ctrl_cmd',{}).get('brake',0)>=.6
            and all(now-self.times.get(t,0)<.3 for t in ['/ego_vehicle_status','/planning/target_speed',
                '/ctrl_cmd','/control/path_error','/planning/long_speed_plan',
                '/planning/drive_status','/planning/fsm_state','/planning/map_match_status']))

    def wait_ready(self, timeout):
        start=time.monotonic();clear=None;printed=start-5
        while time.monotonic()-start<timeout:
            self.spin(.05);now=time.monotonic()
            if self.ready():
                if clear is None:clear=now
                if now-clear>4:return
            else:clear=None
            if now-printed>5:print('WAIT',json.dumps(self.brief()),flush=True);printed=now
        raise RuntimeError('stable readiness not achieved: '+json.dumps(self.brief()))

    def stop(self):
        for _ in range(3):
            try:self.set(enable_drive=False);break
            except Exception as error:print('STOP_RETRY',str(error),flush=True)
        else:return False
        end=time.monotonic()+12;clear=None
        while time.monotonic()<end:
            self.spin(.05);now=time.monotonic();d=self.latest
            stopped=(d.get('/ego_vehicle_status',{}).get('speed',math.inf)<.3
                and d.get('/planning/drive_status')=='DISABLED'
                and d.get('/planning/target_speed')==0
                and d.get('/ctrl_cmd',{}).get('brake',0)>=.6
                and all(now-self.times.get(t,0)<.3 for t in
                    ['/ego_vehicle_status','/planning/target_speed','/planning/drive_status','/ctrl_cmd']))
            if stopped:
                if clear is None:clear=now
                if now-clear>1:return True
            else:clear=None
        return False


def main(argv=None):
    # Serialize prepare/run observers across output directories as well.
    # A second invocation cannot disarm or reconfigure an active trial.
    with RUNNER_LOCK_PATH.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _main(argv)


def _main(argv=None):
    args=parse_args(argv)
    rclpy.init()
    name=('prepare_' if args.prepare else 'drive_')+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
    s=Session(name,args.output_dir)
    result=dict(name=name,armed=False,selected_max_speed_kmh=args.max_speed_kmh,
        overspeed_stop_kmh=args.max_speed_kmh+OVERSPEED_MARGIN_KMH,
        lateral_guard_mps2=args.max_lateral_accel_mps2)
    result['selected_lateral_budget_mps2']=args.lateral_budget_mps2
    armed_at=None;success=False
    try:
        result['graph']=wait_for_graph(s)
        result['control_parameters']=s.control_settings()
        result['before_parameters']=s.get()
        if args.prepare:
            require(s.stop(),'cannot confirm stationary disabled vehicle before preparation')
            response=s.call(s.activate_client,Trigger.Request(),timeout=10.)
            require(response.success,response.message)
            print('ACTIVATE',response.message,flush=True)
            s.set(**preparation_parameters(args.max_speed_kmh,args.legacy_profile,args.lateral_budget_mps2))
            verify_parameters(s.get(),args.max_speed_kmh,args.legacy_profile,args.lateral_budget_mps2)
            result['global_route']=s.capture_global_route()
            result['stationary_reanchor']=s.reanchor_stopped()
            require(s.capture_global_route()['sha256']==result['global_route']['sha256'],
                    'global route changed during stationary reanchor')
            s.wait_ready(75.)
        else:
            verify_parameters(result['before_parameters'],args.max_speed_kmh,args.legacy_profile,args.lateral_budget_mps2)
            result['global_route']=s.capture_global_route()
            s.wait_ready(12.)
            if args.max_lateral_accel_mps2 is not None:
                require(not immediate_faults(s.latest,s.times,time.monotonic(),args.max_speed_kmh,
                                            args.max_lateral_accel_mps2),'lateral guard not ready')
            s.set(enable_drive=True,drive_duration_sec=min(args.seconds+2,122.))
            result['armed']=True;armed_at=time.monotonic();printed=armed_at-5;bad_since={}
            while time.monotonic()-armed_at<args.seconds:
                s.spin(.02);now=time.monotonic();d=s.latest
                # Lease and Control watchdogs provide independent fail-stop paths.
                faults=immediate_faults(d,s.times,now,args.max_speed_kmh,args.max_lateral_accel_mps2)
                if faults:raise RuntimeError('; '.join(faults))
                sustained={
                    'actual speed exceeds selected ceiling +1.5kmh':(
                        d.get('/ego_vehicle_status',{}).get('speed',0)>args.max_speed_kmh+OVERSPEED_MARGIN_KMH,.25),
                    'path error >1m':(abs(d.get('/control/path_error',0))>1.,.25),
                    'long planner unavailable':(d.get('/planning/long_speed_plan',{}).get('status')!='READY',.4),
                    'map match lost':(d.get('/planning/map_match_status')!='MATCHED',.25),
                }
                if args.max_lateral_accel_mps2 is not None:
                    lateral=lateral_accel_estimate(d)
                    sustained['lateral acceleration estimate exceeds selected guard']=(
                        lateral is None or abs(lateral)>args.max_lateral_accel_mps2,.15)
                for reason,(bad,limit) in sustained.items():
                    if bad:
                        bad_since.setdefault(reason,now)
                        if now-bad_since[reason]>=limit:raise RuntimeError(reason)
                    else:bad_since.pop(reason,None)
                if now-armed_at>1 and d.get('/planning/drive_status')!='ACTIVE':
                    raise RuntimeError('drive gate changed: '+str(d.get('/planning/drive_status')))
                if now-printed>=5:print('DRIVE',round(now-armed_at,1),json.dumps(s.brief()),flush=True);printed=now
            result['outcome']='bounded test completed'
        result.setdefault('outcome','stationary readiness verified')
        success=True
    except BaseException as error:
        result['outcome']='stopped: '+type(error).__name__+': '+str(error)
        print('ERROR',result['outcome'],flush=True)
    finally:
        try:result['stop_confirmed']=s.stop()
        except Exception as error:result.update(stop_confirmed=False,stop_error=str(error))
        try:result['end_parameters']=s.get()
        except Exception as error:result['parameter_read_error']=str(error)
        result['final']=s.brief()
        if armed_at is not None:result['armed_elapsed_sec']=time.monotonic()-armed_at
        ego=[r['value'] for r in s.records if r['topic']=='/ego_vehicle_status']
        result['max_speed_kmh']=max([v['speed'] for v in ego],default=0)
        result['max_path_error_m']=max([abs(r['value']) for r in s.records if r['topic']=='/control/path_error'],default=0)
        result['long_status_counts']=dict(collections.Counter(r['value']['status'] for r in s.records if r['topic']=='/planning/long_speed_plan'))
        result['records']=len(s.records)
        (s.output_dir/(name+'_result.json')).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
        print('RESULT',json.dumps(result,ensure_ascii=False),flush=True)
        s.log.close();s.node.destroy_node();rclpy.shutdown()
    return 0 if success and result['stop_confirmed'] else 1


if __name__=='__main__':
    def terminate(signum,frame):raise KeyboardInterrupt('termination requested')
    signal.signal(signal.SIGTERM,terminate)
    sys.exit(main())
