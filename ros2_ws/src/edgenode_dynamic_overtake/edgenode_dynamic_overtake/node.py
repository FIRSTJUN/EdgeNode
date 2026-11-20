"""D integration: independent rear evidence, preserved A/B/C source modules."""
import json
import threading
import time
from collections import deque
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from edgenode_static20_experiment.static20_node import Static20PlanningNode
from edgenode_static20_experiment.fair_schedule import run_with_gap
from edgenode_planning.planning_node import PlanningNode
from .deadline import should_defer,ValidationBudget
from .planner import DynamicPlanner
from .rear_sensing import decode_scene
from .provenance import implementation_sha256,manifest

class DynamicPlanningNode(Static20PlanningNode):
    def __init__(self):
        self._d_plan_costs={'normal':ValidationBudget(.27,.25),'active':ValidationBudget(.20,.22)};self._d_deadline_deferrals=0;self._d_deadline_state={}
        self.traffic={};self.rear={};self.rear_errors=0;self._d_input_lock=threading.RLock();self._d_drained=0
        super().__init__();old=self.avoidance_planner;old.close()
        self.declare_parameter('phase_d_lane_clearance_offsets_m',[0.,.2,.35,.5],ParameterDescriptor(read_only=True))
        self.avoidance_planner=DynamicPlanner(old.network,config=old.config,weights=old.weights,body=old.body,route_provider=lambda:self._route_result,traffic_provider=lambda:self.traffic,coverage_provider=self.coverage,input_admitter=self.drain_d_inputs,lane_clearance_offsets_m=self.get_parameter('phase_d_lane_clearance_offsets_m').value)
        self.d_input_group=MutuallyExclusiveCallbackGroup()
        self.d_tracks_sub=self.create_subscription(String,'/dynamic_tracking/tracks',self.tracks,1)
        self.d_rear_sub=self.create_subscription(String,'/dynamic_overtake/rear_scene',self.rear_scene,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.runtime_pub=self.create_publisher(String,'/planning/phase_d_runtime',1);self.create_timer(1.,self.runtime,callback_group=self.d_input_group)
    def plan(self):
        return run_with_gap(self._phase_b_plan_timer,self._phase_b_period_s,self.plan_with_deadline)
    def plan_with_deadline(self):
        PlanningNode.drain_atomic_vehicle_state(self);PlanningNode.drain_atomic_sensor(self);self.drain_d_inputs()
        now=self.get_clock().now().nanoseconds;planner=self.avoidance_planner
        phase='active' if getattr(planner,'dynamic',None) is not None else 'normal';costs=self._d_plan_costs[phase]
        dynamic=(getattr(planner,'dynamic',None) is not None or any(t.get('motion') in ('MOVING','UNKNOWN') for t in self.traffic.get('tracks',[])));follow_fallback=False
        if dynamic and planner.state!='BLOCKED' and self.get_parameter('enable_drive').value:
            budget=costs.estimate(time.monotonic());defer,remaining=should_defer(self,now,budget)
            self._d_deadline_state=dict(remaining_source_deadline_s=remaining,required_validation_budget_s=budget,deferred=defer,now_ns=now)
            if defer:
                self._d_deadline_deferrals+=1
                if phase=='active':return
                # A costly optional lateral admission must not starve the
                # current lane's fully checked normal following command.
                follow_fallback=True;planner.lateral_admission_allowed=False
                self._d_deadline_state['safe_follow_fallback']=True
        began=time.perf_counter()
        try:return PlanningNode.plan(self)
        finally:
            planner.lateral_admission_allowed=True
            if not follow_fallback:costs.record(time.monotonic(),time.perf_counter()-began)
    def drain_d_inputs(self):
        with self._d_input_lock:
            for name,callback in (('d_tracks_sub',self.tracks),('d_rear_sub',self.rear_scene)):
                sub=getattr(self,name,None)
                if sub is None:continue
                with sub.handle:taken=sub.handle.take_message(sub.msg_type,sub.raw)
                if taken is not None:self._d_drained+=1;callback(taken[0])
    def tracks(self,msg):
        with self._d_input_lock:
            try:
                value=json.loads(msg.data)
                if value.get('published_ns',0)>self.traffic.get('published_ns',0):self.traffic=value
            except (ValueError,TypeError):self.traffic={}
    def rear_scene(self,msg):
        with self._d_input_lock:
            try:
                value=json.loads(msg.data)
                if value.get('implementation_sha256')!=implementation_sha256():raise ValueError('rear worker fingerprint mismatch')
                if value.get('published_ns',0)>self.rear.get('evidence_published_ns',0):self.rear=decode_scene(value)
            except (ValueError,TypeError,KeyError,OSError):self.rear_errors+=1;self.rear={}
    def coverage(self):
        value=dict(self.rear);now=self.get_clock().now().nanoseconds;ns=value.get('evidence_published_ns');fresh=type(ns)is int and -.05<=(now-ns)/1e9<=.6
        value['rear_sensors']=[row for row in value.get('rear_sensors',[]) if -.05<=(now-row['ray'].source_ns)/1e9<=.6]
        aux=value.get('auxiliary_source_ns');aux_fresh=type(aux)is int and -.05<=(now-aux)/1e9<=.6
        value['auxiliary_ready']=bool(fresh and aux_fresh and value.get('auxiliary_ready') and any(row['name']=='d_roof_hdl64' for row in value['rear_sensors']))
        if not fresh:value['auxiliary_groups']=[]
        value.setdefault('auxiliary_groups',[]);return value
    def runtime(self):
        coverage=self.coverage();rear={k:v for k,v in coverage.items() if k not in ('rear_sensors','auxiliary_groups')}
        rear['sensors']=[{k:v for k,v in row.items() if k!='ray'}|dict(source_ns=row['ray'].source_ns) for row in coverage['rear_sensors']];rear['current_rear_footprints']=len(coverage['auxiliary_groups'])
        self.runtime_pub.publish(String(data=json.dumps(dict(mode='D_SOURCE_TIME_DYNAMIC_AND_PRESERVED_B_STATIC',implementation_sha256=implementation_sha256(),files=manifest(),rear_sensing=rear,rear_decode_errors=self.rear_errors,d_input_inbox_frames=self._d_drained,deadline_deferrals=self._d_deadline_deferrals,deadline_state=self._d_deadline_state,tracking_source_ns=self.traffic.get('source_stamp_ns'),dynamic_completed=self.avoidance_planner.dynamic_completed,collision_solids_never_removed=True,actor_api_used=False))))

def main(args=None):
    import faulthandler,signal
    faulthandler.register(signal.SIGUSR2,all_threads=True)
    rclpy.init(args=args);node=DynamicPlanningNode();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
