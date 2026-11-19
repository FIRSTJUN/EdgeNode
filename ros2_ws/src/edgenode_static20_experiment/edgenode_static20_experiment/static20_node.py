"""Explicit B node; v61 and completed A source modules remain unchanged."""
import json
import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from edgenode_planning.planning_node import PlanningNode
from edgenode_normal_experiment.provenance import implementation_sha256 as a_sha
from .static20_planner import Static20Planner
from .entry_search import EntrySearch
from .provenance import implementation_sha256,manifest
from .fair_schedule import run_with_gap

class Static20PlanningNode(PlanningNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('phase_b_motion_mode','STATIC_WITH_PHASE_A_NORMAL_GUARD')
        if self.get_parameter('phase_b_motion_mode').value!='STATIC_WITH_PHASE_A_NORMAL_GUARD':raise ValueError('unexpected B motion mode')
        if self.get_parameter('static_avoidance_method').value!='lane_candidates':raise ValueError('B requires native lane_candidates')
        old=self.avoidance_planner;old.close()
        budget=self._entry_search.budget_s;self._entry_search.close();self._entry_search=EntrySearch(budget_s=budget)
        self.avoidance_planner=Static20Planner(old.network,config=old.config,weights=old.weights,body=old.body,route_provider=lambda:self._route_result)
        self.declare_parameter('phase_b_normal_tracking_clearance_enabled',True)
        self._phase_b_normal_clearance=[]
        if self.get_parameter('phase_b_normal_tracking_clearance_enabled').value:
            from .normal_clearance import extend_normal_cache
            steering_bound=math.tan(self.get_parameter('avoidance_max_front_steer_normalized').value*
                self.get_parameter('avoidance_max_wheel_angle_rad').value)/old.body.wheelbase_m
            self._phase_b_normal_clearance=extend_normal_cache(old.network,old.body,
                steering_curvature_limit=steering_bound)
            self.get_logger().info('B normal reference clearance: '+json.dumps(self._phase_b_normal_clearance,sort_keys=True))
        self._phase_b_plan_timer=next(t for t in self.timers if t.callback==self.plan)
        self._phase_b_period_s=self.get_parameter('planning_period_sec').value
        self.phase_b_pub=self.create_publisher(String,'/planning/phase_b_runtime',1)
        self.create_timer(1.,self.publish_runtime)
    def plan(self):
        return run_with_gap(self._phase_b_plan_timer,self._phase_b_period_s,
            lambda:PlanningNode.plan(self))
    def publish_runtime(self):
        self.phase_b_pub.publish(String(data=json.dumps(dict(mode='STATIC_WITH_PHASE_A_NORMAL_GUARD',implementation_sha256=implementation_sha256(),files=manifest(),phase_a_guard_sha256=a_sha(),requested_avoidance_kmh=self.avoidance_planner.requested_avoidance_kmh,effective_avoidance_kmh=self.avoidance_planner.config.avoid_speed_kmh,normal_clearance=self._phase_b_normal_clearance),sort_keys=True)))

def main(args=None):
    rclpy.init(args=args);node=Static20PlanningNode();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
