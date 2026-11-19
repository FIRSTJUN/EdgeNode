"""Explicit Phase A node; no edits or monkey patches to preserved v61 modules."""
import json
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from edgenode_planning.planning_node import PlanningNode
from .normal_safety import NormalSafetyPlanner
from .provenance import implementation_sha256,manifest

class DisabledSearch:
    active=False
    def cancel(self,*args,**kwargs):pass
    def close(self):pass
    def __call__(self,*args,**kwargs):raise RuntimeError('PHASE_A_SEARCH_MUST_NEVER_RUN')

class NormalPlanningNode(PlanningNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('phase_a_motion_mode','NORMAL_SAFETY_ONLY')
        if self.get_parameter('phase_a_motion_mode').value!='NORMAL_SAFETY_ONLY':raise ValueError('A must be NORMAL only')
        old=self.avoidance_planner;close=getattr(old,'close',None)
        if close is not None:close()
        self._entry_search.close();self._return_search.close()
        self._entry_search=DisabledSearch();self._return_search=DisabledSearch()
        self.avoidance_planner=NormalSafetyPlanner(old.network,config=old.config,weights=old.weights,body=old.body,route_provider=lambda:self._route_result)
        self.phase_a_pub=self.create_publisher(String,'/planning/phase_a_runtime',1)
        self.create_timer(1.,self.publish_phase_a_runtime)
    def publish_phase_a_runtime(self):
        self.phase_a_pub.publish(String(data=json.dumps(dict(mode='NORMAL_SAFETY_ONLY',lateral_search_enabled=False,
            implementation_sha256=implementation_sha256(),files=manifest()),sort_keys=True)))

def main(args=None):
    rclpy.init(args=args);node=NormalPlanningNode();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
