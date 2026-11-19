"""C test Ego motion is A NORMAL-only; reuse B's serialized timer fairness."""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from edgenode_normal_experiment.normal_node import NormalPlanningNode
from edgenode_planning.planning_node import PlanningNode
from edgenode_static20_experiment.fair_schedule import run_with_gap
class TrackingNormalNode(NormalPlanningNode):
    def __init__(self):
        super().__init__()
        self._c_plan_timer=next(t for t in self.timers if t.callback==self.plan)
        self._c_period=self.get_parameter('planning_period_sec').value
    def plan(self):return run_with_gap(self._c_plan_timer,self._c_period,lambda:PlanningNode.plan(self))
def main(args=None):
    rclpy.init(args=args);node=TrackingNormalNode();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
