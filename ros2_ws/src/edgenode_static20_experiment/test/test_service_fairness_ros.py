"""A busy serialized timer must not starve real ROS parameter services."""
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import GetParameters
from edgenode_static20_experiment.fair_schedule import run_with_gap

def test_parameters_remain_acknowledged_during_repeated_plan_overruns():
    context=rclpy.context.Context();rclpy.init(context=context)
    server=Node('phase_b_fairness_server',context=context)
    client=Node('phase_b_fairness_client',context=context)
    executor=MultiThreadedExecutor(num_threads=2,context=context)
    executor.add_node(server);executor.add_node(client);calls=[]
    server.declare_parameter('retained_sensor_limit',.6)
    def expensive():
        calls.append(time.monotonic());time.sleep(.12)
    timer=server.create_timer(.05,lambda:run_with_gap(timer,.05,expensive))
    rpc=client.create_client(GetParameters,'/phase_b_fairness_server/get_parameters')
    try:
        assert rpc.wait_for_service(timeout_sec=2.)
        for i in range(6):
            previous=len(calls);deadline=time.monotonic()+1.
            while len(calls)==previous and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
            assert len(calls)>previous
            future=rpc.call_async(GetParameters.Request(names=['retained_sensor_limit']))
            executor.spin_until_future_complete(future,timeout_sec=1.)
            assert future.done() and future.result().values[0].double_value==.6
        assert len(calls)>=6
    finally:
        executor.shutdown();server.destroy_node();client.destroy_node();rclpy.shutdown(context=context)
