#!/usr/bin/env python3
"""Apply only the verified forward suffix of a saved MORAI route, while parked."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from morai_ros2_msgs.msg import EgoVehicleStatus, CollisionData
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from edgenode_planning.dijkstra_planner import DijkstraPlanner
from edgenode_planning.route_resume import mission_suffix


def remaining_route(planner, saved_start, waypoints, current):
    result=mission_suffix(planner,saved_start,waypoints,current)
    return result['remaining_waypoints'],result


def active_mission_matches(full_points,current_suffix_points,active_points):
    """Accept the fixed saved mission, including its already travelled prefix.

    Reapplying identical parameters intentionally preserves protected progress.
    A cached upstream start is valid; a future start or altered connector is not.
    """
    offset=len(full_points)-len(active_points)
    current_offset=len(full_points)-len(current_suffix_points)
    return bool(active_points) and 0<=offset<=current_offset and active_points==full_points[offset:]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',default='/workspace/local_data/kcity_routes/kcity_demo_route.yaml')
    parser.add_argument('--saved-start-link',required=True,help='original start link of the saved mission')
    parser.add_argument('--start',action='store_true',help='reset while parked, then enable only after readiness checks')
    args=parser.parse_args()
    saved=yaml.safe_load(Path(args.route).read_text())['waypoint_node_ids']
    rclpy.init();node=Node('s4_morai_route_prepare');latest={};received={}
    def receive(key,msg):
        latest[key]=msg;received[key]=time.monotonic()
    subscriptions=[node.create_subscription(kind,topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data)
        for key,kind,topic in [('link',String,'/planning/current_link'),
                              ('ego',EgoVehicleStatus,'/ego_vehicle_status'),
                              ('collision',CollisionData,'/collision_data'),
                              ('sensor',String,'/planning/s4_sensor_state'),
                              ('fsm',String,'/planning/fsm_state'),
                              ('global',NavPath,'/planning/global_path')]]
    def call(client,request):
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('service unavailable')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=4.)
        if not future.done() or future.result() is None:raise RuntimeError('service timeout')
        return future.result()
    try:
        end=time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        getter=node.create_client(GetParameters,'/planning_node/get_parameters')
        names=['enable_drive','mgeo_dir']
        params=dict(zip(names,map(parameter_value_to_python,
            call(getter,GetParameters.Request(names=names)).values)))
        if params['enable_drive']:raise RuntimeError('disable drive before preparing route')
        if any(time.monotonic()-received.get(k,0)>.6 for k in ('link','ego','collision')):
            raise RuntimeError('fresh current link and Ego status required')
        if latest['collision'].collision_object:raise RuntimeError('active collision; keep drive disabled')
        ego=latest['ego']
        if math.hypot(ego.velocity.x,ego.velocity.y,ego.velocity.z)*3.6>=.2:
            raise RuntimeError('vehicle must be stationary')
        planner=DijkstraPlanner(params['mgeo_dir'])
        deadline=time.monotonic()+3.
        while True:
            current=latest['link'].data
            try:
                remaining,result=remaining_route(planner,args.saved_start_link,saved,current)
                break
            except ValueError as error:
                # A junction can briefly match a neighboring connector. Keep
                # parked and await a real mission link; never invent a route.
                if str(error)!='CURRENT_LINK_OUTSIDE_SAVED_ROUTE' or time.monotonic()>=deadline:raise
                rclpy.spin_once(node,timeout_sec=.05)
        full_points=planner.plan_route(args.saved_start_link,saved)['points']
        setter=node.create_client(SetParametersAtomically,'/planning_node/set_parameters_atomically')
        response=call(setter,SetParametersAtomically.Request(parameters=[
            Parameter('enable_drive',value=False).to_parameter_msg(),
            Parameter('route_start_link_id',value=args.saved_start_link).to_parameter_msg(),
            Parameter('route_node_ids',Parameter.Type.STRING_ARRAY,saved).to_parameter_msg()])).result
        if not response.successful:raise RuntimeError(response.reason)
        applied=time.monotonic()
        if args.start:
            reset=call(node.create_client(Trigger,'/planning/reset_avoidance'),Trigger.Request())
            if not reset.success:raise RuntimeError(reset.message)
            end=time.monotonic()+5.;ready=False
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05)
                if any(received.get(k,0)<=applied or time.monotonic()-received.get(k,0)>.6
                       for k in ('sensor','fsm','ego','collision')):continue
                if latest['collision'].collision_object:raise RuntimeError('active collision; keep drive disabled')
                sensor=json.loads(latest['sensor'].data)
                if not (sensor.get('ready') and not sensor.get('recovery_pending')
                        and not sensor.get('latched_reason') and latest['fsm'].data=='READY'):continue
                if received.get('global',0)<=applied:continue
                points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in latest['global'].poses]
                if not active_mission_matches(full_points,result['points'],points):continue
                ego=latest['ego']
                if math.hypot(ego.velocity.x,ego.velocity.y,ego.velocity.z)*3.6>=.2:continue
                if node.count_publishers('/ctrl_cmd')!=1:raise RuntimeError('exactly one Control publisher required')
                ready=True;break
            if not ready:raise RuntimeError('route/sensor/FSM not ready; drive remains disabled')
            response=call(setter,SetParametersAtomically.Request(parameters=[
                Parameter('enable_drive',value=True).to_parameter_msg()])).result
            if not response.successful:raise RuntimeError(response.reason)
        print(json.dumps(dict(saved_waypoints=len(saved),remaining_waypoints=remaining,
            current_link=current,exact_saved_suffix=True,link_ids=result['link_ids'],enable_drive=args.start)))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
