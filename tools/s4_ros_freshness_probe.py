#!/usr/bin/env python3
"""Disarmed real executor/DDS probes; synthetic inputs, memory outputs only.

The producer thread belongs to this test, never to production Planning.
No Control node, MORAI connection or deployment approval is created.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
import time

from s4_timing_audit import NodeCase, dump, load, make_case


def periodic(adapter,seconds):
    from rclpy.executors import SingleThreadedExecutor
    node=adapter.node;executor=SingleThreadedExecutor();executor.add_node(node)
    rows=[];arrivals=[];started=time.monotonic_ns()
    adapter.feed(.05)
    def observe():
        arrival=adapter.feed(.05,reset_guard=False)
        arrivals.append(arrival)
    def plan():
        before=time.monotonic_ns();source=node.sensor_guard.status['source_stamp_ns']
        node.plan();after=time.monotonic_ns()
        rows.append(dict(index=len(rows),start_monotonic_ns=before,end_monotonic_ns=after,
            wall_s=(after-before)/1e9,source_stamp_ns=source,
            source_age_at_end_s=(node.get_clock().now().nanoseconds-source)/1e9,
            state=node.avoidance_planner.state,reason=node.avoidance_planner.reason,
            local_points=len(node.local_path_pub.last.poses),target_speed=node.target_speed_pub.last.data))
    timer=node.create_timer(.05,plan);sensor=node.create_timer(.1,observe)
    try:
        while (time.monotonic_ns()-started)/1e9<seconds:
            executor.spin_once(timeout_sec=.02)
    finally:
        node.destroy_timer(timer);node.destroy_timer(sensor)
        executor.remove_node(node);executor.shutdown()
    for previous,current in zip(rows,rows[1:]):
        current['start_interval_s']=(current['start_monotonic_ns']-previous['start_monotonic_ns'])/1e9
    return dict(planning_period_s=.05,input_period_s=.1,input_age_on_receipt_s=.05,
        requested_duration_s=seconds,callbacks=rows,observations=arrivals,
        calls_exceeding_period=sum(row['wall_s']>.05 for row in rows),
        missed_50ms_start_slots=sum(max(0,round(row.get('start_interval_s',.05)/.05)-1) for row in rows),
        publications=adapter.events)


def queued(adapter,event):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    node=adapter.node;executor=SingleThreadedExecutor();executor.add_node(node)
    producer=Node('stage5c_probe_producer',namespace='/stage5c_disarmed')
    status_sub=next(s for s in node.subscriptions if s.msg_type is adapter.String)
    obstacle_sub=next(s for s in node.subscriptions if s.msg_type is adapter.PoseArray)
    pose_sub=next(s for s in node.subscriptions if s.msg_type is adapter.Odometry)
    publisher=producer.create_publisher(adapter.String,status_sub.topic_name,10)
    obstacles=producer.create_publisher(adapter.PoseArray,obstacle_sub.topic_name,10)
    poses=producer.create_publisher(adapter.Odometry,pose_sub.topic_name,10)
    # Wait for discovery before the timed callback. No vehicle publisher exists.
    deadline=time.monotonic()+3.
    while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
        executor.spin_once(timeout_sec=.02)
    if publisher.get_subscription_count()==0:raise RuntimeError('isolated DDS discovery failed')
    events=[];original=status_sub.callback
    def received(msg):
        events.append(dict(event='status_callback',monotonic_ns=time.monotonic_ns(),data=json.loads(msg.data)))
        original(msg)
    status_sub.callback=received
    adapter.feed(.05);status=dict(node.sensor_guard.status);adapter.events.clear()
    def emit():
        time.sleep(.05)
        stamp=node.get_clock().now().nanoseconds
        status.update(status_seq=2,published_ns=stamp)
        if event=='error':status.update(state='PROCESSING_ERROR',fault_seq=1)
        elif event=='restart':status.update(session_id='d'*32,status_seq=1)
        elif event=='recovered_fault':status.update(fault_seq=1)
        else:
            pose=deepcopy(adapter.pose_message)
            pose.header.stamp.sec=stamp//10**9;pose.header.stamp.nanosec=stamp%10**9
            poses.publish(pose)
            msg=adapter.PoseArray();msg.header.frame_id='map'
            msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
            points=[p for group in adapter.fixture.groups for p in group]
            for x,y in points:
                p=adapter.Pose();p.position.x=float(x);p.position.y=float(y);msg.poses.append(p)
            obstacles.publish(msg)
            status.update(source_stamp_ns=stamp,frame_seq=2,frame_received_ns=stamp,
                obstacle_digest=adapter.m['sensor_readiness'].footprint_digest('map',stamp,points))
        publisher.publish(adapter.String(data=json.dumps(status)))
        events.append(dict(event='DDS_published',monotonic_ns=time.monotonic_ns(),data=dict(status)))
    thread=threading.Thread(target=emit)
    try:
        thread.start();start=time.monotonic_ns();node.plan();end=time.monotonic_ns();thread.join()
        end_state=node.avoidance_planner.state;end_reason=node.avoidance_planner.reason
        before_receive=list(adapter.events)
        deadline=time.monotonic()+.4
        def delivered():
            return (any(e['event']=='status_callback' for e in events) and
                (event!='new_observation' or
                 (node.latest_pose_stamp_ns==status['source_stamp_ns'] and
                  node.obstacle_stamp_ns==status['source_stamp_ns'])))
        while not delivered() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        # A newly matched observation is not a retroactive validation of the old
        # result. Faults latch and synchronously publish STOP on delivery.
        node.plan()
        return dict(event=event,planning_start_monotonic_ns=start,planning_end_monotonic_ns=end,
            state_at_compute_end=end_state,reason_at_compute_end=end_reason,
            publications_before_delivery=before_receive,events=events,
            state_after_delivery=node.avoidance_planner.state,reason_after_delivery=node.avoidance_planner.reason,
            status_seq_after_delivery=node.sensor_guard.status_seq,
            source_stamp_after_delivery=(node.sensor_guard.status or {}).get('source_stamp_ns'),
            final_local_points=len(node.local_path_pub.last.poses),final_target_speed=node.target_speed_pub.last.data,
            all_publications=adapter.events)
    finally:
        thread.join();status_sub.callback=original
        executor.remove_node(node);executor.shutdown();producer.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--periodic-trials',type=int,default=1)
    args=parser.parse_args();args.out.parent.mkdir(parents=True,exist_ok=True)
    loop,modules=load(args.source_root)
    result=dict(source_root=str(args.source_root),module_file=modules['planning_node'].__file__,
        production_enable_drive=False,memory_output_only=True,ros_executor='SingleThreadedExecutor',
        synth_input_only=True,queued=[],periodic_trials=[])
    for event in ('periodic',)*args.periodic_trials+('error','restart','recovered_fault','new_observation'):
        fixture=make_case(loop,modules,dict(name='executor_curve',radius=300.,side='right'))
        with tempfile.TemporaryDirectory(prefix='stage5c_ros_probe_') as directory:
            adapter=NodeCase(fixture,modules,directory)
            try:
                if event=='periodic':
                    trial=periodic(adapter,args.seconds)
                    result['periodic_trials'].append(trial)
                    result['periodic']=result['periodic_trials'][0]
                else:result['queued'].append(queued(adapter,event))
            finally:adapter.close()
        dump(args.out,result)


if __name__=='__main__':main()
