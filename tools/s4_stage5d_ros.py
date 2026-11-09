#!/usr/bin/env python3
"""Isolated DDS timing: real Planning/Control, all CtrlCmd outputs memory-only.

10 Hz synthetic LiDAR/pose producer, normal 20 Hz Planning and 30 Hz Control.
The nodes use separate processes and single-thread executors. No MORAI drive.
"""
import argparse
from copy import deepcopy
import json
import multiprocessing
import os
from pathlib import Path
import tempfile
import threading
import time

from s4_timing_audit import NodeCase, Sink, dump, load, make_case


def moment(node):
    return dict(ros_ns=node.get_clock().now().nanoseconds,monotonic_ns=time.monotonic_ns(),
                wall_unix_ns=time.time_ns())


class Commands:
    def __init__(self,node,events):self.node=node;self.events=events
    def publish(self,msg):
        self.events.append(dict(kind='control_execution',**moment(self.node),accel=msg.accel,
            brake=msg.brake,front_steer=msg.front_steer,velocity=msg.velocity))
        assert msg.accel==0. and msg.velocity==0.,'disarmed probe must never request motion'


class Forward:
    def __init__(self,memory,ros,node,events,name):
        self.memory=memory;self.ros=ros;self.node=node;self.events=events;self.name=name
    def publish(self,msg):
        stamp=msg.header.stamp if hasattr(msg,'header') else None
        published=moment(self.node)
        self.memory.publish(msg);self.ros.publish(msg)
        guard=self.node.sensor_guard
        self.events.append(dict(kind='planning_output',topic=self.name,**published,
            message_stamp_ns=stamp.sec*10**9+stamp.nanosec if stamp else None,
            source_stamp_ns=guard.matched[0]['source_stamp_ns'] if guard.matched else None,
            frame_seq=guard.matched[0]['frame_seq'] if guard.matched else None,
            points=len(msg.poses) if hasattr(msg,'poses') else None,value=getattr(msg,'data',None)))
    @property
    def last(self):return self.memory.last


def control_process(root,output,ready,stopping):
    """Separate interpreter, matching the ordinary two-executable deployment."""
    loop,_=load(Path(root))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from edgenode_control.control_node import ControlNode
    rclpy.init(args=['--ros-args','-r','__ns:=/stage5d_control_disarmed'])
    node=ControlNode();rows=[];error=None
    for name in list(node.__dict__):
        if name.endswith('_pub'):setattr(node,name,Sink(node,name,[]))
    node.ctrl_pub=Commands(node,rows)
    executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        for name,value in loop.production_values().items():
            if node.has_parameter(name):assert node.get_parameter(name).value==value,(name,value)
        for sub in node.subscriptions:
            original=sub.callback;topic=sub.topic_name
            def received(msg,original=original,topic=topic):
                header=getattr(msg,'header',None)
                row=dict(kind='control_receive',topic=topic,**moment(node),
                    source_stamp_ns=header.stamp.sec*10**9+header.stamp.nanosec if header else None)
                original(msg);row['callback_end_monotonic_ns']=time.monotonic_ns();rows.append(row)
            sub.callback=received
        ready.set()
        while not stopping.is_set():executor.spin_once(timeout_sec=.01)
    except Exception as exc:error=repr(exc)
    finally:
        ready.set();executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown()
        dump(Path(output),dict(rows=rows,error=error,ctrlcmd_memory_only=True))


def trial(loop,modules,duration,event,workload,delay_first_s=0.):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from morai_ros2_msgs.msg import EgoVehicleStatus
    fixture=make_case(loop,modules,dict(name='DDS_curve_right',radius=300.,side='right'))
    if workload=='no_hazard':fixture.groups=[]
    rows=[];emitted=[];errors=[];stopping=threading.Event()
    original_plan=fixture.planner.plan;first_call=[True]
    def compute(*args,**kwargs):
        start=time.monotonic_ns();cpu=time.process_time_ns()
        if first_call[0] and delay_first_s:time.sleep(delay_first_s)
        first_call[0]=False
        result=original_plan(*args,**kwargs)
        rows.append(dict(kind='lattice_compute',start_monotonic_ns=start,end_monotonic_ns=time.monotonic_ns(),
            wall_s=(time.monotonic_ns()-start)/1e9,cpu_s=(time.process_time_ns()-cpu)/1e9))
        return result
    fixture.planner.plan=compute
    with tempfile.TemporaryDirectory(prefix='stage5d_disarmed_') as directory:
        adapter=NodeCase(fixture,modules,directory,live_timers=True);node=adapter.node
        for timer in node.timers:timer.cancel()
        context=multiprocessing.get_context('spawn');control_stop=context.Event();control_ready=context.Event()
        control_output=Path(directory)/'control_events.json'
        child=context.Process(target=control_process,args=(str(Path(modules['planning_node'].__file__).resolve().parents[4]),
            str(control_output),control_ready,control_stop))
        child.start()
        if not control_ready.wait(6.):
            control_stop.set();child.join(3.);raise RuntimeError('Control timing process startup failed')
        for name in ('local_path_pub','target_speed_pub'):
            setattr(node,name,Forward(getattr(node,name),adapter.ros_publishers[name],node,rows,name))
        planning_executor=SingleThreadedExecutor();planning_executor.add_node(node)
        producer=Node('s4_stage5d_synthetic_producer')
        original_callbacks=[]
        in_planning=[False]
        original_reason=node.sensor_reason
        def freshness(now):
            reason=original_reason(now);guard=node.sensor_guard
            rows.append(dict(kind='freshness_check',**moment(node),reason=reason,inside_planning=in_planning[0],
                source_stamp_ns=guard.matched[0]['source_stamp_ns'] if guard.matched else None))
            return reason
        node.sensor_reason=freshness
        def wrap_subscriptions(target,label):
            for sub in target.subscriptions:
                original=sub.callback;original_callbacks.append((sub,original));topic=sub.topic_name
                def received(msg,original=original,topic=topic,target=target):
                    header=getattr(msg,'header',None);payload=json.loads(msg.data) if isinstance(msg,adapter.String) else {}
                    stamp=(header.stamp.sec*10**9+header.stamp.nanosec) if header else payload.get('source_stamp_ns')
                    row=dict(kind=label,topic=topic,**moment(target),source_stamp_ns=stamp,
                        frame_seq=payload.get('frame_seq'),status_seq=payload.get('status_seq'),
                        published_ros_ns=payload.get('published_ns'))
                    original(msg);row['callback_end_monotonic_ns']=time.monotonic_ns();rows.append(row)
                sub.callback=received
        wrap_subscriptions(node,'planning_receive')
        work_handles=list(node.timers)
        if getattr(node,'_planning_wakeup',None) is not None:work_handles.append(node._planning_wakeup)
        for timer in work_handles:
            original=timer.callback
            if getattr(original,'__name__','') not in ('planning_tick','plan'):continue
            def timed(original=original,trigger='timer' if timer in list(node.timers) else 'ingress'):
                start=moment(node);cpu=time.process_time_ns()
                source=node.sensor_guard.matched[0]['source_stamp_ns'] if node.sensor_guard.matched else None
                in_planning[0]=True
                try:original()
                finally:in_planning[0]=False
                end=moment(node)
                rows.append(dict(kind='planning_tick',trigger=trigger,**start,end_ros_ns=end['ros_ns'],
                    end_monotonic_ns=end['monotonic_ns'],wall_s=(end['monotonic_ns']-start['monotonic_ns'])/1e9,
                    cpu_s=(time.process_time_ns()-cpu)/1e9,source_stamp_ns=source,
                    skipped=getattr(node,'_planning_skipped',False),execution_id=getattr(node,'_planning_execution_id',None),
                    state=node.avoidance_planner.state,reason=node.avoidance_planner.reason,
                    final_freshness=node.sensor_reason(end['ros_ns'])))
            timer.callback=timed
        pubs={}
        for target in (node,):
            for sub in target.subscriptions:
                if sub.msg_type in (adapter.Odometry,adapter.PoseArray,adapter.String,EgoVehicleStatus):
                    pubs.setdefault(sub.topic_name,producer.create_publisher(sub.msg_type,sub.topic_name,10))
        pose_topic=node.localization_sub.topic_name;obs_topic=node.obstacle_sub.topic_name
        status_topic=node.sensor_status_sub.topic_name
        ego_topic=loop.production_values()['ego_status_topic']
        pubs[ego_topic]=producer.create_publisher(EgoVehicleStatus,ego_topic,10)
        # Discovery proceeds in DDS threads; no unprimed Planning callback runs.
        deadline=time.monotonic()+3.
        def discovered():
            return (all(p.get_subscription_count()>0 for p in pubs.values()) and
                all(adapter.ros_publishers[n].get_subscription_count()>0 for n in ('local_path_pub','target_speed_pub')))
        while not discovered() and time.monotonic()<deadline:
            time.sleep(.01)
        if not discovered():
            control_stop.set();child.join(3.);raise RuntimeError('isolated DDS discovery failed')
        adapter.feed(.05);template=adapter.pose_message
        started=time.monotonic();seq=1;session='c'*32;fault_seq=0
        def produce():
            nonlocal seq,session,fault_seq
            try:
                due=started
                while not stopping.is_set():
                    elapsed=time.monotonic()-started;now=producer.get_clock().now().nanoseconds
                    seq+=1;source=now-50_000_000
                    if event=='stale' and elapsed>=.8:source-=800_000_000
                    if event=='clock_regression' and elapsed>=.2:source-=400_000_000;fault_seq=1
                    if event=='restart' and elapsed>=.2 and session=='c'*32:session='d'*32;seq=1
                    if event=='error' and elapsed>=.2:fault_seq=1
                    pose=deepcopy(template);pose.header.stamp.sec=source//10**9;pose.header.stamp.nanosec=source%10**9
                    pubs[pose_topic].publish(pose)
                    ego=EgoVehicleStatus();ego.velocity.x=3./3.6;pubs[ego_topic].publish(ego)
                    if not (event=='dropout' and elapsed>=.8):
                        groups=list(fixture.groups)
                        if event=='new_obstacle' and elapsed>=.2:
                            groups.append(fixture.road.obstacle(25.,6.,5.,0.))
                        points=[p for group in groups for p in group]
                        array=adapter.PoseArray();array.header=pose.header
                        for x,y in points:
                            p=adapter.Pose();p.position.x=float(x);p.position.y=float(y);array.poses.append(p)
                        pubs[obs_topic].publish(array)
                        state='READY_OBSTACLES' if groups else 'READY_EMPTY'
                        if event=='error' and .2<=elapsed<.3:state='PROCESSING_ERROR'
                        if event=='clock_regression' and elapsed>=.2:state='CLOCK_REGRESSION'
                        status=dict(schema_version=1,clock='ros',session_id=session,status_seq=seq,frame_seq=seq,
                            fault_seq=fault_seq,state=state,source_stamp_ns=source,frame_received_ns=now,
                            published_ns=producer.get_clock().now().nanoseconds,cluster_count=len(groups),
                            obstacle_digest=modules['sensor_readiness'].footprint_digest('map',source,points))
                        pubs[status_topic].publish(adapter.String(data=json.dumps(status)))
                        emitted.append(dict(**status,**{'emitted_'+k:v for k,v in moment(producer).items()}))
                    due+=.1;stopping.wait(max(0.,due-time.monotonic()))
            except Exception as error:errors.append(repr(error));stopping.set()
        sensor_thread=threading.Thread(target=produce)
        for target in (node,):
            for timer in target.timers:timer.reset()
        try:
            sensor_thread.start()
            while time.monotonic()-started<duration and not stopping.is_set():planning_executor.spin_once(timeout_sec=.01)
            state=node.avoidance_planner.state;reason=node.avoidance_planner.reason
        finally:
            stopping.set();sensor_thread.join();control_stop.set();child.join(4.)
            if child.is_alive():child.terminate();child.join();errors.append('Control timing process did not stop')
            if control_output.exists():
                data=json.loads(control_output.read_text());rows.extend(data['rows'])
                if data['error']:errors.append(data['error'])
            else:errors.append('missing Control timing events')
            for sub,original in original_callbacks:sub.callback=original
            planning_executor.remove_node(node)
            planning_executor.shutdown();producer.destroy_node();adapter.close()
        if errors:raise RuntimeError(errors)
    return dict(event=event,workload=workload,final_state=state,final_reason=reason,emissions=emitted,
        rows=rows,source_period_s=.1,source_delay_s=.05,duration_s=duration,
        production_enable_drive=False,ctrlcmd_memory_only=True,vehicle_motion_simulated=False,
        controlled_first_delay_s=delay_first_s,control_separate_process=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--trials',type=int,default=10);parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--events',default='normal,dropout,stale,new_frame,error,restart,clock_regression,new_obstacle')
    parser.add_argument('--workloads',default='',help='override workload list for controlled fault probes')
    parser.add_argument('--delay-first-s',type=float,default=0.,help='test-only wall delay, retained in measurements')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True);loop,modules=load(args.source_root)
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 180<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=220:
        raise RuntimeError('DDS probe requires ROS_LOCALHOST_ONLY=1 and isolated ROS_DOMAIN_ID 180..220')
    summary=[]
    for event in args.events.split(','):
        for workload in (tuple(args.workloads.split(',')) if args.workloads else
                         ('no_hazard','nominal') if event=='normal' else ('nominal',)):
            for i in range(args.trials if event=='normal' else 1):
                result=trial(loop,modules,args.seconds,event,workload,args.delay_first_s);name=f'{event}_{workload}_{i}'
                dump(args.out/(name+'.json'),result)
                row=dict(name=name,state=result['final_state'],reason=result['final_reason'])
                summary.append(row);dump(args.out/'summary.json',dict(module=modules['planning_node'].__file__,trials=summary))
                print(json.dumps(row),flush=True)


if __name__=='__main__':main()
