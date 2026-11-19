#!/usr/bin/env python3
"""Read-only stopped runtime receipt with raw source timestamp progression."""
import argparse,json,pathlib,time,sys
import rclpy
from morai_drive import Monitor
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,Imu
from morai_ros2_msgs.msg import GPSMessage
from rclpy.qos import qos_profile_sensor_data

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--seconds',type=float,default=6.);a=p.parse_args();assert not a.output.exists()
    rclpy.init();n=Monitor(reliable_supervision=True);counts={};samples=[]
    def receive(topic,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec;v=counts.setdefault(topic,dict(messages=0,advancing=0,duplicates=0,backwards=0,first_source_ns=stamp,last_source_ns=0,max_source_age_s=0.))
        v['messages']+=1
        if stamp>v['last_source_ns']:v['advancing']+=1
        elif stamp==v['last_source_ns']:v['duplicates']+=1
        else:v['backwards']+=1
        v['last_source_ns']=stamp;v['max_source_age_s']=max(v['max_source_age_s'],(n.get_clock().now().nanoseconds-stamp)/1e9)
    for topic,kind in [('/lidar/points',PointCloud2),('/Imu',Imu),('/gps',GPSMessage)]:n.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
    n.create_subscription(String,'/planning/phase_a_runtime',lambda m:n.receive('/planning/phase_a_runtime',m),1)
    n.create_subscription(String,'/planning/phase_b_runtime',lambda m:n.receive('/planning/phase_b_runtime',m),1)
    n.spin(2.);end=time.monotonic()+a.seconds
    while time.monotonic()<end:
        n.spin(.1);samples.append(n.snapshot());assert n.stopped() and n.count_publishers('/ctrl_cmd')==1
    for topic,v in counts.items():assert v['advancing']>=10 and v['last_source_ns']>v['first_source_ns'] and v['backwards']==0,(topic,v)
    assert len(counts)==3
    final=n.snapshot();sensor=final['values']['/planning/s4_sensor_state'];assert sensor['ready'] and not sensor['latched_reason'] and sensor['matched_source_age_sec']<.6
    a.output.write_text(json.dumps(dict(raw_sources=counts,parameters=n.parameters(),samples=samples,final=final),indent=2));print(json.dumps(dict(raw_sources=counts,stopped=True,control_publishers=1),indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
