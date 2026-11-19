#!/usr/bin/env python3
"""Disarm and stop exactly the launch recorded by an owned process receipt."""
import argparse,json,pathlib,os,signal,time
import rclpy
from morai_drive import Monitor

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--process-file',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
    info=json.loads(a.process_file.read_text());pid=info['pid'];argv=pathlib.Path('/proc',str(pid),'cmdline').read_bytes();assert b'ros2\x00launch\x00edgenode_bringup\x00' in argv
    rclpy.init();n=Monitor(reliable_supervision=True);n.spin(2.);assert n.stop() and n.count_publishers('/ctrl_cmd')==1;before=n.snapshot();parameters=n.parameters();assert parameters['enable_drive'] is False
    parents={}
    for process in pathlib.Path('/proc').glob('[0-9]*'):
        try:parents[int(process.name)]=int((process/'stat').read_text().rpartition(')')[2].split()[1])
        except (FileNotFoundError,ValueError,IndexError):continue
    owned={pid}
    while True:
        expanded=owned|{child for child,parent in parents.items() if parent in owned}
        if expanded==owned:break
        owned=expanded
    children=sorted(owned-{pid})
    def running(child):
        try:return pathlib.Path('/proc',str(child),'stat').read_text().rpartition(')')[2].split()[0]!='Z'
        except FileNotFoundError:return False
    os.kill(pid,signal.SIGINT);deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        n.spin(.1)
        if n.count_publishers('/ctrl_cmd')==0 and all(not running(c) for c in children):break
    else:raise RuntimeError('owned launch has remaining children or a control publisher')
    a.output.write_text(json.dumps(dict(pid=pid,children=children,parameters_before_stop=parameters,before=before,control_publishers_after_stop=0),indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
