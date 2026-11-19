"""Replace exactly one disarmed LiDAR producer; preserve its command and logs.

The separate Planning restart must follow to admit the producer's NEW session.
Session guards are never reset or weakened inside an existing Planning node.
"""
import argparse,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
sys.path.insert(0,'/workspace/tools')
from morai_drive import Monitor,rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import edgenode_perception

p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True)
p.add_argument('--output-prefix',type=Path,required=True);args=p.parse_args()
assert args.config.is_file()
logfile=Path(str(args.output_prefix)+'.log');record=Path(str(args.output_prefix)+'.json')
assert not logfile.exists() and not record.exists()
rclpy.init();n=Monitor()
n.create_subscription(String,'/perception/lidar_diagnostics',lambda m:n.receive('/perception/lidar_diagnostics',m),qos_profile_sensor_data)
deadline=time.monotonic()+12.
while time.monotonic()<deadline:
    n.spin(.05)
    if n.fresh():break
assert n.fresh() and n.count_publishers('/ctrl_cmd')==1
assert not n.parameters()['enable_drive'] and n.stop()
exe='/workspace/ros2_ws/install/edgenode_perception/lib/edgenode_perception/lidar_obstacle_node'
found=[]
for proc in Path('/proc').glob('[0-9]*'):
    try:argv=proc.joinpath('cmdline').read_bytes().decode().split('\0')
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if exe in argv:found.append((int(proc.name),argv))
assert len(found)==1,found
pid,previous=found[0];os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+12.
while time.monotonic()<deadline:
    try:
        if Path(f'/proc/{pid}/stat').read_text().split()[2]=='Z':break
    except FileNotFoundError:break
    n.spin(.05)
else:raise RuntimeError('old perception did not stop')
argv=['/usr/bin/python3',exe,'--ros-args','--params-file',str(args.config.resolve())]
with logfile.open('w') as log:
    process=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
from edgenode_perception.provenance import implementation_manifest
files=implementation_manifest()
fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
record.write_text(json.dumps(dict(replaced_pid=pid,previous_argv=previous,pid=process.pid,argv=argv,
    runtime_source_sha256=files,expected_implementation_sha256=fingerprint,
    config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest()),indent=2)+'\n')
deadline=time.monotonic()+20.
while time.monotonic()<deadline:
    n.spin(.05)
    if process.poll() is not None:raise RuntimeError('new perception failed: '+logfile.read_text()[-2000:])
    value=n.values.get('/perception/lidar_diagnostics',{})
    if (value.get('implementation_sha256')==fingerprint and
        value.get('footprint_mode')=='clipped_minimum_area_tiled' and
        0<=(n.get_clock().now().nanoseconds-value['source_stamp_ns'])/1e9<=.6):break
else:raise RuntimeError('new producer provenance/freshness not confirmed')
assert not n.parameters()['enable_drive'] and n.stop()
print(json.dumps(dict(pid=process.pid,implementation_sha256=fingerprint,drive_enabled=False)))
n.destroy_node();rclpy.shutdown()
