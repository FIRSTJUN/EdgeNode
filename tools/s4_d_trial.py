"""Isolated D trial observer: freeze runtime; Actor truth remains audit-only."""
import hashlib,importlib,json,pathlib,runpy,sys,tarfile,time
import rclpy
from std_msgs.msg import String
from morai_drive import Monitor
from s4_trial_rpc import acknowledged_call
from s4_stage_trial import capture
from edgenode_dynamic_overtake.provenance import implementation_sha256

def runtime():
    data=capture()
    for name in ['edgenode_dynamic_tracking','edgenode_dynamic_overtake']:
        module=importlib.import_module(name);root=pathlib.Path(module.__file__).parent
        data['modules'][name]=dict(module_path=str(module.__file__),files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.py'))})
    return data

def main():
    output=pathlib.Path(sys.argv[sys.argv.index('--output')+1]);assert not output.exists()
    rclpy.init();n=Monitor(reliable_supervision=True);n.create_subscription(String,'/dynamic_tracking/tracks',lambda m:n.receive('/dynamic_tracking/tracks',m),1);n.create_subscription(String,'/planning/phase_d_runtime',lambda m:n.receive('/planning/phase_d_runtime',m),1)
    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            n.spin(.1)
            if n.stopped() and n.values.get('/planning/fsm_state')=='READY' and n.values.get('/dynamic_tracking/tracks',{}).get('ready') and n.values.get('/planning/phase_d_runtime',{}).get('implementation_sha256')==implementation_sha256():break
        else:raise RuntimeError('D stopped/source-ready admission failed')
        assert n.parameters()['enable_drive'] is False and n.count_publishers('/ctrl_cmd')==1
        assert n.values['/planning/phase_d_runtime']['implementation_sha256']==implementation_sha256()
        (output.parent/(output.name+'-admission.json')).write_text(json.dumps(n.snapshot(),indent=2))
    finally:n.destroy_node();rclpy.shutdown()
    import os
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    # Use the B observer's exact startup-mission reference reconstruction.
    rclpy.init();q=Monitor(reliable_supervision=True)
    try:
        q.spin(1.);names=['phase_b_normal_tracking_clearance_enabled','surveyed_road_file','surveyed_road_sha256','avoidance_max_front_steer_normalized','avoidance_max_wheel_angle_rad','avoidance_wheelbase_m','route_start_link_id','route_node_ids']
        values=list(map(parameter_value_to_python,q.call(q.getter,GetParameters.Request(names=names)).values));a=dict(zip(names,values));import math
        os.environ['EDGENODE_B_REFERENCE_VERIFICATION']=json.dumps(dict(enabled=a[names[0]],road_file=a[names[1]],road_sha256=a[names[2]],steering_curvature_limit=math.tan(a[names[3]]*a[names[4]])/a[names[5]],warm_start_link_id=a[names[6]],warm_node_ids=a[names[7]]))
    finally:q.destroy_node();rclpy.shutdown()
    from s4_b_route_verification import install
    install()
    before=runtime();(output.parent/(output.name+'-runtime-before.json')).write_text(json.dumps(before,indent=2))
    Monitor.call=acknowledged_call
    code=0
    try:runpy.run_path(str(pathlib.Path(__file__).with_name('s4_d_morai_overtake_trial.py')),run_name='__main__')
    except SystemExit as error:code=int(error.code or 0)
    finally:
        after=runtime();same=before['modules']==after['modules']
        if output.exists():
            (output/'runtime-stage-manifest.json').write_text(json.dumps(dict(stage='D',before=before,after=after,runtime_files_unchanged=same),indent=2))
            with tarfile.open(output/'runtime-d-source.tar.gz','x:gz') as archive:
                for filename in before['modules']['edgenode_dynamic_overtake']['files']:archive.add(filename,arcname='edgenode_dynamic_overtake/'+pathlib.Path(filename).name)
        if not same:raise RuntimeError('runtime source changed during C trial')
    return code

if __name__=='__main__':raise SystemExit(main())
