#!/usr/bin/env python3
"""Freeze all executing package files around one isolated, recorded MORAI trial.

This wrapper sends no ground truth to Planning. The existing trial owns
placement, arming, recording, stopping and restoration. Do not edit it or any
pinned runtime file until this wrapper exits.
"""
import argparse,hashlib,importlib,json,os,pathlib,subprocess,sys,tarfile,time,signal

def capture():
    modules={}
    names=['edgenode_planning','edgenode_control','edgenode_perception','edgenode_localization']
    if importlib.util.find_spec('edgenode_normal_experiment') is not None:names.append('edgenode_normal_experiment')
    for name in names:
        module=importlib.import_module(name);root=pathlib.Path(module.__file__).parent
        modules[name]=dict(module_path=str(module.__file__),files={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.iterdir()) if p.suffix in ('.py','.cpp','.so')})
    processes=[]
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:args=[v for v in (p/'cmdline').read_bytes().decode().split('\0') if v]
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if any(v.startswith('/workspace/ros2_ws/install/edgenode_') and '/lib/' in v for v in args):
            processes.append(dict(pid=int(p.name),argv=args))
    return dict(utc_ns=time.time_ns(),modules=modules,processes=processes,
        environment={k:os.environ.get(k) for k in ['AMENT_PREFIX_PATH','PYTHONPATH','ROS_DOMAIN_ID','RMW_IMPLEMENTATION']})

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',choices=['A','B'],required=True)
    p.add_argument('trial_arguments',nargs=argparse.REMAINDER);a=p.parse_args()
    args=a.trial_arguments
    if args and args[0]=='--':args=args[1:]
    output=pathlib.Path(args[args.index('--output')+1]);assert not output.exists()
    if a.stage=='A':
        from morai_drive import Monitor,rclpy
        from edgenode_planning.planning_provenance import implementation_sha256
        rclpy.init();observer=Monitor(reliable_supervision=True);deadline=time.monotonic()+60.;stable=None
        try:
            while time.monotonic()<deadline:
                observer.spin(.05);v=observer.values;sensor=v.get('/planning/s4_sensor_state',{})
                admitted=(observer.stopped() and v.get('/planning/drive_status')=='DISABLED' and
                    v.get('/planning/fsm_state')=='READY' and v.get('/planning/map_match_status')=='MATCHED' and
                    v.get('/planning/local_path',{}).get('points',0)>2 and sensor.get('ready') and
                    not sensor.get('latched_reason') and sensor.get('matched_source_age_sec',999)<.6 and
                    v.get('/planning/avoidance_debug',{}).get('implementation_sha256')==implementation_sha256())
                stable=(stable or time.monotonic()) if admitted else None
                if stable is not None and time.monotonic()-stable>=1.:break
            else:raise RuntimeError('Phase A ready/source/disabled preflight did not settle')
            assert observer.count_publishers('/ctrl_cmd')==1
            (output.parent/(output.name+'-admitted-preflight.json')).write_text(json.dumps(observer.snapshot(),indent=2))
        finally:
            observer.destroy_node();rclpy.shutdown()
    before=capture();receipt=output.parent/(output.name+'-runtime-before.json');assert not receipt.exists()
    receipt.write_text(json.dumps(before,indent=2))
    extra=output.parent/(output.name+'-control-detail');extra.mkdir(exist_ok=False)
    with (extra/'rosbag-console.log').open('x') as stream:
        bag=subprocess.Popen(['ros2','bag','record','-o',str(extra/'bag'),
            '/control/target_left','/control/target_forward','/planning/speed_curvature',
            '/planning/recommended_speed'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            runner='s4_a_trial_rpc.py' if a.stage=='A' else 's4_morai_generalized_trial.py'
            code=subprocess.run([sys.executable,str(pathlib.Path(__file__).with_name(runner)),*args]).returncode
        finally:
            if bag.poll() is None:os.killpg(bag.pid,signal.SIGINT)
            bag.wait(timeout=20.)
    assert bag.returncode==0 and (extra/'bag/metadata.yaml').exists()

    after=capture();same=before['modules']==after['modules']
    if output.exists():
        (output/'runtime-stage-manifest.json').write_text(json.dumps(dict(stage=a.stage,before=before,after=after,runtime_files_unchanged=same,control_detail_bag=str(extra/'bag')),indent=2))
        with tarfile.open(output/'runtime-control-localization-source.tar.gz','x:gz') as archive:
            for name in ['edgenode_control','edgenode_localization']:
                for filename in before['modules'][name]['files']:
                    archive.add(filename,arcname=name+'/'+pathlib.Path(filename).name)
    if not same:raise RuntimeError('runtime source changed while a trial was active')
    return code

if __name__=='__main__':raise SystemExit(main())
