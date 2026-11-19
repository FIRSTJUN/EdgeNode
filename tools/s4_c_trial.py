"""Isolated C trial observer: freeze runtime; Actor truth remains audit-only."""
import hashlib,importlib,json,pathlib,runpy,sys,tarfile,time
import rclpy
from std_msgs.msg import String
from morai_drive import Monitor
from s4_trial_rpc import acknowledged_call
from s4_stage_trial import capture
from edgenode_dynamic_tracking.provenance import implementation_sha256

def runtime():
    data=capture();module=importlib.import_module('edgenode_dynamic_tracking');root=pathlib.Path(module.__file__).parent
    data['modules']['edgenode_dynamic_tracking']=dict(module_path=str(module.__file__),files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.py'))})
    return data

def main():
    output=pathlib.Path(sys.argv[sys.argv.index('--output')+1]);assert not output.exists()
    rclpy.init();n=Monitor(reliable_supervision=True);n.create_subscription(String,'/dynamic_tracking/tracks',lambda m:n.receive('/dynamic_tracking/tracks',m),1)
    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            n.spin(.1)
            if n.stopped() and n.values.get('/planning/fsm_state')=='READY' and n.values.get('/dynamic_tracking/tracks',{}).get('ready'):break
        else:raise RuntimeError('C stopped/source-ready admission failed')
        assert n.parameters()['enable_drive'] is False and n.count_publishers('/ctrl_cmd')==1
        assert n.values['/dynamic_tracking/tracks']['implementation_sha256']==implementation_sha256()
        (output.parent/(output.name+'-admission.json')).write_text(json.dumps(n.snapshot(),indent=2))
    finally:n.destroy_node();rclpy.shutdown()
    before=runtime();(output.parent/(output.name+'-runtime-before.json')).write_text(json.dumps(before,indent=2))
    Monitor.call=acknowledged_call
    code=0
    try:runpy.run_path(str(pathlib.Path(__file__).with_name('s4_c_morai_tracking_trial.py')),run_name='__main__')
    except SystemExit as error:code=int(error.code or 0)
    finally:
        after=runtime();same=before['modules']==after['modules']
        if output.exists():
            (output/'runtime-stage-manifest.json').write_text(json.dumps(dict(stage='C',before=before,after=after,runtime_files_unchanged=same),indent=2))
            with tarfile.open(output/'runtime-c-source.tar.gz','x:gz') as archive:
                for filename in before['modules']['edgenode_dynamic_tracking']['files']:archive.add(filename,arcname='edgenode_dynamic_tracking/'+pathlib.Path(filename).name)
        if not same:raise RuntimeError('runtime source changed during C trial')
    return code

if __name__=='__main__':raise SystemExit(main())
