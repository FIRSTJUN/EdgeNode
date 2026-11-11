"""Tool regressions: no MORAI participants or motion publishers are created."""
import importlib.util
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('live_speed_tool',ROOT/'tools/kcity/live_test.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)


@pytest.fixture(autouse=True)
def isolated_runner_lock(monkeypatch,tmp_path):
    monkeypatch.setattr(tool,'RUNNER_LOCK_PATH',tmp_path/'runner.lock')


def test_overlapping_runner_refuses_before_ros_or_parameter_changes(monkeypatch):
    import fcntl
    monkeypatch.setattr(tool.rclpy,'init',lambda:pytest.fail('must not initialize ROS'))
    with tool.RUNNER_LOCK_PATH.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):tool.main(['--prepare'])


@pytest.mark.parametrize('cap',[20.,30.,40.,50.,60.])
def test_explicit_ceiling_and_low_speed_default(cap):
    args=tool.parse_args(['--prepare','--max-speed-kmh',str(cap)])
    p=tool.preparation_parameters(args.max_speed_kmh)
    assert p['max_drive_speed_kmh']==cap and p['enable_drive'] is False
    assert p['cruise_speed_kmh']==60. and p['sharp_curve_speed_kmh']==30.
    assert tool.parse_args(['--prepare']).max_speed_kmh==20.
    assert 'speed_plan_lateral_accel_mps2' not in p # don't overwrite stricter settings


@pytest.mark.parametrize('argv',[
    [],['--prepare','--run'],['--prepare','--max-speed-kmh','0'],
    ['--prepare','--max-speed-kmh','61'],['--prepare','--max-speed-kmh','nan'],
    ['--run','--max-speed-kmh','inf'],['--run','--seconds','0'],
    ['--run','--seconds','121'],['--run','--seconds','nan']])
def test_bad_arguments_rejected_before_ros_init(monkeypatch,argv):
    monkeypatch.setattr(tool.rclpy,'init',lambda:pytest.fail('must not initialize ROS'))
    with pytest.raises(SystemExit):tool.main(argv)


def parameters(cap):
    return dict(tool.SAFETY_SETTINGS,**tool.preparation_parameters(cap),local_path_length_m=20.)


@pytest.mark.parametrize('requested,actual',[(20.,60.),(60.,20.)])
def test_run_cannot_mistake_low_and_high_modes(requested,actual):
    with pytest.raises(RuntimeError,match='max_drive_speed'):
        tool.verify_parameters(parameters(actual),requested)


@pytest.mark.parametrize('name,value',[('enable_drive',True),('speed_long_preview_enabled',False),
    ('speed_plan_lateral_accel_mps2',2.),('speed_plan_speed_timeout_sec',.5),
    ('speed_plan_deceleration_mps2',2.),('local_path_length_m',30.),
    ('speed_plan_distance_margin_m',1.),('speed_plan_acceleration_mps2',2.)])
def test_changed_safety_settings_refuse_start(name,value):
    p=parameters(60.);p[name]=value
    with pytest.raises(RuntimeError):tool.verify_parameters(p,60.)


def test_stricter_lateral_limit_preserved():
    p=parameters(60.);p['speed_plan_lateral_accel_mps2']=.8
    tool.verify_parameters(p,60.)


@pytest.mark.parametrize('cap',[20.,60.])
def test_prepare_never_enables_drive(monkeypatch,tmp_path,cap):
    sessions=[]
    class Fake:
        def __init__(self,name,output_dir):
            self.output_dir=Path(output_dir);self.records=[];self.log=io.StringIO()
            self.p=parameters(20.);self.changes=[];sessions.append(self)
            publishers={'/planning/target_speed':'planning_node','/ctrl_cmd':'control_node','/ego_vehicle_status':'MoraiSimNode'}
            self.node=SimpleNamespace(get_node_names_and_namespaces=lambda:[(n,'/') for n in ['MoraiSimNode','planning_node','control_node','localization_node']],
                get_publishers_info_by_topic=lambda t:[SimpleNamespace(node_name=publishers[t])],
                destroy_node=lambda:None)
            self.activate_client=object()
        def spin(self,*args):pass
        def get(self):return dict(self.p)
        def control_settings(self):return dict(steering_max_rate_per_sec=1.5)
        def set(self,**values):self.changes.append(values);self.p.update(values)
        def stop(self):self.set(enable_drive=False);return True
        def call(self,*args,**kwargs):return SimpleNamespace(success=True,message='activated')
        def wait_ready(self,*args):assert self.p['enable_drive'] is False
        def reanchor_stopped(self):return dict(available=False)
        def capture_global_route(self):return dict(points=4911,frame='map',length_m=2444.,sha256='fixture')
        def brief(self):return dict(target=0.,speed=0.,drive='DISABLED')
    monkeypatch.setattr(tool,'Session',Fake)
    monkeypatch.setattr(tool.rclpy,'init',lambda:None)
    monkeypatch.setattr(tool.rclpy,'shutdown',lambda:None)
    assert tool.main(['--prepare','--max-speed-kmh',str(cap),'--output-dir',str(tmp_path)])==0
    s=sessions[0]
    assert not any(c.get('enable_drive') is True for c in s.changes)
    assert s.p['max_drive_speed_kmh']==cap
    assert s.p['speed_plan_lateral_accel_mps2']==1.5


@pytest.mark.parametrize('cap',[20.,60.])
def test_monitor_uses_selected_ceiling(cap):
    latest={'/localization/status':'EKF_READY','/localization/odometry':{'age':0.01},'/ego_vehicle_status':{'speed':cap,'age':0.01},'/planning/target_speed':cap,'/control/path_error':.1}
    topics=['/ego_vehicle_status','/planning/target_speed','/ctrl_cmd','/control/path_error',
            '/planning/long_speed_plan','/planning/map_match_status','/localization/status','/localization/odometry','/planning/local_path']
    times={t:100. for t in topics}
    assert tool.immediate_faults(latest,times,100.,cap)==[]
    latest['/planning/target_speed']=cap+.1
    assert 'target exceeds selected ceiling' in tool.immediate_faults(latest,times,100.,cap)
    latest['/planning/target_speed']=cap;latest['/ego_vehicle_status']['speed']=math.nan
    assert tool.immediate_faults(latest,times,100.,cap)
    assert tool.immediate_faults(latest,times,101.,cap)


def test_legacy_profile_is_explicit_and_checked_on_both_modes():
    for mode in ('--prepare','--run'):
        args=tool.parse_args([mode,'--legacy-profile','--max-speed-kmh','50'])
        assert args.legacy_profile
        p=dict(tool.SAFETY_SETTINGS,**tool.preparation_parameters(50.,True),local_path_length_m=20.)
        tool.verify_parameters(p,50.,True)
        with pytest.raises(RuntimeError,match='curve_speed_kmh'):
            tool.verify_parameters(p,50.)


@pytest.mark.parametrize('age',[float('nan'),.31,-.06])
def test_stale_or_future_localization_never_arms(age):
    latest={'/localization/status':'EKF_READY','/localization/odometry':{'age':age},
        '/ego_vehicle_status':{'speed':10.,'age':.01},'/planning/target_speed':10.,'/control/path_error':.1}
    times={t:100. for t in ['/localization/status','/localization/odometry','/planning/local_path',
        '/ego_vehicle_status','/planning/target_speed','/ctrl_cmd','/control/path_error',
        '/planning/long_speed_plan','/planning/map_match_status']}
    assert 'stale/future sensor stamp: /localization/odometry' in tool.immediate_faults(latest,times,100.,50.)
