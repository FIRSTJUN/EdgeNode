from types import SimpleNamespace
from unittest.mock import Mock
import math
import pytest
from edgenode_planning import planning_node as module


def planner(enabled=True, duration=60., started=10.):
    params={'enable_drive':enabled, 'drive_duration_sec':duration, 'status_timeout_sec':.6}
    return SimpleNamespace(p=lambda k:params[k], drive_started=started,
                           collision_latched=False, ego=None, publish=Mock())


def test_drive_lease_expires_before_any_other_output(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:70.)
    p=planner()
    module.PlanningNode.plan(p)
    assert p.publish.call_args.args == ('DRIVE_SESSION_COMPLETE',)
    assert 'speed' not in p.publish.call_args.kwargs  # publish's default is a stop.


@pytest.mark.parametrize('duration',[0.,math.inf,301.])
def test_unbounded_or_invalid_drive_session_stops(monkeypatch,duration):
    monkeypatch.setattr(module.time,'monotonic',lambda:20.)
    p=planner(duration=duration)
    module.PlanningNode.plan(p)
    p.publish.assert_called_once_with('INVALID_DRIVE_DURATION')


def test_disabling_drive_resets_lease_without_enabling_motion(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:80.)
    p=planner(enabled=False)
    module.PlanningNode.plan(p)
    assert p.drive_started is None
    p.publish.assert_called_once_with('WAIT_FOR_EGO')


@pytest.mark.parametrize('enabled,expected', [(False,'PREVIEW_STOP'),(True,'MAP_OR_POSE_INVALID')])
def test_reposition_reacquires_only_when_drive_disabled(monkeypatch,enabled,expected):
    from pathlib import Path
    import yaml
    from edgenode_planning.map_tracker import MapTracker
    from edgenode_planning.fsm import PlanningConfig, PlanningFSM
    from edgenode_planning.input_adapter import PerceptionInputs, obstacle_input
    monkeypatch.setattr(module.time,'monotonic',lambda:20.)
    params=yaml.safe_load((Path(__file__).parents[1]/'config/planning.yaml').read_text())['planning_node']['ros__parameters']
    params['enable_drive']=enabled
    tracker=MapTracker(params['map_file'])
    assert tracker.acquire(__import__('numpy').array([-18.9658,101.1357]),math.radians(-94.81))
    inputs=PerceptionInputs();inputs.error=(0.,20.);inputs.confidence=(.8,20.)
    inputs.obstacle=obstacle_input([math.nan]*3+[math.inf,0.],20.)
    p=SimpleNamespace(p=lambda k:params[k],drive_started=10.,collision_latched=False,
       ego=SimpleNamespace(position=SimpleNamespace(x=125.2452,y=-82.9869),heading=-84.6316,
                           velocity=SimpleNamespace(x=0.,y=0.,z=0.)),ego_stamp=20.,
       inputs=inputs,tracker=tracker,fsm=PlanningFSM(PlanningConfig(cruise_speed=6.,avoid_distance=3.5)),
       lane_confident=True,publish=Mock(),last_report=20.)
    module.PlanningNode.plan(p)
    assert p.publish.call_args.args[0]==expected
    if not enabled:
        assert p.tracker.current=='A222CC001003'
        assert p.publish.call_args.args[1]==0.
