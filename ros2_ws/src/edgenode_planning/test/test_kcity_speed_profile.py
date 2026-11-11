"""60/40/30 policy, independent hard cap, and unchanged physical limits."""
from pathlib import Path
import math

import pytest
import yaml

from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.longitudinal_speed_planner import (
    CurveSpan, RouteSpeedProfile, LongitudinalSpeedPlanner, SpeedPlanSettings,
    braking_distance, approach_speed,
)
from test_speed_parameters_ros import make_node, change, output, set_geometry
from test_long_speed_parameters_ros import atomic_service, status, warm

PROFILE=dict(cruise_speed_kmh=60.,curve_speed_kmh=40.,sharp_curve_speed_kmh=30.)


def test_installed_configuration_separates_profile_from_low_speed_default():
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('edgenode_planning'))/'config'
    base=yaml.safe_load((config/'planning.yaml').read_text())['planning_node']['ros__parameters']
    high=yaml.safe_load((config/'kcity_speed_profile_60.yaml').read_text())['planning_node']['ros__parameters']
    for name,value in PROFILE.items():
        assert base[name]==high[name]==value
    assert base['enable_drive'] is high['enable_drive'] is False
    assert base['max_drive_speed_kmh']==20.
    assert high['max_drive_speed_kmh']==60.
    assert base['speed_long_preview_enabled'] is True
    assert high['speed_long_preview_enabled'] is True


@pytest.mark.parametrize('curvature,mode,policy',[(0.,'CRUISE',60.),(.05,'CURVE',40.),(.2,'SHARP_CURVE',30.)])
@pytest.mark.parametrize('cap',[20.,30.,60.])
def test_atomic_runtime_profile_and_independent_drive_gate(make_node,curvature,mode,policy,cap):
    # Local-only policy isolation. All outputs are mocks in private domain193.
    node=make_node(enable_drive=False)
    set_geometry(node,curvature)
    result=atomic_service(node,**PROFILE,max_drive_speed_kmh=cap,enable_drive=False)
    assert result.successful
    node.plan()
    assert output(node,'speed_mode')==mode
    assert output(node,'recommended_speed')==policy
    assert output(node,'target_speed')==0.
    assert change(node,enable_drive=True).successful
    node.plan()
    assert output(node,'target_speed')==min(cap,policy)


def test_enabled_global_straight_has_sixty_recommendation_while_stationary_disabled(make_node):
    node=make_node(**PROFILE,enable_drive=False,max_drive_speed_kmh=60.,speed_long_preview_enabled=True)
    node.dijkstra_planner.plan.return_value={'points':[[0.,0.,0.],[1000.,0.,0.]]}
    warm(node)
    assert output(node,'recommended_speed')==60.
    assert output(node,'target_speed')==0.
    profile=node._long_planner.profile
    for cap in [20.,60.]:
        assert atomic_service(node,max_drive_speed_kmh=cap,enable_drive=False).successful
        status(node,speed=0.);node.plan()
        assert node._long_planner.profile is profile
        assert output(node,'recommended_speed')==60.
        assert output(node,'target_speed')==0.


@pytest.mark.parametrize('level,k,policy',[(1,.05,40.),(2,.2,30.),(2,1/5.04,30.)])
def test_lateral_cap_can_be_below_each_profile_limit(level,k,policy):
    span=CurveSpan(200.,210.,level,1,k,k*10,3)
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(1000.,(span,)),SpeedPlanner(**PROFILE))
    expected=min(policy,3.6*math.sqrt(1.5/k))
    result=planner.plan(200.,expected,policy)
    assert result['target_speed_kmh']==pytest.approx(expected)
    assert result['target_speed_kmh']<policy


@pytest.mark.parametrize('target',[30.,25.,3.6*math.sqrt(1.5*5.04)])
def test_sixty_braking_budget_and_preview_crossing(target):
    cfg=SpeedPlanSettings()
    required=braking_distance(60.,target,cfg)
    expected=(60/3.6)*.5+((60/3.6)**2-(target/3.6)**2)/3+5
    assert required==pytest.approx(expected)
    assert required>80.
    assert approach_speed(required,target,cfg)==pytest.approx(60.)
    assert approach_speed(required-.25,target,cfg)<60.
    assert approach_speed(0.,target,cfg)==target


def test_sixty_dynamic_preview_sees_tight_curve_before_braking_boundary():
    cfg=SpeedPlanSettings()
    k=1/5.04;goal=3.6*math.sqrt(cfg.lateral_accel_mps2/k)
    required=braking_distance(60.,goal,cfg)
    span=CurveSpan(200.,207.,2,-1,k,-k*7,3)
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(1000.,(span,)),SpeedPlanner(**PROFILE),cfg)
    before=planner.plan(200.-required-.25,60.,60.)
    after=planner.plan(200.-required+.25,60.,60.)
    assert before['target_speed_kmh']==60.
    assert after['target_speed_kmh']<60.
    assert after['distance_m']>100.
    assert after['requested_preview_m']>required>80.
    assert before['braking_feasible'] and not after['urgent']
    # Holding 60 after crossing the budget boundary consumes the safety
    # margin. The diagnostic must report it; this is not a closed-loop model.
    assert not after['braking_feasible']
    tracking=planner.plan(200.-required+.25,after['target_speed_kmh'],60.)
    assert tracking['braking_feasible']


def test_sixty_profile_never_raises_a_lower_upstream_local_limit():
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(1000.,()),SpeedPlanner(**PROFILE))
    assert planner.plan(0.,5.,7.)['target_speed_kmh']==7.


def test_invalid_profile_batch_keeps_old_cap_and_settings(make_node):
    node=make_node(enable_drive=False,max_drive_speed_kmh=20.)
    old=node._speed_planner_snapshot
    result=atomic_service(node,cruise_speed_kmh=60.,curve_speed_kmh=30.,
        sharp_curve_speed_kmh=float('nan'),max_drive_speed_kmh=60.)
    assert not result.successful
    assert node.get_parameter('max_drive_speed_kmh').value==20.
    assert node.get_parameter('cruise_speed_kmh').value==50.
    assert node._speed_planner_snapshot==old
