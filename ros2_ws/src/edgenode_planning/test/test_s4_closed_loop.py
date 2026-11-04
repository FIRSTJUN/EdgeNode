"""Causal regression: future poses come only from the approved output geometry."""
import copy
import math
import json
from pathlib import Path

import pytest

from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner,WorldTransition
from edgenode_planning.lane_geometry import corridor_covers
from edgenode_planning.local_path_planner import LocalPathPlanner
from s4_stateful_helpers import live_sequences,restored_sequence,recorded_inputs
from s4_closed_loop import advance,perturb
from test_adjacent_lane_planner import network,plan


@pytest.fixture(scope='module')
def starts():
    fixture=live_sequences();out={}
    for run in fixture['runs']:
        start=143 if run['name']=='OLD' else 139
        p=restored_sequence(fixture,run)
        for frame in run['frames']:
            if frame['index']>=start:break
            p.plan(**recorded_inputs(run,frame))
        frame=next(f for f in run['frames'] if f['index']==start)
        out[run['name']]=(p,recorded_inputs(run,frame))
    return out


def check_safe(p,result,yaw):
    assert not result['blocked'],result['evaluated_candidate_reject_reason']
    assert result['maximum_curvature']<=p.usable_curvature_limit
    assert result['minimum_clearance_m']>=p.required_clearance_m
    assert result['drivable_safe']
    if p._world_transition is not None:
        _,v,_=p._world_transition.state(0.)
        error=math.atan2(math.sin(math.atan2(v[1],v[0])-yaw),math.cos(math.atan2(v[1],v[0])-yaw))
        assert abs(error)<1e-8
        if result.get('maneuver_evaluation',{}).get('boundary_mode') in ('geometric','corridor_handles'):
            curve=p._world_transition
            for u in (0.,.0001,.001,.003,.01,.03,.1,.2,.4,.6,.8,.9,.99,.999,1.):
                _,v,a=curve.state(u*curve.length)
                k=abs(v[0]*a[1]-v[1]*a[0])/math.hypot(*v)**3
                assert k<=p.usable_curvature_limit+1e-8


@pytest.mark.parametrize('name',['OLD','NEW'])
@pytest.mark.parametrize('noise,dt',[(0.,.05),(.001,.05),(.01,.134)])
def test_last_safe_bag_state_reaches_hold_without_future_bag_odometry(starts,name,noise,dt,record_property):
    p,args=copy.deepcopy(starts[name]);pair=p._lane_pair
    position=args['current_position'];yaw=args['current_heading'];progress=args['road_progress_m']
    history=[];maximum_curvature=0.;minimum_clearance=math.inf
    for cycle in range(160):
        progress=max(progress,pair.original_index.project(position))
        args.update(current_position=position,current_heading=yaw,road_progress_m=progress,
            reference_points=LocalPathPlanner().extract(pair.original,*position,horizon_m=20.)['points'])
        result=p.plan(**args);check_safe(p,result,yaw)
        maximum_curvature=max(maximum_curvature,result['maximum_curvature'])
        minimum_clearance=min(minimum_clearance,result['minimum_clearance_m'])
        history.append(result['lane_change_phase'])
        if history[-1]=='ADJACENT_LANE_HOLD':break
        position,yaw=advance(p,result,position,3./3.6*dt)
        if cycle%47==0:position,yaw=perturb(pair,position,yaw,noise,cycle)
    assert history[0]=='ENTRY'
    assert history[-1]=='ADJACENT_LANE_HOLD'
    # Traverse beyond the map station at which the original vehicle failed.
    assert progress>(19.7095 if name=='OLD' else 19.6738)
    record_property('closed_loop',json.dumps(dict(name=name,noise_m=noise,dt=dt,cycles=len(history),
        phases=list(dict.fromkeys(history)),final_progress=progress,maximum_curvature=maximum_curvature,minimum_clearance=minimum_clearance)))


@pytest.mark.parametrize('noise',[0.,.001,.01])
@pytest.mark.parametrize('side,sign',[('right',-1.),('left',1.)])
def test_causal_full_maneuver_with_bounded_inward_pose_and_yaw_disturbances(noise,side,sign,record_property):
    p=AdjacentLanePlanner(network((side,)));position=[0.,0.];yaw=0.;phases=[];transitions=[]
    maximum_curvature=0.;minimum_clearance=math.inf
    for cycle in range(1000):
        result=plan(p,*position,yaw,speed=3.);check_safe(p,result,yaw)
        maximum_curvature=max(maximum_curvature,result['maximum_curvature'])
        minimum_clearance=min(minimum_clearance,result['minimum_clearance_m'])
        if not phases or phases[-1]!=result['lane_change_phase']:
            phases.append(result['lane_change_phase']);transitions.append([cycle,position[0],result['lane_change_phase']])
        if result['status']=='CLEAR':break
        pair=p._lane_pair;position,yaw=advance(p,result,position,3./3.6/20.)
        if cycle%47==0:position,yaw=perturb(pair,position,yaw,noise,cycle)
    assert phases==['ENTRY','ADJACENT_LANE_HOLD','RECOVERING','CLEAR']
    assert p._lane_pair is None and p._recovery_completion_progress is None
    record_property('closed_loop',json.dumps(dict(name='synthetic_'+side,noise_m=noise,dt=.05,cycles=cycle+1,
        phases=phases,transitions=transitions,maximum_curvature=maximum_curvature,minimum_clearance=minimum_clearance)))


@pytest.mark.parametrize('name,index',[('OLD',145),('NEW',144),('NEW',145)])
def test_original_unrecoverable_recorded_inputs_still_block(name,index):
    fixture=live_sequences();run=next(r for r in fixture['runs'] if r['name']==name)
    p=restored_sequence(fixture,run)
    for frame in run['frames']:
        result=p.plan(**recorded_inputs(run,frame))
        if frame['index']==index:break
    assert result['status']=='BLOCKED'
    assert result['lane_change_phase']=='BLOCKED'
    assert not result['drivable_safe'] or result['maximum_curvature']>p.usable_curvature_limit


def test_continuous_curve_cannot_escape_between_safe_published_vertices():
    curve=WorldTransition.__new__(WorldTransition)
    curve.length=1.
    curve.coefficients=[[0.,1.,0.,0.,0.,0.],[0.,.0005,-.0015,.001,0.,0.]]
    cell=[[[0.,0.],[1.,0.],[1.,1.],[0.,1.]]]
    assert corridor_covers([curve.state(u)[0] for u in (0.,.5,1.)],cell)
    safe,witness=curve.corridor_safe(cell)
    assert not safe
    assert curve.state(witness)[0][1]<0.
