"""Recovery acquisition ends the maneuver only after a safe actual-pose handoff."""
import math

import pytest

from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner
from test_adjacent_lane_planner import network,plan,obstacle


def recovering():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    assert plan(p,17.,-3.5,current='R',speed=3.)['lane_change_phase']=='ADJACENT_LANE_HOLD'
    assert plan(p,22.5,-3.5,current='R',speed=3.)['lane_change_phase']=='ADJACENT_LANE_HOLD'
    r=plan(p,22.6,-3.5,current='R',speed=3.)
    assert r['status']=='RECOVERING'
    return p,p._recovery_completion_progress


@pytest.mark.parametrize('offset,yaw',[(0.,0.),(.001,0.),(.03,.001),(.149999,0.)])
def test_recovery_handoff_uses_fixed_acquisition_goal_and_approved_correction(offset,yaw):
    p,goal=recovering();phases=[]
    for i in range(40):
        progress=goal-.2+i*(3./3.6/20.)
        r=plan(p,progress,-offset,yaw,speed=3.)
        phases.append(r['lane_change_phase'])
        if r['status']=='CLEAR':
            assert r['recovery_acquired']
            assert r['drivable_safe']
            assert r['maximum_curvature']<=p.usable_curvature_limit
            assert r['minimum_clearance_m']>=p.required_clearance_m
            assert r['points'][0][:2]==pytest.approx([progress,-offset])
            assert p._lane_pair is None and p._world_transition is None
            assert p._recovery_completion_progress is None
            assert r['recovery_completion_progress']==goal
            assert r['selected_side']==r['adjacent_link_id']==''
            assert r['entry_points']==r['hold_points']==r['recovery_points']==[]
            break
        assert p._recovery_completion_progress==goal
    assert phases[:5]==['RECOVERING']*5
    assert phases[5]=='CLEAR'
    assert plan(p,progress+.05,-offset,yaw,speed=3.)['status']=='CLEAR'


@pytest.mark.parametrize('offset,yaw',[(.150001,0.),(.03,.01)])
def test_recovery_outside_lateral_or_existing_alignment_bound_cannot_clear(offset,yaw):
    p,goal=recovering()
    for i in range(8):
        r=plan(p,goal+.01+i/24.,-offset,yaw,speed=3.)
        assert r['status']=='RECOVERING'
        assert not r['recovery_acquired']
        assert p._recovery_completion_progress==goal


@pytest.mark.parametrize('fault',['clearance','corridor','direction'])
def test_unsafe_recovery_correction_cannot_complete_the_phase(fault):
    p,goal=recovering();x=goal+.01;y=.01 if fault=='corridor' else -.001
    yaw=math.pi if fault=='direction' else 0.
    obs=obstacle()+[(x,y)] if fault=='clearance' else obstacle()
    r=plan(p,x,y,yaw,obstacles=obs,speed=3.)
    assert r['status']=='BLOCKED'
    assert p._phase=='RECOVERING'
    assert p._recovery_completion_progress==goal
    assert not r.get('recovery_acquired',False)


def test_reset_and_new_obstacle_discard_old_recovery_acquisition_goal():
    p,goal=recovering()
    r=plan(p,22.64,-3.499,current='R',speed=3.,obstacles=obstacle(28.,32.))
    assert r['status']=='AVOIDING'
    assert p._recovery_completion_progress is None
    p.reset()
    assert p._recovery_completion_progress is None
