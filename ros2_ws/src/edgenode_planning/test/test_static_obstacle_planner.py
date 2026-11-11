"""Small contract tests for strictly sequential LiDAR avoidance stages."""
from dataclasses import replace
import math
from unittest.mock import patch

import pytest

from edgenode_planning.static_obstacle_planner import AvoidanceConfig,StaticObstaclePlanner
from edgenode_planning.lattice_path import transition,curvatures,control_local_path
from avoidance_world import world


def setup(side='left',**options):
    lanes,point,obstacle=world(side,**options)
    return StaticObstaclePlanner(lanes),point,obstacle


def plan(planner,point,obstacles,s=0.,offset=0.,yaw=0.,speed=3.,valid=True,link='O0',now=0.,**kwargs):
    reference=[point(s+i*.5,0.) for i in range(101)]
    return planner.plan(reference,obstacles,(*point(s,offset)[:2],yaw),speed_kmh=speed,
        current_link_id=link,route_link_ids=['O0','O1'],observation_valid=valid,
        now_sec=now,global_reference=[point(-10+i*.5,0.) for i in range(301)],**kwargs)


def enter(p,point,obstacle,**kwargs):
    assert plan(p,point,[obstacle()],**kwargs)['state']=='DETECT'
    result=plan(p,point,[],**kwargs)
    assert result['state']=='AVOID_ENTRY',[(c.transition_length_m,c.rejection) for c in p.candidates]
    return result


def acquire(p,point,offset,now=1.):
    end=p.selected.entry_end_s-10.
    for _ in range(p.config.acquisition_confirmations):
        result=plan(p,point,[],s=end-.5,offset=offset,now=now)
    assert result['state']=='AVOID_HOLD'
    return result


def test_no_obstacle_preserves_normal_path_identity():
    p,point,_=setup();reference=[point(i,0.) for i in range(21)]
    result=p.plan(reference,[],(0.,0.,0.))
    assert result['points'] is reference and result['state']=='NORMAL' and not result['active']


@pytest.mark.parametrize('side,offset',[('left',3.5),('right',-3.5)])
def test_one_stage_per_call_no_return_until_timed_safe_hold(side,offset):
    p,point,obstacle=setup(side)
    with patch.object(p,'_generate_entry',wraps=p._generate_entry) as entry,patch.object(p,'_generate_return',wraps=p._generate_return) as recovery:
        r=plan(p,point,[obstacle()])
        assert r['state']=='DETECT' and entry.call_count==recovery.call_count==0
        assert not p.candidates and not r['return_path'] and r['detected_obstacles']
        r=plan(p,point,[])
        assert r['state']=='AVOID_ENTRY' and not recovery.called
        assert p._hold_path==() and all(not c.return_path and math.isnan(c.return_end_s) for c in p.candidates)
        generated=p.generated_transition_count
        acquire(p,point,offset)
        assert p._hold_path==() and not recovery.called
        r=plan(p,point,[],s=45.,offset=offset,now=2.)
        assert r['state']=='AVOID_HOLD' and r['reason']=='HOLD_TIMER'
        assert p.generated_transition_count==generated
        # Time alone is insufficient if the rear body has not passed the solid.
        p2,point2,o2=setup(side);enter(p2,point2,o2);acquire(p2,point2,offset)
        r2=plan(p2,point2,[],s=30.,offset=offset,now=5.)
        assert r2['state']=='AVOID_HOLD' and r2['reason']=='WAIT_OBSTACLE_PASS'
        r=plan(p,point,[],s=45.,offset=offset,now=5.)
        assert r['state']=='RETURN' and not r['return_path'] and not recovery.called
        assert p.return_commits==0
        r=plan(p,point,[],s=45.,offset=offset,now=5.1)
        assert r['state']=='RETURN' and r['return_path'] and recovery.call_count==1
        assert p.return_commits==1
        # Completion uses the saved Global Path station, position and heading.
        end=p.selected.return_end_s-10.
        for _ in range(3):r=plan(p,point,[],s=end-.5,now=6.)
        assert r['state']=='NORMAL' and not r['return_path']
        assert r['entry_commits']==r['return_commits']==1


def test_return_permission_is_checked_only_in_return_stage():
    p,point,obstacle=setup()
    for key in ('A0','A1'):p.network.links[key]['can_move_right_lane']=False
    enter(p,point,obstacle);acquire(p,point,3.5)
    assert plan(p,point,[],s=45.,offset=3.5,now=5.)['state']=='RETURN'
    r=plan(p,point,[],s=45.,offset=3.5,now=5.1)
    assert r['state']=='RETURN' and r['reason']=='NO_SAFE_RETURN' and not r['blocked']
    assert not r['return_path'] and p.return_commits==0
    assert all(c.rejection=='LANE_CHANGE_FORBIDDEN' for c in p.candidates)


def test_future_return_obstacle_does_not_gate_entry():
    p,point,obstacle=setup()
    assert plan(p,point,[obstacle(),obstacle(70.,74.)])['state']=='DETECT'
    r=plan(p,point,[])
    assert r['state']=='AVOID_ENTRY'
    assert not r['return_path'] and p.return_commits==0


def test_hold_requires_surround_body_clearance_and_rechecks_new_lidar():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    near=obstacle(44.,46.,1.5,1.8)
    r=plan(p,point,[near],s=45.,offset=3.5,now=5.)
    assert r['state']=='AVOID_HOLD' and r['reason']=='WAIT_SURROUND_CLEARANCE'
    assert p.return_commits==0


def test_new_collision_and_stale_observation_latch_stop():
    p,point,obstacle=setup();enter(p,point,obstacle);fixed=p.selected
    r=plan(p,point,[obstacle(18.,22.,3.,4.)])
    assert r['blocked'] and p.selected is fixed
    assert plan(p,point,[])['blocked']
    p,point,obstacle=setup();enter(p,point,obstacle)
    assert plan(p,point,[],valid=False)['reason']=='OBSTACLE_OBSERVATION_INVALID_OR_STALE'


@pytest.mark.parametrize('damage',['no_neighbor','opposite','forbidden'])
def test_invalid_entry_topology_or_permission_stops(damage):
    p,point,obstacle=setup()
    if damage=='no_neighbor':p.network.valid.remove('A0')
    elif damage=='opposite':p.network.links['A0']['points'].reverse()
    else:p.network.links['O0']['can_move_left_lane']=False
    assert plan(p,point,[obstacle()])['state']=='DETECT'
    r=plan(p,point,[])
    assert r['blocked'] and not r['points']


def test_normal_measured_body_collision_and_braking_deadline():
    p,point,obstacle=setup()
    assert plan(p,point,[obstacle(1.,3.,1.4,1.8)],offset=.3)['reason']=='BODY_COLLISION'
    p,point,obstacle=setup()
    assert plan(p,point,[obstacle(5.,8.)],speed=10.)['reason']=='BRAKING_DISTANCE'


def test_unchanged_controller_limits_and_fixed_entry_geometry():
    p,point,obstacle=setup();enter(p,point,obstacle)
    fixed=p.selected.entry_path;before=tuple(fixed)
    first=control_local_path(fixed,(0.,0.,0.));second=control_local_path(fixed,(1.,.01,0.))
    assert fixed==before and first[0][0]<second[0][0]
    assert p._rollout(fixed,(0.,0.,0.),3.,p.config.curvature_limit+.001,p.remembered)=='ACTUAL_CURVATURE'
    p._control_horizon_m=3.
    assert p._rollout(fixed,(0.,0.,0.),3.,0.,p.remembered)=='SHORT_CONTROL_HORIZON'
    assert p._body_safety((0.,0.,0.),(0.,0.,0.),(tuple(obstacle(-.5,.5,1.19,2.)),))=='REFERENCE_CLEARANCE'
    short=transition((0.,0.),0.,0.,[(0.,3.5,0.),(30.,3.5,0.)],5.,5.)
    assert max(map(abs,curvatures(short)))>p.config.curvature_limit


@pytest.mark.parametrize('changes',[{'hold_duration_s':-1.},{'hold_clearance_m':-1.},{'wheelbase_m':0.},{'steering_sign':0.}])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):replace(AvoidanceConfig(),**changes)


def test_detection_accepts_bounded_local_reference_start_ahead_of_pose():
    p,point,obstacle=setup()
    reference=[point(.001+i*.5,0.) for i in range(81)]
    result=p.plan(reference,[obstacle(21.,25.)],(0.,0.,0.),speed_kmh=0.,now_sec=0.)
    assert result['state']=='DETECT' and not result['candidates']


def test_entry_uses_forward_neighbor_support_when_start_sections_differ():
    from edgenode_planning.lane_geometry import LanePair
    p,_,_=setup()
    pair=LanePair([(float(i),0.,0.) for i in range(61)],
                  [(float(i),-3.5,0.) for i in range(1,61)],
                  [('O0','A0')],'right',[60.])
    assert pair.at(0.) is None
    candidates=p._generate_entry(pair,(0.,0.,0.),3.,0.,35.)
    assert any(not c.rejection for c in candidates)
    assert all(not c.return_path for c in candidates)


def test_contained_lidar_jitter_keeps_validated_conservative_solid():
    p,point,obstacle=setup();enter(p,point,obstacle)
    remembered=p.remembered
    shifted=[(x+.01,y-.01) for x,y in obstacle()]
    assert not p.needs_validation_stop([point(i,0.) for i in range(51)],[shifted])
    p._remember([shifted])
    assert p.remembered==remembered
    assert p.needs_validation_stop([point(i,0.) for i in range(51)],[obstacle(18.,22.,3.,4.)])


def test_progress_clipping_does_not_invent_new_path_curvature():
    from edgenode_planning.lattice_path import fixed_path_suffix
    p,_,_=setup()
    points=tuple((8.*math.sin(i*.025),8.*(1-math.cos(i*.025)),0.) for i in range(60))
    clipped=fixed_path_suffix(points,.199)
    assert max(map(abs,curvatures(points)))<p.config.curvature_limit
    assert p._path_safety(clipped,(),3.,curvature_points=points)==''


def test_new_clear_roadside_observation_does_not_restart_motion_validation():
    p,point,obstacle=setup();enter(p,point,obstacle)
    far=obstacle(24.,28.,-6.,-5.)
    assert not p.needs_validation_stop([point(i,0.) for i in range(51)],[far])
    result=plan(p,point,[far])
    assert result['state']=='AVOID_ENTRY' and p.return_commits==0
