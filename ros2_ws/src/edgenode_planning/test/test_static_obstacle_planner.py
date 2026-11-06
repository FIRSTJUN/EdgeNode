from dataclasses import replace
import math

import pytest

from edgenode_planning.static_obstacle_planner import AvoidanceConfig,StaticObstaclePlanner
from edgenode_planning.lattice_path import transition,curvatures,reference_cost,control_local_path
from avoidance_world import world


def setup(side='left',road=True,**options):
    lanes,area,point,obstacle=world(side,**options)
    planner=StaticObstaclePlanner(lanes,drivable_area=area if road else None)
    return planner,point,obstacle


def plan(planner,point,obstacles,s=0.,offset=0.,yaw=0.,speed=3.,valid=True,link='O0'):
    reference=[point(s+i*.5,0.) for i in range(101)]
    return planner.plan(reference,obstacles,(*point(s,offset)[:2],yaw),speed_kmh=speed,
        current_link_id=link,route_link_ids=['O0','O1'],observation_valid=valid)


def test_no_obstacle_returns_identical_normal_local_path_even_without_boundaries():
    p,point,obstacle=setup(road=False)
    reference=[point(i,0) for i in range(21)]
    r=p.plan(reference,[],(0.,0.,0.),current_link_id='O0',route_link_ids=['O0','O1'])
    assert r['state']=='NORMAL' and r['points'] is reference and not r['active']


def test_normal_tracking_error_cannot_hide_actual_body_collision_beside_reference():
    p,point,obstacle=setup(road=False)
    # At the reference center this object clears both existing safety floors.
    # The measured +0.3 m tracking error puts the body within the 0.25 m margin.
    groups=[obstacle(1.,3.,1.4,1.8)]
    r=plan(p,point,groups,offset=.3)
    assert r['blocked'] and not r['points'] and r['reason']=='BODY_COLLISION'


@pytest.mark.parametrize('side',['left','right'])
def test_boundary_unknown_still_generates_real_lane_candidates_and_stops(side):
    p,point,obstacle=setup(side,road=False)
    r=plan(p,point,[obstacle()])
    assert r['blocked'] and not r['points'] and r['reason']=='ROAD_BOUNDARY_UNAVAILABLE'
    assert len(r['candidates'])==3
    assert all(c.points and c.rejection for c in r['candidates'])
    assert all(c.side==side for c in r['candidates'])
    assert plan(p,point,[])['blocked']  # Detection loss does not release STOP.


@pytest.mark.parametrize('side,offset',[('left',3.5),('right',-3.5)])
def test_fixed_lifecycle_requires_physical_acquisition_and_full_rear_clearance(side,offset):
    p,point,obstacle=setup(side)
    r=plan(p,point,[obstacle()]);assert r['state']=='AVOID_ENTRY'
    selected=p.selected;fixed=r['points'];end=selected.entry_end_s-10.
    for _ in range(3):
        assert plan(p,point,[])['points'] is fixed
    for _ in range(2):
        assert plan(p,point,[],s=end-.5,offset=offset)['state']=='AVOID_ENTRY'
    r=plan(p,point,[],s=end-.5,offset=offset)
    assert r['state']=='AVOID_HOLD'
    assert plan(p,point,[],s=40.5,offset=offset)['state']=='AVOID_HOLD'
    r=plan(p,point,[],s=40.7,offset=offset,link='A1')
    assert r['state']=='RETURN'
    recovery=r['points'];assert p.selected is selected
    for _ in range(3):assert plan(p,point,[],s=41.,offset=offset,link='O1')['points'] is recovery
    back_end=selected.return_end_s-10.
    for _ in range(3):r=plan(p,point,[],s=back_end-.5,offset=0.,link='O1')
    assert r['state']=='NORMAL'
    assert r['entry_commits']==r['return_commits']==1


@pytest.mark.parametrize('yaw,offset',[(.3,3.5),(math.pi,3.5),(0.,3.0)])
def test_near_center_heading_lateral_and_reverse_counterexamples_do_not_acquire(yaw,offset):
    p,point,obstacle=setup()
    plan(p,point,[obstacle()]);end=p.selected.entry_end_s-10.
    for _ in range(3):r=plan(p,point,[],s=end-.5,offset=offset,yaw=yaw)
    assert r['state'] in ('AVOID_ENTRY','BLOCKED')


def test_confirmation_interruption_resets_count_and_not_path():
    p,point,obstacle=setup();plan(p,point,[obstacle()]);fixed=p.selected
    end=p.selected.entry_end_s-10.
    plan(p,point,[],s=end-.5,offset=3.5);assert p.confirmations==1
    p.invalidate_observation();assert p.confirmations==0
    plan(p,point,[],s=end-.5,offset=3.5);assert p.state=='AVOID_ENTRY' and p.selected is fixed


@pytest.mark.parametrize('damage',['no_neighbor','opposite','forbidden','disconnected'])
def test_invalid_lane_topology_or_permission_is_never_approved(damage):
    p,point,obstacle=setup()
    if damage=='no_neighbor':p.network.valid.remove('A0')
    elif damage=='opposite':p.network.links['A0']['points'].reverse()
    elif damage=='forbidden':p.network.links['O0']['can_move_left_lane']=False
    else:p.network.links['A1']['from_node_idx']='disconnected'
    r=plan(p,point,[obstacle()]);assert r['blocked'] and not r['points']


def test_new_target_obstacle_stops_committed_direction_without_refitting():
    p,point,obstacle=setup();plan(p,point,[obstacle()]);fixed=p.selected
    r=plan(p,point,[obstacle(24,28,3.,4.)],s=2.)
    assert r['blocked'] and p.selected is fixed and p.entry_commits==1


def test_detection_loss_retains_obstacle_and_stale_observation_stops():
    p,point,obstacle=setup();plan(p,point,[obstacle()]);remembered=p.remembered
    assert plan(p,point,[])['state']=='AVOID_ENTRY' and p.remembered==remembered
    r=plan(p,point,[],valid=False)
    assert r['blocked'] and r['reason']=='OBSTACLE_OBSERVATION_INVALID_OR_STALE'


def test_unmeasured_straight_turn_state_after_pose_jump_is_rejected_by_wheel_rate():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    r=plan(p,point,[],s=5.)
    assert r['blocked'] and r['reason']=='STEERING_RATE'


def test_short_control_window_does_not_validate_a_longer_invisible_path():
    p,point,obstacle=setup()
    reference=[point(i*.5,0.) for i in range(101)]
    r=p.plan(reference,[obstacle()],(0.,0.,0.),speed_kmh=3.,
        current_link_id='O0',route_link_ids=['O0','O1'],control_horizon_m=3.)
    assert r['blocked'] and not r['points']
    # The shortest seed may already fail the earlier solid-body road check.
    # Candidates passing geometry must still reject the invisible PP horizon.
    assert all(c.rejection for c in r['candidates'])
    assert any(c.rejection=='SHORT_CONTROL_HORIZON' for c in r['candidates'])


def test_measured_turn_beyond_production_clamp_is_not_replaced_by_planned_curvature():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    assert p._rollout(p.selected.entry_path,(0.,0.,0.),3.,
        p.config.curvature_limit+1e-4,p.remembered)=='ACTUAL_CURVATURE'


def test_control_window_clips_progress_without_modifying_fixed_geometry():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    fixed=p.selected.entry_path;before=tuple(fixed)
    first=control_local_path(fixed,(0.,0.,0.),20.)
    second=control_local_path(fixed,(1.,.01,0.),20.)
    assert fixed is p.selected.entry_path and fixed==before
    assert first[0][0]<second[0][0] and first[-1][0]<second[-1][0]
    assert len(first)<len(fixed) and len(second)<len(fixed)


def test_late_detection_and_both_lanes_obstructed_stop():
    p,point,obstacle=setup()
    assert plan(p,point,[obstacle(5.,8.)],speed=10.)['reason']=='BRAKING_DISTANCE'
    p,point,obstacle=setup()
    r=plan(p,point,[obstacle(),obstacle(30,44,2.5,4.5)])
    assert r['blocked']


def test_two_obstacles_extend_pass_extent_and_prevent_premature_return():
    p,point,obstacle=setup()
    r=plan(p,point,[obstacle(30,33),obstacle(39,42)])
    assert not r['blocked'] and p.selected.return_start_s>=42+10+1.109+.5


def test_obstacle_rear_extension_does_not_release_old_return_gate():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    end=p.selected.entry_end_s-10.
    for _ in range(3):plan(p,point,[],s=end-.5,offset=3.5)
    r=plan(p,point,[obstacle(35.,44.)],s=41.,offset=3.5)
    assert r['state']=='AVOID_HOLD'
    assert p.return_commits==0


def test_target_lane_forbidding_return_is_not_a_viable_maneuver():
    p,point,obstacle=setup();p.network.links['A0']['can_move_right_lane']=False
    assert plan(p,point,[obstacle()])['blocked']


def test_reference_clearance_floor_is_preserved_in_addition_to_full_body_margin():
    p,point,obstacle=setup()
    assert p._body_safety((0.,0.,0.),(0.,0.,0.),
        (tuple(obstacle(-.5,.5,1.19,2.)),),False)=='REFERENCE_CLEARANCE'
    with pytest.raises(ValueError):replace(AvoidanceConfig(),reference_clearance_m=1.19)
    with pytest.raises(ValueError):replace(AvoidanceConfig(),obstacle_margin_m=.24)


def test_second_independent_obstacle_uses_new_lane_pair_progress_frame():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    end=p.selected.entry_end_s-10.
    for _ in range(3):plan(p,point,[],s=end-.5,offset=3.5)
    plan(p,point,[],s=40.7,offset=3.5)
    end=p.selected.return_end_s-10.
    for _ in range(3):result=plan(p,point,[],s=end-.5)
    assert result['state']=='NORMAL'
    result=plan(p,point,[obstacle(95.,98.)],s=70.,link='O1')
    assert result['state']=='AVOID_ENTRY',result['reason']
    assert p.pair.ids[0][0]=='O1'


def test_braking_distance_uses_actual_yawed_front_corner_not_axle_plus_nominal_length():
    p,point,obstacle=setup()
    result=plan(p,point,[obstacle(6.9,8.)],yaw=math.atan2(.93,3.79),speed=10.)
    assert result['blocked'] and result['reason']=='BRAKING_DISTANCE'


def test_cost_prefers_earlier_safe_original_return_on_equal_horizon():
    reference=[(i*.5,0.,0.) for i in range(201)]
    def maneuver(return_start):
        entry=transition((0.,0.),0.,0.,[(0.,3.5,0.),(100.,3.5,0.)],20.,20.)
        hold=tuple((20.+i*.5,3.5,0.) for i in range(1,round((return_start-20)*2)+1))
        back=transition((return_start,3.5),0.,0.,reference,return_start+20,20.)
        tail=tuple((return_start+20+i*.5,0.,0.) for i in range(1,round((100-return_start-20)*2)+1))
        return entry+hold+back[1:]+tail
    adjacent=[(0.,3.5,0.),(100.,3.5,0.)]
    assert reference_cost(maneuver(40),reference,adjacent)[0]<reference_cost(maneuver(60),reference,adjacent)[0]


def test_shorter_adjacent_chain_cannot_compete_on_a_truncated_prediction_support():
    p,point,obstacle=setup(two_sides=True)
    p.network.links['A1']['can_move_right_lane']=False
    r=plan(p,point,[obstacle(25.,28.)])
    assert r['state']=='AVOID_ENTRY' and r['selected_side']=='right'
    left=[c for c in r['candidates'] if c.side=='left']
    assert left and all(c.rejection=='INSUFFICIENT_COMMON_HORIZON' for c in left)
    safe=[c for c in r['candidates'] if not c.rejection]
    supports=[p.pair.original_index.project(c.points[-1])-p.pair.original_index.project(c.points[0]) for c in safe]
    assert supports and all(s==pytest.approx(80.) for s in supports)


def test_short_transition_is_rejected_by_unchanged_curvature_limit():
    p=transition((0.,0.),0.,0.,[(0.,3.5,0.),(30.,3.5,0.)],5.,5.)
    assert max(map(abs,curvatures(p)))>AvoidanceConfig().curvature_limit


@pytest.mark.parametrize('side,origin,yaw', [('left',(120.,-80.),.7),('right',(-90.,42.),-1.2)])
def test_candidate_generation_is_not_tied_to_s4_coordinates(side,origin,yaw):
    p,point,obstacle=setup(side,origin=origin,yaw=yaw)
    r=plan(p,point,[obstacle(front=32.,rear=37.,low=-.7,high=.7)],yaw=yaw)
    assert r['state']=='AVOID_ENTRY',[(c.rejection,c.transition_length_m) for c in p.candidates]


@pytest.mark.parametrize('side',['left','right'])
def test_gentle_curve_candidates_use_actual_adjacent_centerline(side):
    p,point,obstacle=setup(side,radius=300.)
    r=plan(p,point,[obstacle()])
    assert r['state']=='AVOID_ENTRY',[(c.rejection,c.transition_length_m) for c in p.candidates]


@pytest.mark.parametrize('changes',[{'wheelbase_m':0.},{'road_margin_m':-1.},
    {'rollout_seconds':math.inf},{'steering_sign':0.},{'max_front_steer_normalized':1.1},
    {'control_rate_hz':0.}])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):replace(AvoidanceConfig(),**changes)
