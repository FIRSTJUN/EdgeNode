"""Actual lane centers, speed-dependent entry, HOLD/recovery and three gates."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner, WorldTransition
from edgenode_planning.lane_geometry import LaneNetwork, corridor_covers
from edgenode_planning.dijkstra_planner import DijkstraPlanner


def network(sides=('left','right'), separation=3.5):
    links={}
    for name,y in [('O',0.),('L',separation),('R',-separation)]:
        if name=='L' and 'left' not in sides or name=='R' and 'right' not in sides:
            continue
        links[name]=dict(idx=name,from_node_idx=name+'0',to_node_idx=name+'1',max_speed=30,
            lazy_init=False,lane_ch_link_path=[],points=[[i*.5,y,0.] for i in range(161)],
            left_lane_change_dst_link_idx=('L' if 'left' in sides else None) if name=='O' else 'O' if name=='R' else None,
            right_lane_change_dst_link_idx=('R' if 'right' in sides else None) if name=='O' else 'O' if name=='L' else None)
    return LaneNetwork(SimpleNamespace(links_by_id=links,graph={'x':[{'link_id':i} for i in links]}))


def road(x=0.,length=20.):
    return [[x+i*.5,0.,0.] for i in range(round(length/.5)+1)]


def obstacle(front=18.,rear=22.):
    return [(front,-.5),((front+rear)/2,-.5),(rear,-.5),(rear,0.),
            (rear,.5),((front+rear)/2,.5),(front,.5),(front,0.)]


def plan(p,x=0.,y=0.,yaw=0.,obstacles=None,current='O',speed=20.):
    return p.plan(road(x),obstacle() if obstacles is None else obstacles,(x,y),current_heading=yaw,
                  obstacle_groups=None,road_progress_m=x,current_link_id=current,route_link_ids=['O'],speed_kmh=speed)


@pytest.mark.parametrize('side,target',[('left',3.5),('right',-3.5)])
def test_entry_reaches_actual_adjacent_center_beyond_legacy_offsets(side,target):
    p=AdjacentLanePlanner(network((side,)))
    result=plan(p)
    assert result['status']=='AVOIDING'
    assert result['selected_side']==side
    assert result['adjacent_separation']==pytest.approx(target)
    assert result['points'][-1][1]==pytest.approx(target)
    assert result['entry_length']>=20/3.6*3
    assert result['full_offset_start_s']<result['obstacle_front_s']
    assert result['maximum_curvature']<=.16046692048050848
    assert result['minimum_clearance_m']>=1.2
    assert result['drivable_safe']
    assert len(result['recovery_gate_points'])==2
    assert all(p[0]==pytest.approx(22.5) for p in result['recovery_gate_points'])
    assert result['points'][0][:2]==[0.,0.]


def test_no_adjacent_lane_fails_closed():
    result=plan(AdjacentLanePlanner(network(())))
    assert result['status']=='BLOCKED'
    assert result['blocked_reason']=='NO_ADJACENT_LANE'


@pytest.mark.parametrize('damage',['opposite','no_reciprocal','non_drivable','missing_relation'])
def test_unknown_or_wrong_direction_adjacent_is_rejected(damage):
    n=network(('left',))
    if damage=='opposite':n.links['L']['points'].reverse()
    elif damage=='no_reciprocal':n.links['L']['right_lane_change_dst_link_idx']=None
    elif damage=='non_drivable':n.valid.remove('L')
    else:n.links['O']['left_lane_change_dst_link_idx']=None
    assert plan(AdjacentLanePlanner(n))['status']=='BLOCKED'


def test_minimum_entry_duration_uses_speed_and_cannot_miss_front_deadline():
    slow=plan(AdjacentLanePlanner(network(('left',))),speed=5.)
    fast=plan(AdjacentLanePlanner(network(('left',))),speed=20.)
    assert fast['entry_length']>slow['entry_length']
    blocked=plan(AdjacentLanePlanner(network(('left',))),obstacles=obstacle(10.,14.))
    assert blocked['status']=='BLOCKED'
    assert 'ENTRY_DEADLINE' in blocked['evaluated_candidate_reject_reason']
    assert blocked['adjacent_link_id']==''
    assert blocked['adjacent_lane_points']==[]
    assert blocked['lane_change_phase']=='BLOCKED'


def test_entry_suffix_preserves_world_boundary_heading_curvature_and_progress():
    p=AdjacentLanePlanner(network(('left',)))
    first=plan(p)
    transition=p._world_transition
    initial_end=p._transition_end_progress
    for progress in (3.,7.,12.):
        pos,velocity,acceleration=transition.state(progress)
        yaw=math.atan2(velocity[1],velocity[0])
        result=plan(p,progress,pos[1],yaw,current='L')
        assert result['status']=='AVOIDING'
        assert p._transition_end_progress==pytest.approx(initial_end)
        assert p._maneuver_start_global_progress_m==0.
        assert p._maneuver_progress_m==pytest.approx(progress)
        assert result['points'][0][:2]==pytest.approx(pos,abs=1e-8)
        actual=p._world_transition.state(0.)
        assert actual[1]==pytest.approx(velocity,abs=1e-7)
        assert actual[2]==pytest.approx(acceleration,abs=1e-7)
        for distance in (0.,1.,p._world_transition.length):
            assert p._world_transition.state(distance)[0]==pytest.approx(transition.state(progress+distance)[0],abs=1e-7)


def test_adjacent_hold_is_exact_map_center_and_does_not_recover_before_rear_margin():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p)
    for x in (17.,19.,22.5):
        result=plan(p,x,-3.5,current='R')
        assert result['status']=='AVOIDING'
        assert result['lane_change_phase']=='ADJACENT_LANE_HOLD'
        assert result['entry_length']==0.
        assert all(q[1]==-3.5 for q in result['points'])
    recovering=plan(p,22.6,-3.5,current='R')
    assert recovering['status']=='RECOVERING'
    assert recovering['points'][0][1]==-3.5
    assert recovering['points'][-1][1]==pytest.approx(0.)
    assert recovering['maximum_curvature']<=p.usable_curvature_limit
    assert recovering['drivable_safe']


@pytest.mark.parametrize('side,target', [('left',3.5),('right',-3.5)])
@pytest.mark.parametrize('error,yaw_error', [(0.,0.),(.001,0.),(.03,.005)])
def test_terminal_tracking_error_acquires_hold_without_chasing_correction_endpoint(side,target,error,yaw_error):
    p=AdjacentLanePlanner(network((side,)))
    current='L' if side=='left' else 'R'
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    sign=math.copysign(1.,target)
    phases=[]
    for cycle in range(8):
        progress=completion-.2+cycle*(3./3.6/20.)
        result=plan(p,progress,target-sign*error,-sign*yaw_error,current=current,speed=3.)
        assert result['status']=='AVOIDING'
        assert result['drivable_safe']
        assert result['minimum_clearance_m']>=p.required_clearance_m
        assert result['maximum_curvature']<=p.usable_curvature_limit
        assert p._entry_completion_progress==completion
        phases.append(result['lane_change_phase'])
        if error:
            # The current correction still rebases; its endpoint is not the
            # fixed maneuver completion criterion. No exact-center shortcut.
            assert result['entry_length']>0.
            assert p._transition_start_progress==progress
            assert p._transition_end_progress==pytest.approx(progress+result['entry_length'])
    if error:
        assert phases[:5]==['ENTRY']*5
        assert phases[5:]==['ADJACENT_LANE_HOLD']*3
        assert p._transition_end_progress>completion
    else:
        assert phases==['ADJACENT_LANE_HOLD']*8


@pytest.mark.parametrize('side,target', [('left',3.5),('right',-3.5)])
@pytest.mark.parametrize('error', [.151,.20])
def test_passing_entry_goal_outside_acquisition_tolerance_stays_in_entry(side,target,error):
    p=AdjacentLanePlanner(network((side,)))
    current='L' if side=='left' else 'R'
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    for cycle in range(12):
        progress=completion+cycle*(3./3.6/20.)
        result=plan(p,progress,target-math.copysign(error,target),current=current,speed=3.)
        assert result['status']=='AVOIDING'
        assert result['lane_change_phase']=='ENTRY'
        assert p._entry_completion_progress==completion


def test_terminal_correction_carries_world_boundary_when_acquiring_hold():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    plan(p,completion-.1,-3.499,.005,current='R',speed=3.)
    previous=p._world_transition
    position=[completion+.01,-3.499]
    _,velocity,acceleration=previous.state(previous.project_parameter(position))
    speed=math.hypot(*velocity)
    curvature=(velocity[0]*acceleration[1]-velocity[1]*acceleration[0])/speed**3
    result=plan(p,position[0],position[1],.005,current='R',speed=3.)
    assert result['lane_change_phase']=='ADJACENT_LANE_HOLD'
    assert p._world_transition is not previous
    start,first,second=p._world_transition.state(0.)
    assert start==pytest.approx(position)
    assert math.atan2(first[1],first[0])==pytest.approx(.005)
    assert math.hypot(*first)==pytest.approx(speed)
    assert (first[0]*second[1]-first[1]*second[0])/math.hypot(*first)**3==pytest.approx(curvature)
    assert sum(a*b for a,b in zip(first,second))/math.hypot(*first)==pytest.approx(
        sum(a*b for a,b in zip(velocity,acceleration))/speed)
    assert p._transition_start_progress==position[0]
    assert p._transition_end_station==pytest.approx(position[0]+result['entry_length'])
    assert p._transition_last_parameter==0.
    assert p._entry_completion_progress==completion


@pytest.mark.parametrize('fault', ['clearance','corridor','reversed_heading'])
def test_lane_acquisition_cannot_approve_an_unsafe_hold_correction(fault):
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    progress=completion+.01
    y=-3.51 if fault=='corridor' else -3.499
    yaw=math.pi if fault=='reversed_heading' else 0.
    obstacles=obstacle()+[(progress,y)] if fault=='clearance' else obstacle()
    result=plan(p,progress,y,yaw,obstacles=obstacles,current='R',speed=3.)
    assert result['status']=='BLOCKED'
    assert result['lane_change_phase']=='BLOCKED'
    assert p._phase=='ENTRY'  # No unsafe phase approval or maneuver reset.
    assert p._entry_completion_progress==completion
    expected={'clearance':['CLEARANCE'],'corridor':['DRIVABLE_CORRIDOR'],
              'reversed_heading':['CURVATURE','DIRECTION_CONTINUITY']}[fault]
    assert all(reason in result['evaluated_candidate_reject_reason'] for reason in expected)


@pytest.mark.parametrize('side,target', [('left',3.5),('right',-3.5)])
def test_small_error_entry_hold_recovery_and_clear_keep_existing_rear_gate(side,target):
    p=AdjacentLanePlanner(network((side,)))
    current='L' if side=='left' else 'R'
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    y=target-math.copysign(.001,target)
    for cycle in range(8):
        result=plan(p,completion-.2+cycle*(3./3.6/20.),y,current=current,speed=3.)
    assert result['lane_change_phase']=='ADJACENT_LANE_HOLD'
    pair=p._lane_pair
    # Follow the approved HOLD correction to its center before the rear gate.
    assert plan(p,17.,target,current=current,speed=3.)['lane_change_phase']=='ADJACENT_LANE_HOLD'
    assert plan(p,22.5,target,current=current,speed=3.)['lane_change_phase']=='ADJACENT_LANE_HOLD'
    recovering=plan(p,22.6,target,current=current,speed=3.)
    assert recovering['status']=='RECOVERING'
    assert recovering['lane_change_phase']=='RECOVERING'
    assert recovering['drivable_safe']
    assert recovering['maximum_curvature']<=p.usable_curvature_limit
    assert recovering['minimum_clearance_m']>=p.required_clearance_m
    assert p._lane_pair is pair
    endpoint=p._world_transition.end
    tangent=p._world_transition.end_first
    cleared=plan(p,endpoint[0],endpoint[1],math.atan2(tangent[1],tangent[0]),current='O',speed=3.)
    assert cleared['status']=='CLEAR'
    assert cleared['lane_change_phase']=='CLEAR'
    assert p._lane_pair is None
    assert p._world_transition is None
    assert p._entry_completion_progress is None


def test_unsafe_recovery_launch_defers_to_a_freshly_checked_hold_correction():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    plan(p,17.,-3.5,current='R',speed=3.)
    plan(p,17.1,-3.499,current='R',speed=3.)
    plan(p,22.5,-3.499,current='R',speed=3.)
    result=plan(p,22.6,-3.499,current='R',speed=3.)
    assert result['status']=='AVOIDING'
    assert result['lane_change_phase']=='ADJACENT_LANE_HOLD'
    assert p._maneuver_context['recovery_deferred']
    assert 'DRIVABLE_CORRIDOR' in p._maneuver_context['recovery_reject_reason']
    assert result['maximum_curvature']<=p.usable_curvature_limit
    assert result['minimum_clearance_m']>=p.required_clearance_m
    assert result['drivable_safe']


def test_new_obstacle_interrupting_recovery_does_not_reuse_the_old_entry_goal():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    plan(p,17.,-3.5,current='R',speed=3.)
    assert plan(p,22.6,-3.5,current='R',speed=3.)['status']=='RECOVERING'
    result=plan(p,22.64,-3.499,current='R',speed=3.,obstacles=obstacle(28.,32.))
    assert result['status']=='AVOIDING'
    assert result['lane_change_phase']=='ENTRY'
    assert p._entry_completion_progress>completion
    assert p._entry_completion_progress==pytest.approx(22.64+result['entry_length'])


def test_route_reset_forgets_the_entry_acquisition_goal():
    p=AdjacentLanePlanner(network(('right',)))
    plan(p,speed=3.)
    completion=p._entry_completion_progress
    p.reset()
    assert p._entry_completion_progress is None
    result=plan(p,10.,obstacles=obstacle(28.,32.),speed=3.)
    assert result['lane_change_phase']=='ENTRY'
    assert p._entry_completion_progress==pytest.approx(10.+result['entry_length'])
    assert p._entry_completion_progress>completion


def test_obstacle_in_adjacent_lane_cannot_be_ignored_as_far_metadata():
    p=AdjacentLanePlanner(network(('left',)))
    barrier=[(18.,3.+i*.1) for i in range(11)]
    result=plan(p,obstacles=obstacle()+barrier)
    assert result['status']=='BLOCKED'
    assert 'CLEARANCE' in result['evaluated_candidate_reject_reason']


def test_whole_segment_corridor_gate_rejects_excursion_and_hole_crossing():
    quads=[[(0.,0.),(10.,0.),(10.,3.5),(0.,3.5)]]
    assert corridor_covers([[0.,0.],[5.,2.],[10.,3.5]],quads)
    assert not corridor_covers([[0.,0.],[5.,4.],[10.,3.5]],quads)
    disjoint=[[(0.,0.),(4.,0.),(4.,3.5),(0.,3.5)],[(6.,0.),(10.,0.),(10.,3.5),(6.,3.5)]]
    assert not corridor_covers([[1.,1.],[9.,1.]],disjoint)
    # Wrong initial heading can be steer-feasible but outside the selected pair.
    result=plan(AdjacentLanePlanner(network(('left',))),yaw=-.12)
    assert result['status']=='BLOCKED'
    assert 'DRIVABLE_CORRIDOR' in result['evaluated_candidate_reject_reason']


def test_general_world_curvature_matches_independent_dense_evaluation():
    curve=WorldTransition([0.,0.],[1.,.1],[.01,.02],[15.,3.5],[1.,0.],[0.,.01],15.)
    values=[curve.state(v) for v in np.linspace(0.,15.,10001)]
    dense=max(abs(v[1][0]*v[2][1]-v[1][1]*v[2][0])/math.hypot(*v[1])**3 for v in values)
    assert curve.maximum_curvature()==pytest.approx(dense,rel=1e-6)


def test_real_map_separation_and_successor_pairs_use_original_geometry():
    directory=Path('/workspace/local_data/kcity_mgeo')
    if not directory.exists():pytest.skip('local K-City map not available')
    planner=DijkstraPlanner(directory);before=copy.deepcopy(planner.links_by_id)
    n=LaneNetwork(planner)
    pair=n.pair('A219BS010415','right',['A219BS010415','A219BS010419','A219BS010632'])
    assert pair is not None
    assert pair.ids[:2]==[('A219BS010415','A219BS010416'),('A219BS010419','A219BS010420')]
    assert pair.at(5.)['separation']==pytest.approx(-3.47,abs=.02)
    assert planner.links_by_id==before


def test_existing_unsafe_bag_scenes_remain_blocked_in_new_mode():
    directory=Path('/workspace/local_data/kcity_mgeo')
    if not directory.exists():pytest.skip('local K-City map not available')
    n=LaneNetwork(DijkstraPlanner(directory))
    scenes=json.loads((Path(__file__).parent/'fixtures/s4_20kph_blocked_scenes.json').read_text())['cases']
    for scene in scenes:
        p=AdjacentLanePlanner(n)
        result=p.plan(scene['reference'],scene['obstacles'],scene['position'],current_heading=scene['yaw'],
                      obstacle_groups=scene['groups'],road_progress_m=0.,current_link_id='A219BS010419',
                      route_link_ids=['A219BS010419','A219BS010632'],speed_kmh=20.)
        assert result['status']=='BLOCKED'


def test_indexed_projection_matches_original_nearest_segment_geometry():
    from edgenode_planning.lane_geometry import PolylineIndex
    from edgenode_planning.lattice_planner import LatticePlanner
    points=[[i*.3,2.*math.sin(i*.05),0.] for i in range(200)]
    index=PolylineIndex(points)
    rng=np.random.default_rng(72)
    for point in rng.uniform([-3.,-5.],[63.,5.],size=(200,2)):
        expected=LatticePlanner._project_reference(points,index.lengths,point)[0]
        assert index.project(point)==pytest.approx(expected,abs=1e-9)


def test_yaml_activates_adjacent_mode_without_changing_control_geometry():
    import yaml
    parameters=yaml.safe_load((Path(__file__).parents[1]/'config/planning.yaml').read_text())['planning_node']['ros__parameters']
    assert parameters['obstacle_avoidance_mode']=='adjacent_lane'
    assert parameters['lane_change_min_duration_sec']==3.
    assert parameters['ego_half_width_m']+parameters['obstacle_safety_margin_m']==1.2


def test_curved_variable_separation_uses_map_center_suffix_not_constant_offset():
    n=network(('left',))
    n.links['L']['points']=[[i*.5,3.5+.001*(i*.5)**2,0.] for i in range(161)]
    p=AdjacentLanePlanner(n)
    result=plan(p)
    assert result['status']=='AVOIDING'
    assert result['points'][-1][1]>3.8
    pair=p._lane_pair
    endpoint=p._world_transition.end
    velocity=p._world_transition.end_first
    yaw=math.atan2(velocity[1],velocity[0])
    held=plan(p,endpoint[0],endpoint[1],yaw,current='L')
    assert held['lane_change_phase']=='ADJACENT_LANE_HOLD'
    assert held['entry_length']==0.
    # Every held vertex is on the actual selected piecewise MGeo centerline.
    for point in held['points']:
        from edgenode_planning.lane_geometry import sample
        projection=sample(pair.adjacent,pair.adjacent_index.project(point))
        assert math.dist(point[:2],projection[0][:2])<1e-8


def test_collision_bag_frozen_input_reaches_actual_adjacent_map_center(tmp_path):
    fixture=json.loads((Path(__file__).parent/'fixtures/s4_adjacent_lane_scene.json').read_text())
    (tmp_path/'node_set.json').write_text(json.dumps(fixture['nodes']))
    (tmp_path/'link_set.json').write_text(json.dumps(fixture['links']))
    inputs=fixture['inputs']
    planner=AdjacentLanePlanner(LaneNetwork(DijkstraPlanner(tmp_path)))
    r=planner.plan(inputs['reference'],inputs['obstacles'],inputs['position'],
                   current_heading=inputs['yaw'],road_progress_m=inputs['road_progress_m'],
                   current_link_id=inputs['current_link_id'],route_link_ids=inputs['route_link_ids'],speed_kmh=20.)
    assert r['status']=='AVOIDING'
    assert r['selected_side']=='right'
    assert r['selected_offset_m']< -3.4
    assert r['entry_length']==pytest.approx(20/3.6*3.)
    assert r['maximum_curvature']<.08
    assert r['minimum_clearance_m']>2.9
    assert r['drivable_safe']
    assert r['adjacent_link_id']=='A219BS010416'


def test_world_boundary_projection_and_completion_do_not_reset_on_pose_jitter():
    p=AdjacentLanePlanner(network(('left',)))
    plan(p)
    initial=p._world_transition
    endpoint=list(initial.end)
    for physical_x,road_progress in ((5.,5.),(4.95,5.),(5.1,5.1)):
        position,first,_=initial.state(physical_x)
        result=p.plan(road(road_progress),obstacle(),position,
                      current_heading=math.atan2(first[1],first[0]),road_progress_m=road_progress,
                      current_link_id='O',route_link_ids=['O'],speed_kmh=20.)
        assert result['status']=='AVOIDING'
        assert p._world_transition.end==pytest.approx(endpoint,abs=1e-6)
        assert p._maneuver_progress_m==pytest.approx(road_progress)


def test_world_station_fallback_preserves_continuation_without_external_progress():
    p=AdjacentLanePlanner(network(('left',)))
    first=p.plan(road(),obstacle(),(0.,0.),current_heading=0.,current_link_id='O',route_link_ids=['O'],speed_kmh=20.)
    initial=p._world_transition
    position,velocity,_=initial.state(5.)
    following=p.plan(road(5.),obstacle(),position,current_heading=math.atan2(velocity[1],velocity[0]),
                     current_link_id='L',route_link_ids=['O'],speed_kmh=20.)
    assert following['status']=='AVOIDING'
    assert following['entry_length']==pytest.approx(first['entry_length']-5.,abs=1e-6)
