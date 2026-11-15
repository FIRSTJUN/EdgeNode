"""Generalized search witnesses, explicit road evidence and unsafe negatives."""
from dataclasses import replace
import math

import pytest

from edgenode_planning.avoidance_geometry import polygon_distance, swept_body
from edgenode_planning.lattice_path import poses
from edgenode_planning.road_corridor import VerifiedRoadCorridor
from edgenode_planning.static_obstacle_planner import AvoidanceConfig, StaticObstaclePlanner
from avoidance_world import world


def rect(x0,x1,y0,y1):
    return ((x0,y0),(x1,y0),(x1,y1),(x0,y1))


def corridor(cells=None, **kwargs):
    return VerifiedRoadCorridor(cells or [rect(-15.,140.,-5.5,5.5)],
        source='independent synthetic ground truth, metres, same-direction road',
        link_ids=('O0','O1'),allowed_sides=('left','right'),**kwargs)


def test_whole_solid_coverage_detects_interior_hole_despite_covered_vertices_and_edges():
    area=corridor([rect(-5,5,-5,-1),rect(-5,5,1,5),rect(-5,-1,-1,1),rect(1,5,-1,1)])
    assert all(area.covers(rect(x-.01,x+.01,y-.01,y+.01)) for x,y in rect(-3,3,-3,3))
    assert not area.covers(rect(-3,3,-3,3))
    assert area.covers(rect(-4,-2,-4,4))


def test_cell_union_supports_shared_edges_and_overlaps_but_never_bridges_a_gap():
    assert corridor([rect(-5,0,-5,5),rect(0,5,-5,5)]).covers(rect(-3,3,-3,3))
    assert corridor([rect(-5,1,-5,5),rect(-1,5,-5,5)]).covers(rect(-3,3,-3,3))
    assert not corridor([rect(-5,-.01,-5,5),rect(.01,5,-5,5)]).covers(rect(-3,3,-3,3))


@pytest.mark.parametrize('source,links,sides', [('',('O0',),('left',)),('s',(),('left',)),('s',('O0',),()),('s',('O0',),('unknown',))])
def test_road_evidence_requires_provenance_direction_and_permission(source,links,sides):
    with pytest.raises(ValueError):
        VerifiedRoadCorridor([rect(-5,5,-5,5)],source=source,link_ids=links,allowed_sides=sides)


@pytest.mark.parametrize('cell', [((0.,0.),(2.,0.),(1.,1.),(2.,2.),(0.,2.)), ((0.,0.),(1.,0.),(2.,0.)), ((0.,0.),(1.,0.),(math.nan,1.))])
def test_unknown_or_nonconvex_road_cells_are_not_convexified_into_free_space(cell):
    with pytest.raises(ValueError):corridor([cell])


def tick(planner,point,solids,**kwargs):
    return planner.plan([point(i*.5,0.) for i in range(161)],solids,(*point(0.,0.)[:2],kwargs.pop('yaw',0.)),
        speed_kmh=10.,current_link_id='O0',route_link_ids=['O0','O1'],**kwargs)


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('radius',[math.inf,150.,300.])
@pytest.mark.parametrize('front',[24.,35.,52.])
def test_multiple_obstacle_positions_on_straights_and_curves_use_full_body_and_full_entry_rollout(side,radius,front):
    lanes,point,obstacle=world(side,radius=radius)
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    assert tick(p,point,[obstacle(front,front+4)])['state']=='DETECT'
    result=tick(p,point,[])
    assert result['state']=='AVOID_ENTRY', [(c.transition_length_m,c.rejection) for c in p.candidates]
    selected=p.selected
    assert not selected.rejection
    pp=poses(selected.points)
    assert min(polygon_distance(swept_body(p.body,a,b),p.remembered[0]) for a,b in zip(pp,pp[1:]))>.25
    duration=__import__('edgenode_planning.lane_geometry',fromlist=['arc']).arc(selected.kinematic_points)[-1]/(10/3.6)+p.config.rollout_seconds
    assert duration>2.
    assert p._rollout(selected.points,(*point(0,0)[:2],0.),10.,0.,p.remembered,duration_s=duration)==''


@pytest.mark.parametrize('blocked,selected',[('left','right'),('right','left')])
def test_both_sides_are_evaluated_and_a_solid_in_one_adjacent_lane_selects_the_other(blocked,selected):
    lanes,point,obstacle=world(two_sides=True)
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    low,high=(2.,5.) if blocked=='left' else (-5.,-2.)
    tick(p,point,[obstacle(),obstacle(5.,65.,low,high)])
    result=tick(p,point,[])
    assert result['state']=='AVOID_ENTRY' and result['selected_side']==selected
    assert {c.side for c in p.candidates}=={'left','right'}
    assert all(c.rejection for c in p.candidates if c.side==blocked)


def road_scene(available=True,narrow=False,forbidden=False):
    lanes,point,obstacle=world()
    for key in ('O0','O1'):lanes.links[key]['left_lane_change_dst_link_idx']=None
    if available:lanes.road_corridor=corridor([rect(-15,140,-1.8,1.8)] if narrow else None)
    if forbidden:
        for key in ('O0','O1'):lanes.links[key]['can_move_left_lane']=False;lanes.links[key]['can_move_right_lane']=False
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    tick(p,point,[obstacle()]);return p,point,tick(p,point,[])


def test_offset_only_works_with_explicit_area_and_crossing_scope():
    p,point,result=road_scene()
    assert result['state']=='AVOID_ENTRY' and p.pair.mode=='road_offset'
    assert result['entry_context']['road_offset_evidence']==p.network.road_corridor.source
    assert p._road_area is p.network.road_corridor
    assert p._return_permission(0.,50.)==''
    assert road_scene(available=False)[2]['reason']=='NO_VALID_ADJACENT_LANE'
    assert road_scene(forbidden=True)[2]['blocked']


def test_narrow_proven_road_still_rejects_offset_for_whole_body_containment():
    p,_,result=road_scene(narrow=True)
    assert result['blocked'] and not result['points']
    assert any(c.rejection=='ROAD_AREA_UNPROVEN' for c in p.candidates)
    assert all(c.rejection for c in p.candidates)


def test_no_obstacle_keeps_normal_identity_even_with_road_evidence_and_off_path_solids():
    lanes,point,obstacle=world();lanes.road_corridor=corridor()
    p=StaticObstaclePlanner(lanes);reference=[point(i,0.) for i in range(81)]
    result=p.plan(reference,[obstacle(10.,20.,8.,10.)],(0.,0.,0.))
    assert result['state']=='NORMAL' and result['points'] is reference and not result['active']


def test_braking_violation_cannot_be_rescued_by_road_area_or_extra_candidates():
    lanes,point,obstacle=world();lanes.road_corridor=corridor();p=StaticObstaclePlanner(lanes)
    result=p.plan([point(i,0.) for i in range(81)],[obstacle(8.,12.)],(0.,0.,0.),speed_kmh=60.)
    assert result['reason']=='BRAKING_DISTANCE' and result['blocked'] and p.entry_commits==0


def test_static_map_cache_never_caches_obstacles_and_invalidates_for_a_new_boundary_source():
    from unittest.mock import Mock
    lanes,point,obstacle=world();lanes.boundaries=Mock();lanes.boundaries.swept_reason.return_value=''
    p=StaticObstaclePlanner(lanes);p._road_area=corridor()
    path=[point(i*.25,0.) for i in range(81)]
    assert p._path_safety(path,(),3.)==''
    calls=lanes.boundaries.swept_reason.call_count
    assert p._path_safety(path,(tuple(obstacle(10.,12.)),),3.)=='BODY_COLLISION'
    assert lanes.boundaries.swept_reason.call_count==calls
    lanes.boundaries=Mock();lanes.boundaries.swept_reason.return_value='LANE_BOUNDARY_COLLISION:new'
    assert p._path_safety(path,(),3.)=='LANE_BOUNDARY_COLLISION:new'


def test_longitudinal_reserve_is_swept_as_obstacle_uncertainty():
    lanes,point,obstacle=world();p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.))
    tick(p,point,[obstacle()]);result=tick(p,point,[],entry_reserve_m=100.)
    assert result['blocked'] and p.entry_commits==0


def test_clearance_can_be_proven_during_transition_before_adjacent_center_acquisition():
    lanes,point,obstacle=world()
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    tick(p,point,[obstacle(18.,22.)]);result=tick(p,point,[])
    assert result['state']=='AVOID_ENTRY'
    front=min(p.pair.original_index.project(q) for q in p.remembered[0])
    # The old center-acquisition deadline would reject this swept-safe entry.
    assert p.selected.entry_end_s+p.body.wheelbase_m+p.body.front_overhang_m+.25>front


@pytest.mark.parametrize('side',['left','right'])
def test_entry_can_cross_a_connected_same_direction_lane_pair_join(side):
    lanes,point,obstacle=world(side,radius=300.)
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    reference=[point(32.+i*.5,0.) for i in range(161)]
    pose=(*point(32.,0.)[:2],32/300.)
    kwargs=dict(speed_kmh=10.,current_link_id='O0',route_link_ids=['O0','O1'])
    assert p.plan(reference,[obstacle(63.,67.)],pose,**kwargs)['state']=='DETECT'
    result=p.plan(reference,[],pose,**kwargs)
    assert result['state']=='AVOID_ENTRY'
    assert p.selected.entry_end_s>p.pair.section_ends[0]


def test_disconnected_pair_join_cannot_be_filled_in_by_extended_search():
    lanes,point,obstacle=world();lanes.links['A1']['from_node_idx']='UNCONNECTED'
    p=StaticObstaclePlanner(lanes,config=replace(AvoidanceConfig(),avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    reference=[point(32.+i*.5,0.) for i in range(161)];pose=(*point(32.,0.)[:2],0.)
    kwargs=dict(speed_kmh=10.,current_link_id='O0',route_link_ids=['O0','O1'])
    p.plan(reference,[obstacle(63.,67.)],pose,**kwargs)
    result=p.plan(reference,[],pose,**kwargs)
    assert result['blocked'] and p.entry_commits==0
    assert all(c.rejection=='INSUFFICIENT_ENTRY_SUPPORT' for c in p.candidates)


@pytest.mark.parametrize('side',['left','right'])
def test_road_offset_model_actually_passes_and_returns_inside_explicit_surface(side):
    from offline_static_avoidance import run
    def configure(lanes):
        for key in ('O0','O1'):lanes.links[key]['left_lane_change_dst_link_idx']=None
        lanes.road_corridor=VerifiedRoadCorridor([rect(-15.,140.,-5.5,5.5)],
            source='independent synthetic road with both lateral limits and no exclusions',
            link_ids=('O0','O1'),allowed_sides=(side,))
    result=run(10.,side,obstacle_front_m=41.,configure_network=configure,
        brake_to_default=True,periods=(.1,),duration=50.,reference_horizon_m=80.,
        config_overrides=dict(avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    assert result['success'],result['failure']
    assert result['entry_commits']==result['return_commits']==1
    assert result['min_body_clearance_m']>.25
    assert all(row['inside'] for row in result['rows'])
