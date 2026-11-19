from dataclasses import replace
from types import SimpleNamespace
import math
from edgenode_planning.lane_geometry import PolylineIndex,sample,arc
from edgenode_planning.lattice_path import center_suffix
from test_static_obstacle_planner import setup


def curved():
    p,_,obstacle=setup()
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.,tracking_limit_m=.15,steering_slew_limit_radps=.6)
    original=tuple((22.*math.sin(s/22.),22.*(1.-math.cos(s/22.)),0.) for s in (i*.25 for i in range(241)))
    pair=SimpleNamespace(original=original,original_index=PolylineIndex(original),mode='road_offset')
    return p,pair,obstacle


def test_native_approach_can_be_adopted_after_control_has_followed_a_bend():
    p,pair,obstacle=curved();p.complete_offset_search=True
    pose=(0.,0.,0.)
    prepared,prefix,turn=p._entry_preparation(pair,pose,10.,0.,0.)
    end=pair.original_index.project(prepared[:2])
    path=prefix+center_suffix(pair.original,end,30.)[1:]
    row=sample(pair.original,end)
    actual=(*row[0][:2],math.atan2(row[1][1],row[1][0]))
    assert p._path_safety(path,(),10.)==''
    assert p._rollout(path,actual,10.,turn,())==''
    assert p._path_safety(path,[obstacle(1.,4.,-1.,2.)],10.)=='BODY_COLLISION'
    legacy,_=p._preparation(pose,10.,0.,0.)
    assert math.dist(legacy[:2],prepared[:2])>p.config.tracking_limit_m


def test_legacy_entry_preparation_preserves_the_existing_delay_model():
    p,pair,_=curved()
    actual=p._entry_preparation(pair,(0.,0.,0.),10.,.01,0.)
    expected=p._preparation((0.,0.,0.),10.,.01,0.)
    assert actual[:2]==expected and actual[2]==.01


def test_offset_approach_keeps_the_straight_guide_after_preparation_has_finished_turning():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    p,_,obstacle=setup()
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.,
        tracking_limit_m=.15,steering_slew_limit_radps=.6)
    p.complete_offset_search=True
    road=VerifiedRoadCorridor([((-20.,-8.),(150.,-8.),(150.,8.),(-20.,8.))],
        source='synthetic surveyed straight road',link_ids=('O0','O1'),allowed_sides=('left',))
    pair=road.offset_pair(p.network,'O0','left',['O0','O1'],3.5,0,maximum_station_m=120.)
    p._global_path=pair.original;p._entry_onset_floor_xy=(20.,0.)
    p._remember((tuple(obstacle(35.,39.)),));p._hazards=p.remembered
    front=pair.original_index.project((35.,0.))
    candidates=p._generate_entry(pair,(0.,0.,0.),10.,.044,front,{})
    assert candidates and candidates[0].entry_path
    approach=[point for point in candidates[0].entry_path if 4.<point[0]<18.]
    assert approach
    # The measured turn belongs to the completed preparation. Reinjecting
    # it creates an unnecessary second bend before any lateral maneuver.
    assert max(abs(point[1]) for point in approach)<1e-5
