from concurrent.futures import Future,ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace
import pytest

from edgenode_planning.entry_search import EntrySearch
from edgenode_planning.return_search import ReturnSearch
from edgenode_planning.lane_continuation import continuation
from edgenode_planning.lane_geometry import LanePair
from test_static_obstacle_planner import setup,plan,enter,acquire


def continuous():
    p,point,obstacle=setup()
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6)
    return p,point,obstacle


def test_moving_entry_keeps_approach_then_commits_once_with_live_validation():
    p,point,obstacle=continuous();assert plan(p,point,[obstacle()],speed=10.)['state']=='DETECT'
    search=EntrySearch(ThreadPoolExecutor(max_workers=1))
    try:
        pending=plan(p,point,[],speed=10.,now=.1,entry_search=search)
        assert not pending['blocked'] and pending['points'] and p.entry_commits==0
        search._future.result(timeout=15.)
        committed=plan(p,point,[],s=1.,speed=10.,now=.2,entry_search=search)
        assert committed['state']=='AVOID_ENTRY' and p.entry_commits==1
        assert committed['points'][0][0]<1.  # Fixed prefix remains, rather than snapping to a future origin.
    finally:search.close()


def test_lane_cruise_is_not_immediate_return_and_background_return_keeps_motion():
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    cruise=plan(p,point,[],s=45.,offset=3.5,speed=10.,now=5.)
    assert cruise['state']=='AVOID_HOLD' and cruise['reason']=='AVOID_LANE_CRUISE'
    assert cruise['points'] and not cruise['blocked'] and p.return_commits==0
    ready=plan(p,point,[],s=90.,offset=3.5,speed=10.,now=6.)
    assert ready['state']=='RETURN' and ready['points']
    search=ReturnSearch(ThreadPoolExecutor(max_workers=1))
    try:
        waiting=plan(p,point,[],s=90.,offset=3.5,speed=10.,now=6.1,return_search=search)
        assert waiting['points'] and not waiting['blocked'] and p.return_commits==0
        search._future.result(timeout=15.)
        result=plan(p,point,[],s=91.,offset=3.5,speed=10.,now=6.2,return_search=search)
        assert not result['blocked'] and p.return_commits==1
    finally:search.close()


def test_natural_join_requires_real_node_and_geometry_continuity():
    original=[(0.,0.,0.),(10.,0.,0.)];adjacent=[(0.,3.,0.),(10.,3.,0.)]
    pair=LanePair(original,adjacent,[('O','A')],'left',[10.])
    links={
        'O':dict(from_node_idx='o0',to_node_idx='o1',points=original),
        'A':dict(from_node_idx='a0',to_node_idx='a1',points=adjacent),
        'M':dict(from_node_idx='a1',to_node_idx='j',points=[(10.,3.,0.),(20.,0.,0.)]),
        'O1':dict(from_node_idx='o1',to_node_idx='j',points=[(10.,0.,0.),(20.,0.,0.)]),
        'O2':dict(from_node_idx='j',to_node_idx='goal',points=[(20.,0.,0.),(40.,0.,0.)]),
    }
    network=SimpleNamespace(links=links,valid=set(links))
    result=continuation(network,pair,['O','O1','O2'])
    assert result.merge_link_id=='O2' and result.link_ids==('A','M','O2')
    assert result.points[-1]==(40.,0.,0.)
    links['M']['to_node_idx']='unrelated'
    assert continuation(network,pair,['O','O1','O2']).merge_station_m is None


def test_new_obstacle_requests_return_before_control_window_reaches_it():
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    result=plan(p,point,[obstacle(82.,86.,3.,4.)],s=45.,offset=3.5,speed=10.,now=5.)
    assert not result['blocked'] and result['points']
    assert result['state']=='RETURN' and result['reason']=='NEW_LANE_OBSTACLE_RETURN_SEARCH'
    assert result['entry_context']['lane_obstacle_gap_m']>p._control_horizon_m
    assert p.return_commits==0


def test_stopped_recovery_validation_does_not_commit_filter_overshoot_as_motion():
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    plan(p,point,[],s=45.,offset=3.5,speed=10.,now=5.)
    previous=p._last_progress
    result=plan(p,point,[],s=46.4,offset=3.5,speed=0.,now=5.1,motion_authorized=False)
    assert not result['blocked'] and p._last_progress==previous
    result=plan(p,point,[],s=46.,offset=3.5,speed=0.,now=5.2,motion_authorized=False)
    assert not result['blocked'] and p._last_progress==previous
    result=plan(p,point,[],s=46.,offset=3.5,speed=1.,now=5.3,motion_authorized=True)
    assert not result['blocked'] and p._last_progress>previous


def test_slow_entry_search_cannot_drive_beyond_its_captured_origin():
    class DeferredExecutor:
        def submit(self,*args):return Future()
        def shutdown(self,**kwargs):pass
    p,point,obstacle=continuous()
    assert plan(p,point,[obstacle()],speed=10.)['state']=='DETECT'
    search=EntrySearch(DeferredExecutor())
    try:
        parked=plan(p,point,[],speed=0.,now=.05,entry_search=search,motion_authorized=False)
        assert parked['points'] and search._future is None
        first=plan(p,point,[],speed=10.,now=.1,entry_search=search)
        closer=plan(p,point,[],s=4.,speed=3.,now=.2,entry_search=search)
        at_origin=plan(p,point,[],s=5.6,speed=0.,now=.3,entry_search=search)
        assert first['points'] and closer['points'] and at_origin['points']
        assert 0.<closer['speed_cap_kmh']<first['speed_cap_kmh']<=10.
        assert at_origin['speed_cap_kmh']==0. and p.entry_commits==0
        # A short approach budget never makes a fresh close obstacle safe.
        blocked=plan(p,point,[obstacle(6.,9.)],s=5.6,speed=3.,now=.4,entry_search=search)
        assert blocked['blocked'] and not blocked['points']
    finally:search.close()


def test_offset_future_obstacle_replans_only_on_a_live_validated_prefix():
    from unittest.mock import Mock
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.)
    p.selected=replace(p.selected,mode='road_offset')
    search=Mock(side_effect=lambda planner,*args:planner._result(planner._entry_pending_path,speed_cap=5.))
    result=plan(p,point,[obstacle(20.,23.,3.,4.)],speed=10.,now=.1,entry_search=search)
    assert not result['blocked'] and result['points'] and search.call_count==1
    assert result['entry_context']['active_replan_reason']=='BODY_COLLISION'
    assert not p._validate_active(result['points'],(0.,0.,0.),10.,0.,{})
    # A hazard in the current body/rollout cannot use the future replan path.
    blocked=plan(p,point,[obstacle(1.,4.,-.5,.5)],speed=10.,now=.2,entry_search=search)
    assert blocked['blocked'] and search.call_count==1


@pytest.mark.parametrize('divider_start,expected',[(45.,'BLOCKED'),(100.,'AVOID_ENTRY')])
def test_offset_entry_requires_a_return_opening_before_a_solid_divider(divider_start,expected):
    from edgenode_planning.avoidance_geometry import intersects
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    from test_strategy_dispatch import scene
    p,reference=scene('1')
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6)
    p.network.road_corridor=VerifiedRoadCorridor(
        [((-15.,-8.),(150.,-8.),(150.,8.),(-15.,8.))],
        source='synthetic surveyed road',link_ids=('O0','O1'),allowed_sides=('left',))
    divider=((divider_start,1.7),(150.,1.7))
    class Boundaries:
        def permission_reason(self,*args):return 'SHARED_BOUNDARY_UNAVAILABLE'
        def swept_reason(self,polygon):
            return 'LANE_BOUNDARY_COLLISION:divider' if intersects(polygon,divider) else ''
    p.network.boundaries=Boundaries()
    result=p.plan(reference,[],(0.,0.,0.),speed_kmh=10.,current_link_id='O0',route_link_ids=['O0','O1'])
    assert result['state']==expected
    if expected=='BLOCKED':
        assert result['reason']=='OFFSET_RETURN_UNPROVEN' and p.entry_commits==0
        assert not result['points']
    else:
        assert result['entry_context']['offset_return_proof']['source']=='LIVE_LIDAR_AND_VERIFIED_MAP'
        assert p.entry_commits==1


def test_narrow_offset_between_an_obstacle_and_divider_is_searched_before_a_distant_lane():
    from unittest.mock import patch
    from edgenode_planning.lattice_path import LatticeCandidate
    from test_strategy_dispatch import scene
    p,reference=scene('1');p.config=replace(p.config,continuous_lane_follow=True)
    tried=[]
    def generate(pair,*args,**kwargs):
        value=pair.adjacent[0][1]-pair.original[0][1];tried.append(value)
        reason=('BODY_COLLISION' if value<.78 else 'LANE_BOUNDARY_COLLISION:divider' if value>.88 and value<2. else '')
        path=tuple((float(i),value,0.) for i in range(61))
        return [LatticeCandidate('left',path,16.,0.,0.,12.,value,{},reason,
            entry_path=path,kinematic_points=path,mode='road_offset')]
    with patch.object(p,'_road_offsets',side_effect=lambda road,ref,side:(.3,1.1,3.) if side=='left' else ()), \
         patch.object(p,'_generate_entry',side_effect=generate), \
         patch.object(p,'_offset_return_proof',return_value={'source':'synthetic test'}):
        result=p.plan(reference,[],(0.,0.,0.),current_link_id='O0',route_link_ids=['O0','O1'])
    assert result['state']=='AVOID_ENTRY'
    assert .78<=p.pair.adjacent[0][1]<=.88 and max(tried)<2.


def test_precomputed_offset_return_is_adopted_with_live_checks_before_hold_tail_rebuild():
    from unittest.mock import patch
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    p.selected=replace(p.selected,mode='road_offset')
    search=ReturnSearch(ThreadPoolExecutor(max_workers=1))
    try:
        search.prefetch(p,(45.,3.5,0.),10.,0.)
        search._future.result(timeout=15.)
        assert p.return_commits==0 and p.state=='AVOID_HOLD' and not p._return_path
        with patch.object(p,'_hold',side_effect=AssertionError('unneeded distant hold tail')):
            result=plan(p,point,[],s=45.,offset=3.5,speed=10.,now=5.,return_search=search)
        assert result['state']=='RETURN' and not result['blocked'] and p.return_commits==1
    finally:search.close()


def test_precomputed_offset_return_cannot_ignore_a_new_observed_body_collision():
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    p.selected=replace(p.selected,mode='road_offset')
    search=ReturnSearch(ThreadPoolExecutor(max_workers=1))
    try:
        search.prefetch(p,(45.,3.5,0.),10.,0.);search._future.result(timeout=15.)
        p._remember(tuple(tuple(map(tuple,o)) for o in [obstacle(45.,49.,3.,4.)]))
        reason=search(p,(45.,3.5,0.),10.,0.,{})
        assert reason=='RETURN_SEARCH_LIVE_RETRY:BODY_COLLISION'
        assert p.return_commits==0 and not p._return_path
    finally:search.close()


def test_finite_hold_prefix_keeps_full_controller_support_and_rejects_a_shorter_one():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    from edgenode_planning.lane_geometry import arc
    p,point,obstacle=continuous();enter(p,point,obstacle,speed=10.);acquire(p,point,3.5)
    p._road_area=VerifiedRoadCorridor([((-20.,-8.),(65.,-8.),(65.,8.),(-20.,8.))],
        source='finite surveyed road',link_ids=('O0','O1','A0','A1'),allowed_sides=('left',))
    result=plan(p,point,[],s=45.,offset=3.5,speed=10.,now=5.)
    assert not result['blocked'] and p._rollout_distance(10.)<arc(result['points'])[-1]<20.
    assert not p._rollout(result['points'],(45.,3.5,0.),10.,0.,p.remembered)
    blocked=plan(p,point,[],s=55.,offset=3.5,speed=10.,now=6.)
    assert blocked['blocked'] and not blocked['points']
