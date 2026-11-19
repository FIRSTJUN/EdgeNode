from concurrent.futures import Future,ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace
import pickle
import pytest

from edgenode_planning.lane_geometry import LaneNetwork
from edgenode_planning.lane_candidate_bank import build_lane_bank,LaneCandidateBank
from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
from edgenode_planning.static_obstacle_planner import AvoidanceConfig
from edgenode_planning.entry_search import EntrySearch


def lane_world(count):
    links={}
    for lane in range(count):
        for part,(start,end) in enumerate(((-10.,40.),(40.,140.))):
            key=f'L{lane}_{part}'
            links[key]=dict(idx=key,from_node_idx=f'n{lane}_{part}',to_node_idx=f'n{lane}_{part+1}',
                points=[(float(x),lane*3.5,0.) for x in range(round(start),round(end)+1)],
                link_type='6',max_speed=30.,can_move_left_lane=True,can_move_right_lane=True,
                left_lane_change_dst_link_idx=f'L{lane+1}_{part}' if lane+1<count else None,
                right_lane_change_dst_link_idx=f'L{lane-1}_{part}' if lane else None)
    return LaneNetwork(SimpleNamespace(links_by_id=links,graph={'nodes':[dict(link_id=k) for k in links]}))


@pytest.mark.parametrize('count',[1,2,3,6])
def test_candidate_count_follows_actual_reachable_lanes_instead_of_a_fixed_five(count):
    snapshot=build_lane_bank(lane_world(count),'L0_0',('L0_0','L0_1'))
    assert len(snapshot.options)==count
    assert [o.lane_id for o in snapshot.options]==[f'L{i}_0' for i in range(count)]
    assert snapshot.options[0].pair is None
    for option in snapshot.options[1:]:
        assert option.points[0][1]==pytest.approx(option.hops*3.5)
        assert not option.exit_reason


@pytest.mark.parametrize('damage',['denied','opposite','nonreciprocal'])
def test_bank_does_not_count_lanes_beyond_a_denied_or_invalid_crossing(damage):
    network=lane_world(4)
    if damage=='denied':network.links['L1_0']['can_move_left_lane']=False
    elif damage=='opposite':network.links['L2_0']['points'].reverse()
    else:network.links['L2_0']['right_lane_change_dst_link_idx']=None
    snapshot=build_lane_bank(network,'L0_0',('L0_0','L0_1'))
    assert [o.lane_id for o in snapshot.options]==['L0_0','L1_0']


def test_ambiguous_intersection_prepares_only_its_original_reference_for_offset_fallback():
    network=lane_world(3);network.links['L0_0']['link_type']='1'
    snapshot=build_lane_bank(network,'L0_0',('L0_0','L0_1'))
    assert len(snapshot.options)==1 and snapshot.reason=='AMBIGUOUS_LANES_USE_OFFSET'


class ReadyBank:
    def __init__(self,snapshot):self.snapshot=snapshot
    def refresh(self,*args):return self.snapshot
    def close(self):pass


def planner():
    network=lane_world(3)
    p=LaneCandidatePlanner(network,config=replace(AvoidanceConfig(),continuous_lane_follow=True,
        integrated_strategy=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    p._lane_bank.close();p._lane_bank=ReadyBank(build_lane_bank(network,'L0_0',('L0_0','L0_1')))
    return p


def obstacle(front=35.,rear=39.,low=-.5,high=.5):
    middle=(front+rear)/2
    return [(front,low),(middle,low),(rear,low),(rear,(low+high)/2),
            (rear,high),(middle,high),(front,high),(front,(low+high)/2)]


def tick(p,objects=(),x=0.,stamp=1,search=None):
    reference=[(i*.5,0.,0.) for i in range(281)]
    return p.plan(reference,objects,(x,0.,0.),speed_kmh=10.,current_link_id='L0_0',
        route_link_ids=['L0_0','L0_1'],observation_stamp_ns=stamp,global_reference=reference,entry_search=search)


def test_selected_lane_and_output_survive_a_pending_candidate_refresh():
    p=planner()
    try:
        assert tick(p,[obstacle()])['state']=='DETECT'
        selected=tick(p,stamp=2)
        assert selected['state']=='AVOID_ENTRY' and selected['lane_candidate_count']==3
        assert selected['committed_lane_token']=='L1_0'
        fixed=p.selected;p._lane_bank.snapshot=None
        for i in range(3,6):
            result=tick(p,[obstacle()],x=(i-2)*.1,stamp=i)
            assert result['points'] and not result['blocked'] and p.selected is fixed and p.entry_commits==1
        blocked=tick(p,[obstacle(1.,4.)],x=.3,stamp=6)
        assert blocked['blocked'] and not blocked['points']
    finally:p.close()


def test_blocked_near_lane_selects_a_further_reachable_lane_without_invented_edges():
    p=planner()
    try:
        assert tick(p,[obstacle(),obstacle(70.,74.,3.,4.)])['state']=='DETECT'
        result=tick(p,stamp=2)
        assert result['state']=='AVOID_ENTRY' and result['committed_lane_token']=='L2_0'
        assert p.pair.bank_crossings[0]==('L0_0','L1_0','L2_0')
        assert result['entry_context']['candidate_checks'][0]['reason']=='NATIVE_LANE_BLOCKED'
    finally:p.close()


def test_prepared_snapshot_is_serializable_and_latest_obstacles_still_reject_a_worker_result():
    p=planner();search=EntrySearch()
    try:
        assert tick(p)['state']=='NORMAL'
        restored=pickle.loads(pickle.dumps(p))
        assert restored._lane_bank is None and len(restored._lane_snapshot.options)==3
        assert tick(p,[obstacle()])['state']=='DETECT'
        tick(p,stamp=2,search=search);search._future.result(timeout=25.)
        result=tick(p,[obstacle(1.,4.)],stamp=3,search=search)
        assert result['blocked'] and p.entry_commits==0 and not result['points']
    finally:search.close();p.close()


def test_async_map_refresh_keeps_completed_snapshots_and_does_not_block_the_caller():
    class Deferred:
        def submit(self,*args):self.result=Future();return self.result
        def shutdown(self,**kwargs):pass
    network=lane_world(2);executor=Deferred();bank=LaneCandidateBank(network,executor)
    try:
        assert bank.refresh('L0_0',('L0_0','L0_1')) is None
        executor.result.set_result(build_lane_bank(network,'L0_0',('L0_0','L0_1')))
        prepared=bank.refresh('L0_0',('L0_0','L0_1'))
        assert len(prepared.options)==2
        assert bank.refresh('L0_1',('L0_1',)) is None
        assert bank.refresh('L0_0',('L0_0','L0_1')) is prepared
    finally:bank.close()


def test_no_natural_join_and_denied_return_prevents_entering_a_trapping_lane():
    p=planner()
    try:
        p.network.links['L2_0']['can_move_right_lane']=False
        p._lane_bank.snapshot=build_lane_bank(p.network,'L0_0',('L0_0','L0_1'))
        assert p._lane_bank.snapshot.options[2].exit_reason=='LANE_CHANGE_FORBIDDEN'
        tick(p,[obstacle(),obstacle(70.,74.,3.,4.)])
        result=tick(p,stamp=2)
        assert result['blocked'] and p.entry_commits==0 and not result['points']
    finally:p.close()


def test_verified_intersection_uses_the_existing_offset_solver_and_return_proof():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    p=planner()
    try:
        p.network.links['L0_0']['link_type']='1'
        p.network.road_corridor=VerifiedRoadCorridor([((-20.,-8.),(150.,-8.),(150.,8.),(-20.,8.))],
            source='synthetic surveyed intersection',link_ids=('L0_0','L0_1'),allowed_sides=('left',))
        p._lane_bank.snapshot=build_lane_bank(p.network,'L0_0',('L0_0','L0_1'))
        tick(p,[obstacle()]);result=tick(p,stamp=2)
        assert result['selected_strategy']=='OFFSET_STATIC' and result['selected_mode']=='road_offset'
        assert result['entry_context']['offset_return_proof']
        assert p.selected.complete_offset and p.selected.return_path
        assert result['entry_context']['offset_return_proof']['whole_maneuver_pp_proven']
    finally:p.close()


def test_complete_offset_prefers_rear_clear_return_when_the_opening_is_available():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    p=planner()
    try:
        p.network.links['L0_0']['link_type']='1'
        p.network.road_corridor=VerifiedRoadCorridor([((-20.,-8.),(150.,-8.),(150.,8.),(-20.,8.))],
            source='synthetic surveyed intersection',link_ids=('L0_0','L0_1'),allowed_sides=('left',))
        p._lane_bank.snapshot=build_lane_bank(p.network,'L0_0',('L0_0','L0_1'))
        tick(p,[obstacle()]);result=tick(p,stamp=2)
        assert not result['blocked'] and p.selected.complete_offset
        rear=max(p.pair.original_index.project(point) for solid in p._hazards for point in solid)
        assert p.selected.return_start_s>=rear+p.body.rear_overhang_m+p.config.return_margin_m
        assert result['entry_context']['offset_return_proof']['whole_maneuver_pp_proven']
    finally:p.close()


@pytest.mark.parametrize('count',[1,2])
def test_exhausted_native_lanes_try_verified_offset_and_prove_return(count):
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    network=lane_world(count)
    p=LaneCandidatePlanner(network,config=replace(AvoidanceConfig(),continuous_lane_follow=True,
        integrated_strategy=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    p._lane_bank.close();p._lane_bank=ReadyBank(build_lane_bank(network,'L0_0',('L0_0','L0_1')))
    network.road_corridor=VerifiedRoadCorridor([((-20.,-5.),(150.,-5.),(150.,5.),(-20.,5.))],
        source='surveyed single road',link_ids=('L0_0','L0_1'),allowed_sides=('right',),exact_union_backend=True)
    try:
        solids=[obstacle()]+([obstacle(70.,74.,3.,4.)] if count>1 else [])
        tick(p,solids);result=tick(p,stamp=2)
        assert result['state']=='AVOID_ENTRY' and result['selected_strategy']=='OFFSET_STATIC'
        assert p.selected.complete_offset
        assert result['selected_side']=='right' and result['entry_context']['offset_return_proof']
        assert result['entry_context']['lane_search_exhausted']
        assert 'native_lane_search' in result['entry_context']
        fixed=p.selected
        result=tick(p,solids,x=.1,stamp=3)
        assert result['points'] and p.selected is fixed and p.entry_commits==1
        result=tick(p,[obstacle(1.,4.)],x=.2,stamp=4)
        assert result['blocked'] and not result['points']
    finally:p.close()


@pytest.mark.parametrize('speed',[10.,15.])
def test_complete_offset_causal_pp_loop_returns_without_a_second_search(monkeypatch,speed):
    import offline_static_avoidance as model
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    class Prepared(LaneCandidatePlanner):
        def __init__(self,network,**kwargs):
            super().__init__(network,**kwargs)
            self._lane_bank.close()
            self._lane_bank=ReadyBank(build_lane_bank(network,'O0',('O0','O1')))
    def setup(network):
        for link in network.links.values():link['link_type']='1'
        network.road_corridor=VerifiedRoadCorridor([((-20.,-6.),(150.,-6.),(150.,6.),(-20.,6.))],
            source='independent synthetic surveyed road',link_ids=('O0','O1'),
            allowed_sides=('left','right'),exact_union_backend=True)
    monkeypatch.setattr(model,'StaticObstaclePlanner',Prepared)
    result=model.run(speed,duration=45.,periods=(.1,),configure_network=setup,
        reference_horizon_m=80.,config_overrides=dict(continuous_lane_follow=True,
            integrated_strategy=True,steering_slew_limit_radps=.6,lookahead_max_m=12.,
            lookahead_preview_time_sec=.8,lookahead_heading_budget_rad=1.6))
    assert result['success'],result['failure']
    assert result['entry_commits']==result['return_commits']==1
    assert result['entry_path_objects']==result['return_path_objects']==1
    assert result['min_body_clearance_m']>.25 and result['max_tracking_m']<=.15


def test_offset_tracking_envelope_cannot_touch_an_internal_solid_boundary():
    from edgenode_planning.avoidance_geometry import intersects
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    p=planner()
    try:
        p._road_area=VerifiedRoadCorridor([((-20.,-8.),(150.,-8.),(150.,8.),(-20.,8.))],
            source='surveyed road',link_ids=('L0_0',),allowed_sides=('left',),exact_union_backend=True)
        line=((-10.,1.1),(150.,1.1))
        class Boundary:
            def swept_reason(self,polygon):
                return 'LANE_BOUNDARY_COLLISION:internal' if intersects(polygon,line) else ''
        p.network.boundaries=Boundary()
        p._planning_tracking_reserve=p.config.tracking_limit_m
        assert p._path_safety(((0.,0.,0.),(5.,0.,0.),(10.,0.,0.)),(),10.).startswith('LANE_BOUNDARY_COLLISION:')
    finally:p.close()


def test_native_fallback_never_creates_missing_road_evidence():
    p=planner()
    try:
        tick(p,[obstacle(),obstacle(70.,74.,3.,7.5)])
        result=tick(p,stamp=2)
        assert result['blocked'] and p.selected is None
        assert result['reason']=='NO_VERIFIED_MANEUVER_SPACE'
    finally:p.close()




def test_surveyed_hold_uses_continuation_stations_beyond_the_original_lane_pair():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    from edgenode_planning.lane_geometry import PolylineIndex
    p=planner()
    try:
        tick(p,[obstacle()]);tick(p,stamp=2)
        p.network.morai_unmarked_junction_experiment=True
        p._road_area=VerifiedRoadCorridor([((-20.,-3.),(270.,-3.),(270.,8.),(-20.,8.))],
            source='surveyed continuation',link_ids=tuple(p.network.links),
            allowed_sides=('left','right'),exact_union_backend=True)
        guide=tuple((float(x),3.5,0.) for x in range(-10,251))
        p._continuation=SimpleNamespace(points=guide,reason='',link_ids=('L1_0','L1_1'),merge_station_m=None)
        p._continuation_index=PolylineIndex(guide)
        p.state='AVOID_HOLD'
        result=p._hold((100.,3.5,0.),10.,0.,{})
        assert result['points'] and not result['blocked']
        assert result['points'][-1][0]>p.pair.adjacent[-1][0]
    finally:p.close()


def test_native_reference_handling_cannot_hide_a_predicted_body_leaving_surveyed_road():
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    p=planner()
    try:
        tick(p,[obstacle()]);tick(p,stamp=2)
        p.network.morai_unmarked_junction_experiment=True
        p._road_area=VerifiedRoadCorridor([((-20.,-3.),(106.,-3.),(106.,8.),(-20.,8.))],
            source='surveyed road ending ahead of the moving body',link_ids=tuple(p.network.links),
            allowed_sides=('left','right'),exact_union_backend=True)
        p.state='AVOID_HOLD';p._hold_path=tuple((float(x),3.5,0.) for x in range(95,141))
        # Current body is inside; the production PP forecast moves its front
        # through the exact road cap. Reference handling must still stop it.
        result=p._validate_active(p._hold_path,(100.,3.5,0.),10.,0.,{})
        assert result=='ROAD_AREA_UNPROVEN'
        assert p._road_area is not None
    finally:p.close()


@pytest.mark.parametrize('road_evidence',['native_only','surveyed','return_missing'])
def test_near_lane_end_proves_a_return_beyond_the_short_entry_controller_tail(road_evidence):
    from edgenode_planning.road_corridor import VerifiedRoadCorridor
    network=lane_world(2)
    for key,link in network.links.items():
        if key.endswith('_1'):link['points']=[(float(x),link['points'][0][1],0.) for x in range(40,95)]
    if road_evidence!='native_only':
        end=70. if road_evidence=='return_missing' else 150.
        network.road_corridor=VerifiedRoadCorridor([((-20.,-3.),(end,-3.),(end,6.5),(-20.,6.5))],
            source='surveyed road with exact end cap',link_ids=('L0_0','L0_1'),
            allowed_sides=('left','right'),exact_union_backend=True)
    p=LaneCandidatePlanner(network,config=replace(AvoidanceConfig(),continuous_lane_follow=True,
        integrated_strategy=True,avoid_speed_kmh=10.,steering_slew_limit_radps=.6))
    p._lane_bank.close();p._lane_bank=ReadyBank(build_lane_bank(network,'L0_0',('L0_0','L0_1')))
    reference=[(i*.5,0.,0.) for i in range(189)]
    def step(stamp,objects=()):
        return p.plan(reference,objects,(30.,0.,0.),speed_kmh=10.,current_link_id='L0_0',
            route_link_ids=['L0_0','L0_1'],observation_stamp_ns=stamp,global_reference=reference)
    try:
        step(1,[obstacle(60.,64.)]);result=step(2)
        if road_evidence=='return_missing':
            assert result['blocked'] and not p.entry_commits
            assert not result['points']
            return
        assert result['state']=='AVOID_ENTRY' and p.selected.mode=='lane'
        assert p.selected.complete_return and not p.selected.complete_offset
        assert p.selected.return_start_s>p.selected.entry_end_s+p._rollout_distance(10.)
        assert result['entry_context']['native_return_proof']['whole_maneuver_pp_proven']
    finally:p.close()


def test_previous_slow_solver_cannot_anchor_the_next_entry_past_the_obstacle():
    from edgenode_planning.avoidance_geometry import polygon_distance,swept_body
    from edgenode_planning.lattice_path import poses
    p=planner();search=EntrySearch(ThreadPoolExecutor(max_workers=1))
    try:
        search.elapsed_ms=25000.
        tick(p,[obstacle()]);result=tick(p,stamp=2,search=search)
        assert result['points'] and not result['blocked']
        assert search._pose[0]+p.body.wheelbase_m+p.body.front_overhang_m<35.
        assert search._prefix
        route=poses(search._prefix)
        assert min(polygon_distance(swept_body(p.body,a,b),p.remembered[0])
                   for a,b in zip(route,route[1:]))>.25
        search._future.result(timeout=20.)
        result=tick(p,stamp=3,search=search)
        if p.entry_commits:
            assert not p._path_safety(p.selected.entry_path,p.remembered,10.)
            assert result['entry_context']['entry_search_anchor']['requested_advance_m']>35.
    finally:search.close();p.close()


def test_background_return_keeps_its_origin_ahead_during_a_two_second_worker_delay():
    from edgenode_planning.return_search import ReturnSearch
    class Deferred:
        def submit(self,fn,*args):
            self.future=Future();self.value=fn(*args);return self.future
        def shutdown(self,**kwargs):pass
    p=planner();executor=Deferred();search=ReturnSearch(executor)
    reference=[(i*.5,0.,0.) for i in range(281)]
    def step(x,stamp):
        return p.plan(reference,[],(x,3.5,0.),speed_kmh=10.,current_link_id='L0_1',
            route_link_ids=['L0_0','L0_1'],global_reference=reference,
            observation_stamp_ns=stamp,return_search=search)
    try:
        tick(p,[obstacle()]);tick(p,stamp=2)
        end=p.selected.entry_end_s-10.-.5
        for stamp in range(3,6):step(end,stamp)
        assert p.state=='AVOID_HOLD'
        step(90.,6);assert p.state=='RETURN'
        result=step(90.,7);assert result['points'] and not p.return_commits
        # The real two-lane MORAI search took 1.8 seconds. A vehicle must not
        # pass the captured origin while a two-second result is in flight.
        travelled=10./3.6*2.
        assert search._pose[0]>90.+travelled
        executor.future.set_result(executor.value)
        result=step(90.+travelled,9)
        assert result['points'] and not result['blocked'] and p.return_commits==1
    finally:search.close();p.close()


def test_far_preview_cannot_interrupt_a_valid_lane_or_hide_a_fresh_near_collision():
    class DeferredExecutor:
        def submit(self,*args):self.future=Future();return self.future
        def shutdown(self,**kwargs):pass
    p=planner()
    try:
        tick(p,[obstacle()]);tick(p,stamp=2)
        reference=[(i*.5,0.,0.) for i in range(281)]
        entry_end=p.selected.entry_end_s-10.
        def step(x,objects=()):
            return p.plan(reference,objects,(x,3.5,0.),speed_kmh=10.,current_link_id='L0_0',
                route_link_ids=['L0_0','L0_1'],global_reference=reference,observation_stamp_ns=round(x*1e9))
        for _ in range(3):result=step(entry_end-.5)
        assert result['state']=='AVOID_HOLD'
        p._preview_executor.shutdown(wait=False);deferred=DeferredExecutor();p._preview_executor=deferred
        fixed=p.selected
        result=step(45.)
        assert result['state']=='AVOID_HOLD' and result['points'] and not result['blocked']
        assert deferred.future is not None and not deferred.future.done()
        result=step(45.5,[obstacle(82.,86.,3.,4.)])
        assert result['points'] and p.selected is fixed
        deferred.future.set_result((p._continuation_index.project((82.,3.5)),2.))
        result=step(46.)
        assert result['state']=='RETURN' and result['reason']=='NEW_LANE_OBSTACLE_RETURN_SEARCH'
        assert result['entry_context']['lane_obstacle_gap_m']>p._control_horizon_m
        # The executor's older snapshot can never authorize a new close solid.
        result=step(46.1,[obstacle(47.,50.,3.,4.)])
        assert result['blocked'] and not result['points']
    finally:p.close()
