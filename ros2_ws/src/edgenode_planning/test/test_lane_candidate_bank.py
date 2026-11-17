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
    finally:p.close()


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
