from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import pytest

from edgenode_planning.entry_search import EntrySearch
from edgenode_planning.lane_geometry import LanePair,PolylineIndex,sample
from test_continuous_avoidance import continuous


def scene(speed=10.):
    p,point,obstacle=continuous()
    p.config=replace(p.config,optimize_entry_onset=True,avoid_speed_kmh=speed)
    reference=[point(i*.5,0.) for i in range(281)]
    return p,reference,obstacle


def tick(p,reference,objects=(),x=0.,speed=10.,stamp=1,search=None):
    return p.plan(reference,objects,(x,0.,0.),speed_kmh=speed,
        current_link_id='O0',route_link_ids=['O0','O1'],global_reference=reference,
        observation_stamp_ns=stamp,entry_search=search)


@pytest.mark.parametrize('speed',[10.,15.])
def test_early_commit_contains_a_verified_straight_approach_before_dynamic_onset(speed):
    p,reference,obstacle=scene(speed)
    assert tick(p,reference,[obstacle(70.,74.)],speed=speed)['state']=='DETECT'
    result=tick(p,reference,speed=speed,stamp=2)
    assert result['state']=='AVOID_ENTRY' and p.entry_commits==1
    timing=result['entry_context']['onset_plan']
    assert not timing['calculation_deferred'] and timing['approach_length_m']>20.
    assert timing['transition_length_m']>=timing['minimum_transition_m']
    assert all(abs(q[1])<1e-7 for q in result['points'] if q[0]<timing['onset_station_m']-10.)
    assert not p._validate_active(result['points'],(10.,0.,0.),speed,0.,{})
    # The already computed path remains subject to fresh obstacle rejection.
    blocked=tick(p,reference,[obstacle(12.,15.)],x=10.,speed=speed,stamp=3)
    assert blocked['blocked'] and not blocked['points']


def test_maneuver_request_changes_with_speed_lane_separation_and_measured_error():
    p,reference,_=scene();pair=p.network.pair('O0','left',['O0','O1'])
    ten=p._entry_onset_request(pair,(0.,0.,0.),10.,0.,70.)
    fifteen=p._entry_onset_request(pair,(0.,0.,0.),15.,0.,70.)
    narrow=LanePair(pair.original,[(q[0],.7,q[2]) for q in pair.original],pair.ids,'left',pair.section_ends)
    small=p._entry_onset_request(narrow,(0.,0.,0.),10.,0.,70.)
    error=p._entry_onset_request(pair,(0.,.1,0.),10.,0.,70.)
    assert fifteen['minimum_transition_m']>ten['minimum_transition_m']>small['minimum_transition_m']
    assert error['reserve_m']>ten['reserve_m']


def test_candidates_are_computed_during_braking_and_cannot_be_adopted_at_high_speed():
    p,reference,obstacle=scene();search=EntrySearch(ThreadPoolExecutor(max_workers=1))
    try:
        assert tick(p,reference,[obstacle(70.,74.)],speed=40.)['state']=='DETECT'
        braking=tick(p,reference,speed=40.,stamp=2,search=search)
        assert braking['reason']=='DECELERATING_ENTRY_SEARCH' and search._future is not None
        assert braking['speed_cap_kmh']<=10. and p.entry_commits==0
        search._future.result(timeout=25.)
        waiting=tick(p,reference,speed=40.,stamp=3,search=search)
        assert waiting['state']=='DETECT' and p.entry_commits==0 and search._future is not None
        accepted=tick(p,reference,x=20.,speed=10.,stamp=4,search=search)
        assert accepted['state']=='AVOID_ENTRY' and p.entry_commits==1
        assert accepted['entry_context']['onset_plan']['commit_to_onset_m']>0.
    finally:search.close()
