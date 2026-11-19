"""Entry search stays stopped and adopts only current-pose/current-solid work."""
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from threading import Event
from unittest.mock import patch
from dataclasses import replace

from edgenode_planning.entry_search import EntrySearch
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from test_static_obstacle_planner import setup, plan


def test_continuous_budget_exhaustion_latches_instead_of_restarting_same_search():
    p,point,obstacle=setup()
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.)
    assert plan(p,point,[obstacle()],speed=10.)['state']=='DETECT'
    search=EntrySearch(ThreadPoolExecutor(max_workers=1),budget_s=1e-9)
    try:
        plan(p,point,[],speed=10.,now=.1,entry_search=search)
        search._future.result(timeout=10.)
        result=plan(p,point,[],speed=10.,now=.2,entry_search=search)
        assert result['blocked'] and result['reason']=='ENTRY_SEARCH_BUDGET_EXHAUSTED'
        assert p.state=='BLOCKED' and search._future is None
        assert plan(p,point,[],speed=0.,now=.3,entry_search=search)['blocked']
        assert search._future is None
    finally:search.close()


def test_rejected_attempts_share_one_entry_episode_deadline():
    p,point,obstacle=setup();p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.)
    plan(p,point,[obstacle()],speed=10.)
    search=EntrySearch(ThreadPoolExecutor(max_workers=1),budget_s=12.)
    try:
        with patch('edgenode_planning.entry_search.time.monotonic',return_value=100.) as clock:
            with patch.object(StaticObstaclePlanner,'_search_entry',lambda s,*a:s._block('NO_VALID_LANE')):
                plan(p,point,[],speed=10.,now=.1,entry_search=search)
                search._future.result(timeout=10.)
                assert plan(p,point,[],speed=10.,now=.2,entry_search=search)['reason']=='ENTRY_SEARCH_PENDING'
                clock.return_value=112.1
                result=plan(p,point,[],speed=10.,now=.3,entry_search=search)
        assert result['blocked'] and result['reason']=='ENTRY_SEARCH_BUDGET_EXHAUSTED'
        assert p._entry_context['search_budget']['classification']=='COMPUTATION_LIMIT_NOT_PHYSICAL_IMPOSSIBILITY'
        assert search._future is None
    finally:search.close()


def test_coasting_vehicle_stays_stopped_without_starting_obsolete_search():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    search=EntrySearch(ThreadPoolExecutor(max_workers=1))
    try:
        for speed in (1.,.21):
            result=plan(p,point,[],speed=speed,entry_search=search)
            assert result['blocked'] and not result['points']
            assert search._future is None and p.entry_commits==0
        plan(p,point,[],speed=.1,entry_search=search)
        assert search._future is not None
        search._future.result(timeout=12.)
    finally:search.close()


def test_search_keeps_callbacks_available_and_revalidates_a_new_solid():
    p,point,obstacle=setup();plan(p,point,[obstacle()])
    started=Event();release=Event();original=StaticObstaclePlanner._search_entry
    def slow(snapshot,*args):
        started.set();assert release.wait(4.)
        return original(snapshot,*args)
    search=EntrySearch(executor=ThreadPoolExecutor(max_workers=1))
    try:
        with patch.object(StaticObstaclePlanner,'_search_entry',slow):
            result=plan(p,point,[],now=.1,speed=0.,entry_search=search)
            assert started.wait(1.)
            assert result['reason']=='ENTRY_SEARCH_PENDING' and result['blocked'] and not result['points']
            assert p.entry_commits==0 and p.state=='DETECT'
            assert plan(p,point,[],now=.2,speed=0.,entry_search=search)['reason']=='ENTRY_SEARCH_PENDING'
            release.set();search._future.result(timeout=12.)
        result=plan(p,point,[obstacle(15.,20.,-1.,5.)],now=.3,speed=0.,entry_search=search)
        assert result['blocked'] and not result['points'] and p.entry_commits==0
    finally:release.set();search.close()


def test_stale_start_pose_discards_worker_result_without_latching_a_failure():
    p,point,obstacle=setup();plan(p,point,[obstacle()]);search=EntrySearch(ThreadPoolExecutor(max_workers=1))
    try:
        plan(p,point,[],now=.1,speed=0.,entry_search=search);search._future.result(timeout=12.)
        result=plan(p,point,[],s=1.,now=.2,speed=0.,entry_search=search)
        assert result['reason']=='ENTRY_SEARCH_PENDING' and p.state=='DETECT' and not p.selected
        assert p.entry_commits==0 and search._future is None
        plan(p,point,[],s=1.,now=.3,speed=0.,entry_search=search);search._future.result(timeout=12.)
        assert plan(p,point,[],s=1.,now=.4,speed=0.,entry_search=search)['state']=='AVOID_ENTRY'
        assert p.entry_commits==1
    finally:search.close()


def test_production_worker_search_is_serializable_and_commits_only_after_live_validation():
    p,point,obstacle=setup();plan(p,point,[obstacle()]);search=EntrySearch()
    try:
        assert isinstance(search._executor,ProcessPoolExecutor)
        assert plan(p,point,[],now=.1,speed=0.,entry_search=search)['reason']=='ENTRY_SEARCH_PENDING'
        result,snapshot,generated,elapsed=search._future.result(timeout=20.)
        assert result['state']=='AVOID_ENTRY' and generated>0 and elapsed>0
        assert p.entry_commits==0 and snapshot.entry_commits==1
        assert plan(p,point,[],now=.2,speed=0.,entry_search=search)['state']=='AVOID_ENTRY'
        assert p.entry_commits==1
    finally:search.close()


class DeferredSearch:
    def submit(self,*values):
        from concurrent.futures import Future
        self.values=values
        return Future()
    def shutdown(self,**kwargs):pass


def test_pending_approach_speed_cap_survives_small_pid_overspeed():
    p,point,obstacle=setup();p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.)
    plan(p,point,[obstacle()],speed=10.)
    search=EntrySearch(DeferredSearch(),budget_s=12.)
    try:
        plan(p,point,[],speed=10.,now=.1,entry_search=search)
        assert search.active
        result=plan(p,point,[],speed=10.3,now=.2,entry_search=search)
        assert result['reason']=='ENTRY_SEARCH_PENDING'
        assert result['speed_cap_kmh']<10. and p._entry_context['search_approach_cap_kmh']==result['speed_cap_kmh']
    finally:search.close()


def test_turn_alignment_wait_uses_existing_guide_and_consumes_total_budget():
    p,point,obstacle=setup();p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.)
    p.complete_offset_search=True
    plan(p,point,[obstacle()],speed=10.)
    search=EntrySearch(DeferredSearch(),budget_s=12.)
    try:
        with patch('edgenode_planning.entry_search.time.monotonic',return_value=100.) as clock:
            result=plan(p,point,[],speed=10.,yaw=.1,now=.1,entry_search=search)
            assert result['reason']=='ENTRY_ALIGNMENT_PENDING' and result['points'] and not result['blocked']
            assert not search.active
            clock.return_value=112.1
            result=plan(p,point,[],speed=10.,yaw=0.,now=.2,entry_search=search)
            assert result['reason']=='ENTRY_SEARCH_BUDGET_EXHAUSTED' and result['blocked']
            assert not search.active
    finally:search.close()
