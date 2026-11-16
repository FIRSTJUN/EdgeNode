"""Entry search stays stopped and adopts only current-pose/current-solid work."""
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from threading import Event
from unittest.mock import patch

from edgenode_planning.entry_search import EntrySearch
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from test_static_obstacle_planner import setup, plan


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
