"""STEP 4 work cannot starve callbacks or authorize an obsolete result."""
from threading import Event
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import os
from unittest.mock import patch
import pytest

from edgenode_planning.return_search import ReturnSearch
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from test_static_obstacle_planner import setup, enter, acquire, plan


def test_return_search_waits_for_measured_stop():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    assert plan(p,point,[],s=45.,offset=3.5,now=5.)['state']=='RETURN'
    search=ReturnSearch(ThreadPoolExecutor(max_workers=1))
    try:
        for speed in (10.,1.,.21):
            result=plan(p,point,[],s=45.,offset=3.5,speed=speed,now=5.1,return_search=search)
            assert result['blocked'] and not result['points']
            assert search._future is None and p.return_commits==0
        plan(p,point,[],s=45.,offset=3.5,speed=.1,now=5.2,return_search=search)
        assert search._future is not None
        search._future.result(timeout=12.)
    finally:search.close()


def test_stop_only_return_search_does_not_commit_filter_overshoot_as_motion():
    p,point,obstacle=setup();enter(p,point,obstacle);acquire(p,point,3.5)
    assert plan(p,point,[],s=45.,offset=3.5,now=5.)['state']=='RETURN'
    previous=p._last_progress
    def pending(*args):return 'RETURN_SEARCH_PENDING'
    # MORAI 15kmh_changed_fast: GPS stopped while EKF overshot 0.6m,
    # then corrected backward by >0.30m during the STOP-only search.
    for station in (45.8,46.4,46.0,45.8):
        r=plan(p,point,[],s=station,offset=3.5,speed=0.,now=5.1,return_search=pending)
        assert r['state']=='VALIDATING' and not r['points']
        assert p._last_progress==previous
    # Returning to an authorized motion path still rejects actual regression.
    r=plan(p,point,[],s=45.8,offset=3.5,speed=0.,now=5.2)
    assert not r['blocked'] and p._return_path
    r=plan(p,point,[],s=45.3,offset=3.5,speed=3.,now=5.3)
    assert r['reason']=='PROGRESS_REGRESSION' and not r['points']


def test_async_return_keeps_sequential_gates_and_rechecks_live_obstacles():
    p,point,obstacle=setup()
    enter(p,point,obstacle);acquire(p,point,3.5)
    assert plan(p,point,[],s=45.,offset=3.5,now=5.)['state']=='RETURN'
    original=StaticObstaclePlanner._generate_return
    started=Event();release=Event()
    def slow(snapshot,*args):
        started.set()
        assert release.wait(3.)
        return original(snapshot,*args)
    search=ReturnSearch(executor=ThreadPoolExecutor(max_workers=1))
    try:
        with patch.object(StaticObstaclePlanner,'_generate_return',slow):
            r=plan(p,point,[],s=45.,offset=3.5,now=5.1,speed=0.,return_search=search)
            assert started.wait(1.)
            assert r['stage']=='RETURN' and r['blocked'] and not r['points']
            assert p.return_commits==0 and not p._return_path
            # Another ROS planning cycle remains available during the search.
            r=plan(p,point,[],s=45.,offset=3.5,now=5.2,speed=0.,return_search=search)
            assert r['reason']=='RETURN_SEARCH_PENDING' and r['blocked']
            release.set();search._future.result(timeout=5.)
        # A newly observed solid on the return path must be revalidated even
        # though it was absent from the worker's snapshot.
        solid=obstacle(60.,64.,-1.,1.)
        r=plan(p,point,[solid],s=45.,offset=3.5,now=5.3,speed=0.,return_search=search)
        assert r['blocked'] and not r['points'] and p.reason in ('BODY_COLLISION','REFERENCE_CLEARANCE')
        p.reset();search.cancel()
        assert p.state=='NORMAL' and search._future is None
    finally:
        release.set();search.close()


def test_production_return_search_runs_outside_the_ros_process():
    planner, point, obstacle = setup()
    enter(planner, point, obstacle); acquire(planner, point, 3.5)
    assert plan(planner, point, [], s=45., offset=3.5, now=5.)['state'] == 'RETURN'
    search = ReturnSearch()
    try:
        assert isinstance(search._executor, ProcessPoolExecutor)
        plan(planner, point, [], s=45., offset=3.5, now=5.1, speed=0.,return_search=search)
        reason, snapshot, generated, elapsed, pid = search._future.result(timeout=15.)
        assert pid != os.getpid() and not reason and snapshot._return_path
        assert planner.return_commits == 0
        result = plan(planner, point, [], s=45., offset=3.5, now=5.2, speed=0.,return_search=search)
        assert not result['blocked'] and planner.return_commits == 1
    finally:
        search.close()


@pytest.mark.parametrize('shift,yaw', [(-.29,0.),(0.,.08)])
def test_return_discards_settled_pose_change_then_searches_fresh_origin(shift,yaw):
    planner,point,obstacle=setup()
    enter(planner,point,obstacle);acquire(planner,point,3.5)
    assert plan(planner,point,[],s=45.,offset=3.5,now=5.)['state']=='RETURN'
    search=ReturnSearch(ThreadPoolExecutor(max_workers=1))
    try:
        plan(planner,point,[],s=45.,offset=3.5,now=5.1,speed=0.,return_search=search)
        search._future.result(timeout=12.)
        result=plan(planner,point,[],s=45.+shift,offset=3.5,yaw=yaw,
                    now=5.2,speed=0.,return_search=search)
        assert result['reason']=='RETURN_SEARCH_PENDING'
        assert planner.return_commits==0 and not planner._return_path
        assert search._future is None and search._pose is None
        # A fresh stationary search may now authorize the unchanged gates.
        plan(planner,point,[],s=45.+shift,offset=3.5,yaw=yaw,
             now=5.3,speed=0.,return_search=search)
        search._future.result(timeout=12.)
        result=plan(planner,point,[],s=45.+shift,offset=3.5,yaw=yaw,
                    now=5.4,speed=0.,return_search=search)
        assert not result['blocked'] and planner.return_commits==1
    finally:search.close()
