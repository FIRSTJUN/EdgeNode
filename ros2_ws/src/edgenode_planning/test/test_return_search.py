"""STEP 4 work cannot starve callbacks or authorize an obsolete result."""
from threading import Event
from unittest.mock import patch

from edgenode_planning.return_search import ReturnSearch
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from test_static_obstacle_planner import setup, enter, acquire, plan


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
    search=ReturnSearch()
    try:
        with patch.object(StaticObstaclePlanner,'_generate_return',slow):
            r=plan(p,point,[],s=45.,offset=3.5,now=5.1,return_search=search)
            assert started.wait(1.)
            assert r['stage']=='RETURN' and r['blocked'] and not r['points']
            assert p.return_commits==0 and not p._return_path
            # Another ROS planning cycle remains available during the search.
            r=plan(p,point,[],s=45.,offset=3.5,now=5.2,return_search=search)
            assert r['reason']=='RETURN_SEARCH_PENDING' and r['blocked']
            release.set();search._future.result(timeout=5.)
        # A newly observed solid on the return path must be revalidated even
        # though it was absent from the worker's snapshot.
        solid=obstacle(60.,64.,-1.,1.)
        r=plan(p,point,[solid],s=45.,offset=3.5,now=5.3,return_search=search)
        assert r['blocked'] and not r['points'] and p.reason in ('BODY_COLLISION','REFERENCE_CLEARANCE')
        p.reset();search.cancel()
        assert p.state=='NORMAL' and search._future is None
    finally:
        release.set();search.close()
