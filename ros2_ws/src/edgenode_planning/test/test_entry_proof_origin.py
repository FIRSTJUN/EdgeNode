from concurrent.futures import Future
from dataclasses import replace
from edgenode_planning.entry_search import EntrySearch
from test_static_obstacle_planner import setup,plan


class CaptureExecutor:
    def submit(self,function,*args):
        self.arguments=args;return Future()
    def shutdown(self,**kwargs):pass


def captured(complete):
    p,point,obstacle=setup();p.complete_offset_search=complete
    p.config=replace(p.config,continuous_lane_follow=True,avoid_speed_kmh=10.)
    common=dict(offset=.04,yaw=.01,speed=10.,actual_curvature=.003)
    assert plan(p,point,[obstacle()],**common)['state']=='DETECT'
    executor=CaptureExecutor();search=EntrySearch(executor,budget_s=12.)
    try:
        plan(p,point,[],entry_search=search,**common)
        return executor.arguments
    finally:search.close()


def test_complete_maneuver_worker_uses_measured_origin_and_a_reserved_future_onset():
    snapshot,args=captured(True)
    assert args[1]==(0.,.04,.01) and args[3]==.003
    assert snapshot._entry_onset_floor_xy[0]>5.
    assert snapshot._entry_onset_floor_xy[1]==0.


def test_legacy_selection_keeps_its_existing_future_origin_contract():
    snapshot,args=captured(False)
    assert args[1][0]>5.
    assert args[1][1]==0.
    assert not hasattr(snapshot,'_entry_onset_floor_xy')
