import pathlib,runpy
from dataclasses import replace
from edgenode_static20_experiment.static20_planner import Static20Planner
from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner

fixture=runpy.run_path(str(pathlib.Path(__file__).parents[2]/'edgenode_normal_experiment/test/test_normal_safety.py'))
PATH=fixture['PATH'];box=fixture['box']
def planner():
    normal=fixture['planner']()
    p=Static20Planner(normal.network,config=replace(normal.config,integrated_strategy=True),weights=normal.weights,body=normal.body,route_provider=lambda:{})
    p._refresh_normal_bank=lambda kwargs:None
    return p

def test_clear_normal_path_keeps_phase_a_guard_and_never_invokes_lateral_planning(monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('unexpected lateral planner invocation')
    monkeypatch.setattr(LaneCandidatePlanner,'plan',forbidden);p=planner()
    try:
        result=p.plan(PATH,[],(0.,0.,0.),speed_kmh=60.,now_sec=0.,observation_stamp_ns=1)
        assert not result['blocked'] and not result['active'] and result['points']==PATH
        assert result['phase_a_normal_guard_implementation_sha256'] and result['phase_b_implementation_sha256']
    finally:p.close()

def test_current_risk_starts_original_confirmation_without_waiting_for_surface_shape_confirmation(monkeypatch):
    calls=[]
    def native(self,reference,*args,**kwargs):calls.append(kwargs);return self._result(reference)
    monkeypatch.setattr(LaneCandidatePlanner,'plan',native);p=planner()
    try:
        first=p.plan(PATH,[box(55,-1,2,4)],(0.,0.,0.),speed_kmh=20.,now_sec=0.,observation_stamp_ns=1)
        assert len(calls)==1 and first['speed_cap_kmh'] is not None
        p.plan(PATH,[box(55,-1,2,4)],(0.,0.,0.),speed_kmh=20.,now_sec=.1,observation_stamp_ns=2)
        assert len(calls)==2
    finally:p.close()

def test_vanished_provisional_return_cannot_start_lateral_preparation():
    p=planner()
    try:
        p.normal_guard.plan(PATH,[box(55,-1,2,4)],(0.,0.,0.),speed_kmh=20.,now_sec=0.,observation_stamp_ns=1)
        assert not p._preparation_conflict(PATH,[])
        assert p._preparation_conflict(PATH,[box(55,-1,2,4)])
    finally:p.close()

def test_preparation_uses_the_same_observation_uncertainty_as_immediate_normal_risk():
    p=planner()
    try:
        # Raw edge is clear of body+margin, but inside the admitted .1m
        # observation uncertainty. Preparation must not silently drop that
        # uncertainty and wait until the obstruction is much closer.
        edge=p.body.width_m/2+p.config.obstacle_margin_m+.05
        groups=[box(55,edge,.2,4)]
        result=p.normal_guard.plan(PATH,groups,(0.,0.,0.),speed_kmh=20.,now_sec=0.,observation_stamp_ns=1)
        assert result['phase_a_normal_safety']['first_conflict_distance_m'] is not None
        assert p._preparation_conflict(PATH,groups)
    finally:p.close()

def test_imminent_risk_stops_immediately_without_waiting_for_confirmation(monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('must stop before lateral dispatch')
    monkeypatch.setattr(LaneCandidatePlanner,'plan',forbidden);p=planner()
    try:
        result=p.plan(PATH,[box(8,-1,2,4)],(0.,0.,0.),speed_kmh=60.,now_sec=0.,observation_stamp_ns=1)
        assert result['blocked'] and result['speed_cap_kmh']==0 and result['reason'].startswith('B_NORMAL_')
    finally:p.close()

def test_search_snapshot_excludes_node_bound_normal_guard_and_its_lambda():
    p=planner()
    try:assert p.__getstate__()['normal_guard'] is None
    finally:p.close()

def test_confirmed_obstacle_survives_occlusion_before_speed_based_detection_trigger(monkeypatch):
    p=planner();p.config=replace(p.config,normal_detection_latency_s=1.6,normal_obstacle_confirmations=2)
    # The original A-only fixture never entered the static map-proof cache.
    # A real network has identity hashing; preserve its fixture geometry here.
    p.network=type('HashableNetwork',(),dict(vars(p.network)))()
    monkeypatch.setattr(p,'_detection_entry_distance',lambda *args:20.)
    try:
        for stamp in (1,2):
            result=p.plan(PATH,[box(55,-1,2,4)],(0.,0.,0.),speed_kmh=0.,now_sec=stamp*.1,observation_stamp_ns=stamp)
            assert result['state']=='NORMAL'
        # Now inside the trigger with the confirmed static face occluded.
        # The planner must prepare from its existing proof, not wait to see
        # the face again after consuming the lateral maneuver's approach.
        result=p.plan(PATH,[],(35.,0.,0.),speed_kmh=0.,now_sec=.3,observation_stamp_ns=3)
        assert result['state']=='DETECT' and result['reason']=='OBSTACLE_DETECTED'
    finally:p.close()
