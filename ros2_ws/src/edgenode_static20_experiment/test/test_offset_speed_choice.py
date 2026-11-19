from dataclasses import replace
import runpy,pathlib
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.search_budget import SearchBudgetExceeded
helpers=runpy.run_path(str(pathlib.Path(__file__).with_name('test_normal_handoff.py')))
PATH=helpers['PATH']
def planner():
 p=helpers['planner']();p.config=replace(p.config,avoid_speed_kmh=20.);p.requested_avoidance_kmh=20.
 return p

def test_proven_twenty_is_selected_without_slower_probes(monkeypatch):
 calls=[]
 def solve(self,*args,**kwargs):calls.append(self.config.avoid_speed_kmh);self.state='AVOID_ENTRY';return self._result(PATH)
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',solve);p=planner()
 try:
  result=p._offset_speed_search(PATH,(0.,0.,0.),20.);assert not result['blocked'];assert calls==[20.];assert p.config.avoid_speed_kmh==20.
 finally:p.close()

def test_bounded_high_speed_failures_select_a_lower_complete_proof_without_relaxing_limits(monkeypatch):
 calls=[]
 def solve(self,*args,**kwargs):
  calls.append(self.config)
  if self.config.avoid_speed_kmh>10:raise SearchBudgetExceeded()
  self.state='AVOID_ENTRY';return self._result(PATH)
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',solve);p=planner();original=p.config
 try:
  result=p._offset_speed_search(PATH,(0.,0.,0.),20.);assert not result['blocked'] and [v.avoid_speed_kmh for v in calls]==[20.,15.,10.]
  assert all(replace(v,avoid_speed_kmh=20.)==original for v in calls)
  assert p.requested_avoidance_kmh==20. and p.config.avoid_speed_kmh==10. and result['speed_cap_kmh']==10.
 finally:p.close()

def test_map_rejection_remains_a_hard_rejection_at_all_speeds(monkeypatch):
 calls=[]
 def solve(self,*args,**kwargs):calls.append(self.config.avoid_speed_kmh);return self._block('ROAD_AREA_UNPROVEN')
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',solve);p=planner()
 try:assert p._offset_speed_search(PATH,(0.,0.,0.),20.)['blocked'] and calls==[20.]
 finally:p.close()

def test_completed_maneuver_releases_the_selected_lower_speed(monkeypatch):
 from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
 def done(self,reference,*args,**kwargs):self.state='NORMAL';return self._result(reference)
 monkeypatch.setattr(LaneCandidatePlanner,'plan',done);p=planner();p.state='RETURN';p.config=replace(p.config,avoid_speed_kmh=10.)
 try:
  result=p.plan(PATH,[],(0.,0.,0.),speed_kmh=10.,observation_stamp_ns=1,now_sec=0.)
  assert p.config.avoid_speed_kmh==20. and result['effective_avoidance_kmh']==20. and result['speed_cap_kmh'] is None
 finally:p.close()

def test_each_lower_proof_uses_its_conditional_adoption_speed(monkeypatch):
 calls=[]
 def solve(self,reference,pose,speed,*args,**kw):
  calls.append((self.config.avoid_speed_kmh,speed))
  if self.config.avoid_speed_kmh>10:raise SearchBudgetExceeded()
  self.state='AVOID_ENTRY';return self._result(PATH)
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',solve);p=planner()
 try:
  assert not p._offset_speed_search(PATH,(0.,0.,0.),19.8)['blocked']
  assert calls==[(20.,19.8),(15.,15.),(10.,10.)]
 finally:p.close()

def test_alignment_consumed_budget_still_reserves_a_complete_lower_speed_proof(monkeypatch):
 from edgenode_static20_experiment import static20_planner as module
 now=[100.];windows=[]
 monkeypatch.setattr(module.time,'perf_counter',lambda:now[0])
 def solve(self,*args,**kw):
  windows.append((self.config.avoid_speed_kmh,self._search_deadline_sec-now[0]))
  if self.config.avoid_speed_kmh>10:
   now[0]=self._search_deadline_sec;raise SearchBudgetExceeded()
  self.state='AVOID_ENTRY';return self._result(PATH)
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',solve);p=planner();p._search_deadline_sec=108.75
 try:
  assert not p._offset_speed_search(PATH,(0.,0.,0.),19.8)['blocked']
  assert windows==[(20.,3.),(15.,1.75),(10.,4.)]
  assert p._search_deadline_sec==108.75
 finally:p.close()

def test_a_lower_native_proof_is_considered_after_requested_offset_failure(monkeypatch):
 from types import SimpleNamespace
 from edgenode_static20_experiment.static20_planner import Static20Planner
 calls=[]
 def offset(self,*args,**kwargs):
  calls.append(('offset',self.config.avoid_speed_kmh));raise SearchBudgetExceeded()
 def native(self,*args,**kwargs):
  assert kwargs['_native_only'];calls.append(('native',self.config.avoid_speed_kmh))
  self.state='AVOID_ENTRY';self.selected=SimpleNamespace(mode='lane',side='left',complete_offset=False,complete_return=True)
  return self._result(PATH)
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',offset)
 monkeypatch.setattr(Static20Planner,'_search_entry',native);p=planner()
 try:
  result=p._offset_speed_search(PATH,(0.,0.,0.),20.,lane_search_exhausted=True)
  assert not result['blocked'] and p.config.avoid_speed_kmh==15.
  assert calls==[('offset',20.),('native',15.)]
 finally:p.close()

def test_missing_lower_native_proof_keeps_the_same_speed_offset_fallback(monkeypatch):
 from edgenode_static20_experiment.static20_planner import Static20Planner
 calls=[]
 def offset(self,*args,**kwargs):
  calls.append(('offset',self.config.avoid_speed_kmh))
  if self.config.avoid_speed_kmh==20.:raise SearchBudgetExceeded()
  self.state='AVOID_ENTRY';return self._result(PATH)
 def native(self,*args,**kwargs):
  calls.append(('native',self.config.avoid_speed_kmh));return self._block('NO_VALID_ADJACENT_LANE')
 monkeypatch.setattr(StaticObstaclePlanner,'_search_entry',offset)
 monkeypatch.setattr(Static20Planner,'_search_entry',native);p=planner()
 try:
  assert not p._offset_speed_search(PATH,(0.,0.,0.),20.,lane_search_exhausted=True)['blocked']
  assert calls==[('offset',20.),('native',15.),('offset',15.)]
 finally:p.close()
