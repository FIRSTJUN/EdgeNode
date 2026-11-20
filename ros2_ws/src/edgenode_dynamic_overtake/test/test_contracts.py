import math
from types import SimpleNamespace
import numpy as np
import pytest
from edgenode_dynamic_overtake.coverage import RaySupport
from edgenode_dynamic_overtake.trajectory import maneuver
from edgenode_dynamic_overtake.traffic import assess
from edgenode_planning.lane_geometry import LanePair
from edgenode_planning.vehicle_geometry import sonata_geometry

def test_absent_rear_returns_are_unproven_and_old_frames_never_clear_space():
    ray=RaySupport([[10,0,-1],[20,0,-1]],1_000_000_000)
    assert not ray.support((-5,0),now_ns=1_000_000_000)['proven']
    assert not ray.support((5,0),now_ns=1_700_000_000)['proven']
    assert not RaySupport([[-2,0,-.5],[-3,0,-.6]],1_000_000_000).support((-5,0),now_ns=1_000_000_000)['proven']
    assert RaySupport([[-10,0,-1],[-20,0,-2.4]],1_000_000_000).support((-5,0),now_ns=1_000_000_000)['proven']

def lead():return dict(id=1,center_xy=(35.,0.),velocity_xy_mps=(10/3.6,0.),vehicle_length_model_m=4.93,vehicle_width_model_m=2.12,heading_rad=0.,covariance=np.diag((.01,.01,.005,.005)).tolist(),observed_ns=1_000_000_000)

def test_finite_native_return_reserves_the_moving_lead_and_rejects_short_lane():
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    body=sonata_geometry();m=maneuver(pair,(5.,0.,0.),0.,20.,lead(),body)
    at_return=35.+10/3.6*(m['return_start_s']-5.)/(20/3.6)
    assert m['return_start_s']-body.rear_overhang_m-at_return-4.93/2>=9.-1e-6
    assert m['return_end_s']<138. and m['points'][0]==(5.,0.,0.)
    pair=LanePair([(0.,0.,0.),(60.,0.,0.)],[(0.,3.5,0.),(60.,3.5,0.)],[('native','adjacent')],'left',[60.])
    with pytest.raises(ValueError,match='RETURN_SPACE'):maneuver(pair,(5.,0.,0.),0.,20.,lead(),body)

def test_timed_prediction_rejects_actual_body_conflict_and_stale_tracks():
    points=[(float(i),0.,0.) for i in range(81)];body=sonata_geometry();t=lead();t['center_xy']=(5.,0.)
    result=assess(points,(0.,0.,0.),20.,[t],body,now_ns=1_000_000_000,source_ns=1_000_000_000)
    assert not result['ready'] and 'CONFLICT' in result['reason']
    t=lead();assert assess(points,(0.,0.,0.),20.,[t],body,now_ns=1_000_000_000,source_ns=1_000_000_000)['ready']
    assert not assess(points,(0.,0.,0.),20.,[t],body,now_ns=1_700_000_000,source_ns=1_000_000_000)['ready']

def test_road_motion_keeps_acceleration_response_but_reduces_lateral_noise_and_accepts_a_real_turn():
    from edgenode_dynamic_overtake.road_kinematics import RoadObstacleTracking
    from edgenode_dynamic_tracking.kinematics import Observation
    tracker=RoadObstacleTracking(heading_provider=lambda center:0.)
    def observation(x,y):return Observation((x,y),(((x-2,y-1),(x+2,y-1),(x+2,y+1),(x-2,y+1)),))
    for i in range(41):tracks=tracker.update(1_000_000_000+i*100_000_000,[observation(i*.2777778,0.)])
    t=tracks[0];assert t.speed_mps==pytest.approx(10/3.6,abs=.1)
    assert t.covariance[2,2]>t.covariance[3,3]*2
    for i in range(41,71):tracks=tracker.update(1_000_000_000+i*100_000_000,[observation(i*.2777778,(i-40)*.15)])
    observed=[t for t in tracks if t.observed_ns==8_000_000_000]
    assert observed and any(t.state[3]>.7 for t in observed)

def test_disarmed_planner_warms_native_geometry_without_starting_static_search(monkeypatch):
    import time
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    from edgenode_static20_experiment.static20_planner import Static20Planner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.coverage_provider=lambda:{};planner.state='NORMAL';planner.dynamic=None
    planner.traffic_provider=lambda:dict(ready=True,source_stamp_ns=time.time_ns(),tracks=[])
    planner._lead=lambda *args:None;events=[]
    planner._refresh_normal_bank=lambda kwargs:events.append('geometry_only')
    planner._follow=lambda *args:dict(state='NORMAL',raw_safety_checked=True)
    monkeypatch.setattr(Static20Planner,'plan',lambda *args,**kwargs:pytest.fail('disarmed static preparation must not run'))
    result=planner.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,0.,0.),observation_valid=True,motion_authorized=False)
    assert events==['geometry_only'] and result['raw_safety_checked']

def test_tracking_reset_null_epoch_holds_the_current_path_without_latching_or_changing_lanes():
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.coverage_provider=lambda:{};planner.state='AVOID_HOLD';fixed=[(0.,3.5,0.),(80.,3.5,0.)];planner.dynamic=dict(points=fixed)
    planner.traffic_provider=lambda:dict(ready=False,source_stamp_ns=None)
    planner._result=lambda path,**kwargs:dict(points=path,**kwargs)
    result=planner.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,3.5,0.),observation_valid=True,motion_authorized=True)
    assert result['points'] is fixed and result['speed_cap']==0. and planner.state=='AVOID_HOLD'
    assert planner.dynamic_debug['source_age_s'] is None

def test_source_owned_vehicle_motion_does_not_leave_static_ghosts_and_unowned_static_history_survives():
    from edgenode_dynamic_overtake.scene_memory import SceneMemory
    from edgenode_planning.lane_geometry import PolylineIndex
    from shapely.geometry import Point
    def piece(x,y):return [(x-1,y-.5),(x,y-.5),(x+1,y-.5),(x+1,y),(x+1,y+.5),(x,y+.5),(x-1,y+.5),(x-1,y)]
    value={};memory=SceneMemory(lambda:value);index=PolylineIndex([(0.,0.,0.),(80.,0.,0.)]);wall=piece(20,3.5)
    for i in range(4):
        ns=1_000_000_000+i*100_000_000;car=piece(10+3*i,0.);pieces=wall+car if i<2 else car
        value=dict(source_stamp_ns=ns,tracker_session_id='sensor',observation_scene=dict(source_ns=ns,points=pieces),tracks=[dict(id=1,observed_ns=ns,state_source_ns=ns,source_footprints_raw=[car],missed=False)])
        geometry=memory.update(([wall] if i<2 else [])+[car],index,0.,1.,.1,pose=(0.,0.,0.),stamp=ns,now=ns/1e9)
    assert geometry.intersects(Point(20,3.5)) and not geometry.intersects(Point(10,0.))
    assert memory.static.tracks[0]['confirmed'] and len(memory.vehicles)==1
    value=dict(source_stamp_ns=2_000_000_000,tracker_session_id='sensor',observation_scene=dict(source_ns=2_000_000_000,points=[]),tracks=[])
    geometry=memory.update([],index,0.,1.,.1,pose=(0.,0.,0.),stamp=2_000_000_000,now=2.)
    assert geometry.intersects(Point(19,0.)) and geometry.intersects(Point(20,3.5))

def test_rear_permission_uses_the_existing_road_polygon_sequence_contract():
    import time
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.body=sonata_geometry();planner.config=SimpleNamespace(avoid_speed_kmh=20.)
    checked=[]
    def covers(polygon):
        assert isinstance(polygon,tuple) and len(polygon)==4
        checked.append(polygon);return True
    planner.network=SimpleNamespace(road_corridor=SimpleNamespace(covers=covers));ns=time.time_ns()
    planner.coverage_provider=lambda:dict(ray=SimpleNamespace(source_ns=ns,support=lambda *args,**kwargs:dict(proven=True)),ego_source_pose=(10.,0.,0.))
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    assert planner._rear_proof(pair,(10.,0.,0.),dict(source_stamp_ns=ns))['proven'] and checked

def test_actual_overlapping_lidar_collection_has_exact_stable_visibility_predicates():
    import pathlib,json
    from shapely import wkt
    from shapely.ops import unary_union
    from edgenode_dynamic_overtake.scene_memory import collision_parts
    data=json.loads((pathlib.Path(__file__).parent/'data/actual-scene-predicate-failure.json').read_text())
    ray=wkt.loads(data['first_wkt']);original=wkt.loads(data['second_wkt']);parts=collision_parts(original)
    assert parts and all(p.geom_type!='GeometryCollection' for p in parts)
    exact=unary_union(list(original.geoms))
    assert unary_union(parts).symmetric_difference(exact).area<1e-10
    assert any(ray.intersects(piece) for piece in parts)==ray.intersects(exact)

def test_missed_moving_track_never_falls_through_to_static_lateral_search(monkeypatch):
    import time
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    from edgenode_static20_experiment.static20_planner import Static20Planner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.coverage_provider=lambda:{};planner.state='NORMAL';planner.dynamic=None
    planner.traffic_provider=lambda:dict(ready=True,source_stamp_ns=time.time_ns(),tracks=[dict(motion='MOVING',missed=True)])
    planner._lead=lambda *args:None;planner._follow=lambda *args,**kwargs:dict(state='NORMAL',reason=kwargs['reason'])
    monkeypatch.setattr(Static20Planner,'plan',lambda *args,**kwargs:pytest.fail('missed mover must not start static search'))
    result=planner.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,0.,0.),observation_valid=True,motion_authorized=True)
    assert result['reason']=='D_MOVING_OR_UNKNOWN_OBSERVATION_UNCERTAIN'

def test_uncommitted_static_preparation_is_cancelled_without_resetting_safety_history(monkeypatch):
    import time
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.state='DETECT';planner.dynamic=None;planner.dynamic_completed=2
    memory=SimpleNamespace(vehicles={'retained':dict(motion='MOVING')});guard=SimpleNamespace(occupancy=memory);planner.normal_guard=guard
    planner.coverage_provider=lambda:{};planner.traffic_provider=lambda:dict(ready=True,source_stamp_ns=time.time_ns(),tracks=[])
    planner._lead=lambda *args:None;planner._follow=lambda *args,**kwargs:dict(state=planner.state)
    def reset_core(p):p.state='NORMAL'
    monkeypatch.setattr(LaneCandidatePlanner,'reset',reset_core)
    result=planner.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,0.,0.),observation_valid=True,motion_authorized=True)
    assert result['state']=='NORMAL' and planner.dynamic_completed==2 and planner.normal_guard is guard and planner.normal_guard.occupancy is memory
    assert memory.vehicles['retained']['motion']=='MOVING'

def test_auxiliary_rear_source_failure_stops_an_active_maneuver_even_with_fresh_primary():
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner.state='AVOID_ENTRY';planner.dynamic={'points':[(0.,0.,0.),(80.,3.5,0.)]}
    planner.coverage_provider=lambda:dict(auxiliary_ready=False)
    planner.stop=lambda reason:dict(blocked=True,reason=reason)
    result=planner.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,0.,0.),observation_valid=True,motion_authorized=True)
    assert result['blocked'] and result['reason']=='D_AUXILIARY_REAR_SENSOR_STALE'

def test_rear_cloud_waits_for_later_source_pose_and_attitude_without_inventing_an_epoch():
    import base64
    from collections import deque
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from edgenode_dynamic_tracking.ego_history import EgoHistory
    from edgenode_dynamic_overtake.rear_sensing import RearSensingNode,decode_scene
    worker=RearSensingNode.__new__(RearSensingNode);ns=2_000_000_000
    worker.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ns+100_000_000))
    worker.ego=EgoHistory();worker.pending={};worker.frames={};worker.attitudes=deque();worker.errors=0
    h=Header();h.stamp.sec=2;cloud=create_cloud_xyz32(h,[[-10.,3.,-2.],[-20.,6.,-4.]])
    worker.cloud('roof',cloud);worker.process();assert ('roof',ns) in worker.pending and not worker.frames
    worker.ego.add(ns-20_000_000,(10.,20.,0.),5.);worker.ego.add(ns+20_000_000,(10.2,20.,0.),5.)
    worker.attitudes.append((ns,0.,0.));worker.process();assert not worker.pending and worker.errors==0
    row=worker.frames['roof'];assert row['source_ns']==ns and row['ego_source_pose']==pytest.approx((10.1,20.,0.))
    restored=decode_scene(dict(sensors=[row],auxiliary_ready=True));ray=restored['rear_sensors'][0]['ray']
    assert ray.source_ns==ns and np.array_equal(ray.xyz,np.array([[-10.,3.,-2.],[-20.,6.,-4.]])[np.argsort(np.arctan2([3.,6.],[-10.,-20.]))])

def test_speed_envelope_also_rejects_a_rear_vehicle_which_only_conflicts_at_lower_ego_speed():
    body=sonata_geometry();points=[(float(i),0.,0.) for i in range(81)];track=lead()
    track.update(center_xy=(-8.,0.),velocity_xy_mps=(15/3.6,0.),covariance=np.eye(4).tolist(),road_motion_conditioned=True)
    track['covariance']=(np.eye(4)*.001).tolist()
    fast=assess(points,(0.,0.,0.),20.,[track],body,now_ns=1_000_000_000,source_ns=1_000_000_000,acceleration_std=0.)
    interval=assess(points,(0.,0.,0.),20.,[track],body,now_ns=1_000_000_000,source_ns=1_000_000_000,acceleration_std=0.,minimum_speed_kmh=5.)
    assert fast['ready'] and not interval['ready'] and interval['reason']=='D_PREDICTED_TRAFFIC_CONFLICT'

def test_passed_vehicle_behind_does_not_limit_native_cruise_but_missing_or_ahead_location_remains_uncertain():
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);pose=(10.,0.,0.)
    behind=dict(tracks=[dict(motion='MOVING',center_xy=(0.,0.))]);ahead=dict(tracks=[dict(motion='MOVING',center_xy=(20.,0.))])
    assert not planner._forward_uncertainty(behind,None,pose)
    assert planner._forward_uncertainty(ahead,None,pose)
    assert planner._forward_uncertainty(dict(tracks=[dict(motion='MOVING',missed=True)]),None,pose)

def test_prepared_geometry_is_not_permission_and_current_rear_or_new_raw_hazard_rejects_it():
    import time
    from collections import OrderedDict
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    planner=DynamicPlanner.__new__(DynamicPlanner);planner._proposal_generation=1;planner.dynamic=None;planner.state='NORMAL';planner.dynamic_debug={}
    planner.config=SimpleNamespace(tracking_limit_m=.15,avoid_speed_kmh=20.,observation_padding_m=.1);planner.body=sonata_geometry()
    planner._map_sweep_cache=OrderedDict();planner._fixed_sweep_cache=OrderedDict();planner._inflate=lambda p,m:p
    planner.normal_guard=SimpleNamespace(plan=lambda *args,**kwargs:dict(blocked=False),occupancy=SimpleNamespace(static=SimpleNamespace(tracks=[])))
    path=[(float(i),0.,0.) for i in range(61)]
    probe=SimpleNamespace(dynamic=dict(points=path),pair=SimpleNamespace(side='left'),selected=object(),_road_area=object(),_map_sweep_cache={},_fixed_sweep_cache={})
    planner.network=SimpleNamespace(boundaries=object(),road_corridor=object());planner._geometry_context=lambda:('stable_map',)
    proposal=dict(executable=False,accepted_geometry=True,generation=1,current_link='native',tracker_session='s',lead_id=1,dynamic=probe.dynamic,pair=probe.pair,selected=probe.selected,owner_map_context=planner._map_identity(),config=planner.config,body=planner.body,map_proofs=[],fixed_sweeps=[])
    value=dict(source_stamp_ns=time.time_ns(),tracker_session_id='s',tracks=[]);lead_row=dict(track=dict(id=1));kwargs=dict(current_link_id='native',speed_kmh=18.,actual_curvature=0.)
    planner._rear_proof=lambda *args:dict(proven=False)
    assert not planner._admit_proposal(proposal,path,[],(0.,0.,0.),lead_row,value,kwargs)
    assert planner.dynamic is None and planner.state=='NORMAL'
    planner._rear_proof=lambda *args:dict(proven=True);seen=[]
    def current_hazard(points,obstacles,speed):
        seen.extend(obstacles);return 'BODY_COLLISION'
    planner._path_safety=current_hazard
    planner._rollout=lambda *args:pytest.fail('collision must reject before rollout')
    new_wall=[(10.,-1.),(12.,-1.),(12.,1.),(10.,1.)]
    assert not planner._admit_proposal(proposal,path,[new_wall],(0.,0.,0.),lead_row,value,kwargs)
    assert seen and planner.dynamic is None and planner.state=='NORMAL' and planner.selected is None
    assert planner.dynamic_debug['proposal_rejected']=='BODY_COLLISION'


def test_late_lane_position_is_safe_infeasibility_and_native_tail_covers_timed_handoff():
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    body=sonata_geometry()
    with pytest.raises(ValueError,match='ENTRY_SPACE'):maneuver(pair,(130.,0.,0.),0.,20.,lead(),body)
    m=maneuver(pair,(5.,0.,0.),0.,20.,lead(),body)
    assert m['native_handoff_tail_m']>=20/3.6*2.+3.
    assert m['points'][-1][0]==pytest.approx(m['return_end_s']+m['native_handoff_tail_m'])


def test_queued_rear_admission_precedes_planning_and_late_old_callback_cannot_roll_source_back(monkeypatch):
    import json,threading
    from std_msgs.msg import String
    import edgenode_dynamic_overtake.node as module
    node=module.DynamicPlanningNode.__new__(module.DynamicPlanningNode)
    node._d_input_lock=threading.RLock();node._d_drained=0;node.traffic={};node.rear={};node.rear_errors=0
    now=2_000_000_000
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now))
    monkeypatch.setattr(module,'implementation_sha256',lambda:'test')
    def scene(pub,source):return String(data=json.dumps(dict(implementation_sha256='test',published_ns=pub,auxiliary_source_ns=source,auxiliary_ready=True)))
    monkeypatch.setattr(module,'decode_scene',lambda v:dict(evidence_published_ns=v['published_ns'],auxiliary_source_ns=v['auxiliary_source_ns'],auxiliary_ready=True,auxiliary_groups=[],rear_sensors=[dict(name='d_roof_hdl64',ray=RaySupport([[-10.,0.,-2.]],v['auxiliary_source_ns']))]))
    class Handle:
        def __init__(self,msg):self.msg=msg
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def take_message(self,*args):message=self.msg;self.msg=None;return (message,None) if message is not None else None
    old=scene(now-300_000_000,now-700_000_000);new=scene(now-10_000_000,now-100_000_000)
    node.rear_scene(old);assert not node.coverage()['auxiliary_ready']
    node.d_rear_sub=SimpleNamespace(handle=Handle(new),msg_type=String,raw=False)
    node.drain_d_inputs();assert node.coverage()['auxiliary_ready'] and node._d_drained==1
    node.rear_scene(old);assert node.rear['auxiliary_source_ns']==now-100_000_000
    node.rear_scene(scene(now,now-700_000_000));assert not node.coverage()['auxiliary_ready']


def test_lane_clearance_uses_more_space_without_shrinking_vehicle_or_uncertainty():
    from edgenode_dynamic_overtake.lane_clearance import clearance_pair,parallel_clearance
    from edgenode_planning.lattice_path import curvatures
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    track=lead();track.update(road_motion_conditioned=True,covariance=np.diag((.025,.01,.15,.005)).tolist())
    body=sonata_geometry();center=parallel_clearance(pair,track,body);shifted=clearance_pair(pair,.35);reserved=parallel_clearance(shifted,track,body)
    assert not center['proven'] and reserved['minimum_gap_m']>center['minimum_gap_m']+.34
    assert pair.adjacent[0][1]==3.5 and shifted.adjacent[0][1]==pytest.approx(3.85)
    assert shifted.ids==pair.ids and shifted.original==pair.original
    m=maneuver(shifted,(5.,0.,0.),0.,20.,track,body)
    assert max(map(abs,curvatures(m['points'])))*(20/3.6)**2<1.5
    with pytest.raises(ValueError):clearance_pair(pair,.8)


def test_dynamic_input_admission_is_before_source_and_observation_decisions():
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    p=DynamicPlanner.__new__(DynamicPlanner);p.state='NORMAL';p.dynamic=None;p.input_admitter=lambda:events.append('take_complete_source')
    events=[];p.coverage_provider=lambda:(events.append('read_coverage') or {})
    p.traffic_provider=lambda:(events.append('read_tracks') or dict(ready=False,source_stamp_ns=None))
    p._result=lambda points,**kwargs:dict(points=points,**kwargs)
    p.plan([(0.,0.,0.),(80.,0.,0.)],[],(0.,0.,0.),observation_valid=False)
    assert events==['take_complete_source','read_coverage','read_tracks']


def test_lossless_ray_pointer_retains_exact_coordinates_epoch_and_detects_corruption(tmp_path,monkeypatch):
    import edgenode_dynamic_overtake.ray_transport as transport
    from edgenode_dynamic_overtake.rear_sensing import decode_scene
    monkeypatch.setattr(transport,'ROOT',tmp_path/'owned')
    xyz=np.array([[-10.125,3.4,-2.1],[-21.,6.7,-4.2]],dtype='<f8');files=transport.RayFiles();ref=files.write(xyz.tobytes(),2_000_000_000,'roof')
    assert np.array_equal(transport.read(ref,2_000_000_000),xyz)
    with pytest.raises(ValueError,match='immutable'):files.write((xyz+1).tobytes(),2_000_000_000,'roof')
    with pytest.raises(ValueError,match='epoch'):transport.read(ref,2_000_000_001)
    transport.verify_inline(xyz.tobytes(),2_000_000_000,ref)
    with pytest.raises(ValueError,match='archive'):transport.verify_inline((xyz+1).tobytes(),2_000_000_000,ref)
    from pathlib import Path
    Path(ref['path']).write_bytes(b'bad')
    with pytest.raises(ValueError,match='integrity'):transport.read(ref,2_000_000_000)
    Path(ref['path']).unlink()
    with pytest.raises(FileNotFoundError):transport.read(ref,2_000_000_000)


def test_maneuver_tail_can_reserve_the_full_control_rollout_plus_lookahead():
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    m=maneuver(pair,(5.,0.,0.),0.,20.,lead(),sonata_geometry(),minimum_handoff_tail_m=19.)
    assert m['native_handoff_tail_m']>=19. and m['points'][-1][0]==pytest.approx(m['return_end_s']+19.)


def test_deadline_deferral_keeps_source_clock_and_never_defers_fault_or_expired_input():
    from edgenode_dynamic_overtake.deadline import should_defer
    ns=2_000_000_000;source=ns-450_000_000
    status=dict(source_stamp_ns=source,frame_received_ns=source,published_ns=ns-10_000_000)
    guard=SimpleNamespace(max_age_ns=600_000_000,timeout_ns=600_000_000,matched=(status,[],ns-100_000_000,ns-100_000_000),status=status,steady_time_ns=lambda:ns)
    node=SimpleNamespace(sensor_guard=guard,latest_pose_stamp_ns=ns-20_000_000,traffic=dict(ready=True,source_stamp_ns=ns-100_000_000),_sensor_pause=None,sensor_reason=lambda now:'',coverage=lambda:dict(auxiliary_ready=True,auxiliary_source_ns=ns-100_000_000,evidence_published_ns=ns-10_000_000,rear_sensors=[]))
    deferred,remaining=should_defer(node,ns,.25);assert deferred and remaining==pytest.approx(.15)
    assert guard.matched[0]['source_stamp_ns']==source
    status['source_stamp_ns']=status['frame_received_ns']=ns-100_000_000
    assert not should_defer(node,ns,.25)[0]
    node.sensor_reason=lambda now:'SENSOR_FRAME_IDENTITY_CHANGED'
    assert not should_defer(node,ns,.25)[0]
    node.sensor_reason=lambda now:'';status['source_stamp_ns']=ns-610_000_000
    assert not should_defer(node,ns,.25)[0]


def test_auxiliary_new_complete_frame_emits_immediately_but_heartbeat_never_renews_source():
    import json
    from edgenode_dynamic_overtake.auxiliary_frame import AuxiliaryAdmission
    from edgenode_dynamic_overtake.rear_sensing import RearSensingNode
    from edgenode_planning.sensor_readiness import footprint_digest
    from std_msgs.msg import String
    ns=2_000_000_000;now=[ns+100_000_000]
    status=dict(schema_version=1,clock='ros',session_id='a'*32,status_seq=1,frame_seq=1,fault_seq=0,state='READY_EMPTY',source_stamp_ns=ns,frame_received_ns=ns,published_ns=now[0],cluster_count=0,obstacle_digest=footprint_digest('map',ns,[]),fault_history=[])
    def message():return String(data=json.dumps(dict(schema_version=1,frame_id='map',status=dict(status),points=[])))
    worker=RearSensingNode.__new__(RearSensingNode);worker.auxiliary=AuxiliaryAdmission();events=[]
    worker.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now[0]))
    worker.publish=lambda **kwargs:events.append(kwargs)
    worker.rear_frame(message());assert events==[dict(archive=False,drain=False)] and worker.auxiliary.snapshot(now[0])['ready']
    now[0]+=100_000_000;status.update(status_seq=2,published_ns=now[0]);worker.rear_frame(message());assert len(events)==1
    assert worker.auxiliary.snapshot(now[0])['source_ns']==ns
    now[0]+=100_000_000;status.update(status_seq=3,frame_seq=2,source_stamp_ns=now[0],frame_received_ns=now[0],published_ns=now[0],obstacle_digest=footprint_digest('map',now[0],[]));worker.rear_frame(message());assert len(events)==2
    prior=now[0];now[0]+=50_000_000;status.update(status_seq=4,fault_seq=1,state='LOCALIZATION_UNAVAILABLE',published_ns=now[0],fault_history=[dict(seq=1,state='LOCALIZATION_UNAVAILABLE')]);worker.rear_frame(message())
    snapshot=worker.auxiliary.snapshot(now[0]);assert snapshot['ready'] and snapshot['source_ns']==prior
    now[0]=prior+610_000_000;assert not worker.auxiliary.snapshot(now[0])['ready']


def test_auxiliary_digest_corruption_latches_and_cannot_be_hidden_by_a_later_good_frame():
    import json
    from edgenode_dynamic_overtake.auxiliary_frame import AuxiliaryAdmission
    from edgenode_planning.sensor_readiness import footprint_digest
    ns=2_000_000_000;gate=AuxiliaryAdmission()
    status=dict(schema_version=1,clock='ros',session_id='b'*32,status_seq=1,frame_seq=1,fault_seq=0,state='READY_EMPTY',source_stamp_ns=ns,frame_received_ns=ns,published_ns=ns,cluster_count=0,obstacle_digest='0'*64,fault_history=[])
    def value():return json.dumps(dict(schema_version=1,frame_id='map',status=dict(status),points=[]))
    gate.accept(value(),ns);assert not gate.snapshot(ns)['ready']
    status.update(status_seq=2,obstacle_digest=footprint_digest('map',ns,[]));gate.accept(value(),ns)
    assert not gate.snapshot(ns)['ready'] and gate.snapshot(ns)['reason']=='INVALID_SENSOR_FRAME'


def test_more_lateral_clearance_reserves_more_transition_distance_without_raising_acceleration():
    from edgenode_dynamic_overtake.lane_clearance import clearance_pair
    from edgenode_dynamic_overtake.trajectory import transition_length
    from edgenode_planning.lattice_path import curvatures
    pair=LanePair([(0.,0.,0.),(143.,0.,0.)],[(0.,3.5,0.),(143.,3.5,0.)],[('native','adjacent')],'left',[143.])
    shifted=clearance_pair(pair,.5);pose=(5.,0.,0.)
    assert transition_length(shifted,pose,20.)>transition_length(pair,pose,20.)
    m=maneuver(shifted,pose,0.,20.,lead(),sonata_geometry())
    assert max(map(abs,curvatures(m['points'])))*(20/3.6)**2<1.4


def test_view_consistent_partial_face_rejects_prediction_on_the_wrong_side_of_visible_surface():
    from edgenode_dynamic_overtake.vehicle_observation import vehicle_observation
    from edgenode_dynamic_tracking.vehicle_observation import vehicle_observation as completed_c
    from edgenode_dynamic_tracking.kinematics import Observation
    matcher=SimpleNamespace(links=[dict(idx='native',points=[(0.,0.,0.),(100.,0.,0.)],_bbox=(0.,-2.,100.,2.))],bbox_candidate=lambda *args:True)
    for side in (-1.,1.):
        face=((20.,.6*side),(24.6,.6*side),(24.6,1.06*side),(20.,1.06*side))
        raw=Observation((22.3,.83*side),(face,));arguments=dict(predicted_centers=[(22.3,1.66*side)])
        before,_=completed_c(raw,(40.,5.*side,0.),matcher,**arguments)
        after,debug=vehicle_observation(raw,(40.,5.*side,0.),matcher,**arguments)
        assert abs(before.center_xy[1])>1. and abs(after.center_xy[1])<1e-8
        assert after.footprints is before.footprints and after.model_width_m==before.model_width_m
        assert debug['view_constraints'] and debug['raw_collision_solids_unchanged']


def test_joint_visible_parts_keep_one_existing_vehicle_id_but_separate_adjacent_cars():
    from edgenode_dynamic_overtake.view_tracker import ViewConsistentTracker,joint_surface_support
    from edgenode_dynamic_overtake.road_kinematics import RoadObstacleTracking
    from edgenode_dynamic_tracking.kinematics import Observation
    def rect(x0,x1,y0,y1):return ((x0,y0),((x0+x1)/2,y0),(x1,y0),(x1,(y0+y1)/2),(x1,y1),((x0+x1)/2,y1),(x0,y1),(x0,(y0+y1)/2))
    p=ViewConsistentTracker.__new__(ViewConsistentTracker);p.matcher=SimpleNamespace(links=[dict(idx='native',points=[(0.,0.,0.),(100.,0.,0.)],_bbox=(0.,-2.,100.,8.))],bbox_candidate=lambda *args:True)
    p.filter=RoadObstacleTracking(heading_provider=lambda center:0.);p.length_prior=4.93;p.width_prior=2.12;p.reset()
    def frame(ns,parts):return dict(status=dict(source_stamp_ns=ns),points=[v for part in parts for v in part],objects=[dict(center_xy=np.asarray(part).mean(axis=0).tolist(),footprint_indices=[i],point_count=50) for i,part in enumerate(parts)])
    for i in range(30):
        center=20.+i*.2;p.update(frame(1_000_000_000+i*100_000_000,[rect(center-2.465,center+2.465,-1.06,1.06)]),(center+7.,6.,0.))
    center+=.2;parts=[rect(center+2.,center+2.45,.5,1.06),rect(center-2.4,center+.9,-1.06,1.06)]
    p.update(frame(4_000_000_000,parts),(center+7.,6.,0.));assert len(p.filter.tracks)==1
    track=next(iter(p.filter.tracks.values()));assert track.identifier==1 and len(track.observation.footprints)==2 and track.observed_ns==4_000_000_000
    first=Observation((center,0.),(rect(center-2.465,center+2.465,-1.06,1.06),));adjacent=Observation((center,3.5),(rect(center-2.465,center+2.465,2.44,4.56),))
    assert not joint_surface_support(first,adjacent,p.matcher,4.93,2.12)
    ahead=Observation((center+7.,0.),(rect(center+4.535,center+9.465,-1.06,1.06),))
    assert not joint_surface_support(first,ahead,p.matcher,4.93,2.12)


def test_partial_side_support_does_not_invent_front_face_or_shrink_unobserved_position_covariance():
    from edgenode_dynamic_overtake.vehicle_observation import PartialVehicleObservation
    from edgenode_dynamic_overtake.road_kinematics import RoadObstacleTracking
    from edgenode_dynamic_tracking.kinematics import Observation
    tracker=RoadObstacleTracking(heading_provider=lambda center:0.)
    for i in range(30):tracker.update(1_000_000_000+i*100_000_000,[Observation((20.+i*.2,0.),(),model_length_m=4.93,model_width_m=2.12)])
    track=tracker.tracks[1];predicted=track.state[0]+track.state[2]*.1;before=track.covariance.copy();history=len(track.history)
    partial=PartialVehicleObservation((predicted,0.),(),None,4.93,2.12,((0.,1.),),(((1.,0.),predicted-1.,predicted+1.),))
    tracker.update(4_000_000_000,[partial]);track=tracker.tracks[1]
    assert len(tracker.tracks)==1 and track.state[0]==pytest.approx(predicted,abs=1e-8)
    assert track.covariance[0,0]>before[0,0] and len(track.history)==history
    assert track.observed_ns==4_000_000_000 and np.linalg.eigvalsh(track.covariance).min()>0.
def test_validation_budget_expires_without_execution_and_follow_deferral_never_admits():
    from edgenode_dynamic_overtake.deadline import ValidationBudget
    from edgenode_dynamic_overtake.planner import DynamicPlanner
    budget=ValidationBudget(.27,.25)
    budget.record(10.,.5)
    assert budget.estimate(11.)==pytest.approx(.52)
    assert budget.estimate(12.1)==pytest.approx(.29)
    planner=object.__new__(DynamicPlanner);planner.lateral_admission_allowed=False;planner.dynamic_debug={}
    assert not planner._try_entry(None,None,None,None,None,{})
    assert planner.dynamic_debug['lateral_admission_deferred_for_safe_follow']

def test_return_space_reservation_includes_measured_acceleration_delay():
    pair=LanePair([(0.,0.,0.),(200.,0.,0.)],[(0.,3.5,0.),(200.,3.5,0.)],[('native','adjacent')],'left',[200.])
    body=sonata_geometry();target=lead()
    constant=maneuver(pair,(5.,0.,0.),0.,20.,target,body)
    accelerating=maneuver(pair,(5.,0.,0.),0.,20.,target,body,initial_speed_kmh=10.)
    assert accelerating['return_start_s']>constant['return_start_s']+3.
    def gap(plan,penalty):
        elapsed=(plan['return_start_s']-5.)/(20/3.6)+penalty
        return plan['return_start_s']-body.rear_overhang_m-35.-10/3.6*elapsed-4.93/2
    assert gap(constant,accelerating['acceleration_time_penalty_s'])<8.
    assert gap(accelerating,accelerating['acceleration_time_penalty_s'])>=9.-1e-6

