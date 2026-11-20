"""D-only dynamic passing; B static behavior is delegated outside an episode."""
import math,time
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from copy import copy
import os
from .preparation import initialize,prepare_current
import numpy as np
from edgenode_static20_experiment.static20_planner import Static20Planner
from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
from edgenode_planning.lane_geometry import PolylineIndex,sample,arc
from edgenode_planning.lattice_path import LatticeCandidate,control_local_path,heading_error
from edgenode_planning.lane_candidate_bank import reverse_crossing_reason
from edgenode_planning.dynamic_traffic import following_speed_kmh
from edgenode_planning.avoidance_geometry import obstacle_polygons
from .trajectory import maneuver,APPROACH_M,transition_length
from .traffic import assess,rectangle
from .scene_memory import SceneMemory,collision_parts
from .lane_clearance import clearance_pair,parallel_clearance
from .provenance import implementation_sha256

class DynamicPlanner(Static20Planner):
    def __init__(self,*args,traffic_provider,coverage_provider,input_admitter=None,lane_clearance_offsets_m=(0.,.2,.35,.5),**kwargs):
        self.input_admitter=input_admitter;self.lateral_admission_allowed=True
        self.lane_clearance_offsets_m=tuple(lane_clearance_offsets_m)
        if not self.lane_clearance_offsets_m or any(not 0.<=x<=.5 for x in self.lane_clearance_offsets_m):raise ValueError('D lane clearance offsets outside supported bound')
        self._proposal_executor=None;self._proposal_future=None;self._proposal_generation=0;self._proposal_map_context=None
        self.traffic_provider=traffic_provider;self.coverage_provider=coverage_provider;self.dynamic=None;self.dynamic_debug={};self.dynamic_completed=0;self.dynamic_pose=None
        super().__init__(*args,**kwargs)
        self.normal_guard.occupancy=SceneMemory(self.traffic_provider)
        self._proposal_executor=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'),initializer=initialize,initargs=(copy(self),));self._proposal_warmup=self._proposal_executor.submit(os.getpid)
    def close(self):
        super().close()
        if self._proposal_executor is not None:self._proposal_executor.shutdown(wait=True,cancel_futures=True)
    def __getstate__(self):
        state=super().__getstate__();state['traffic_provider']=None;state['coverage_provider']=None;state['input_admitter']=None;state['dynamic']=None;state['_proposal_executor']=None;state['_proposal_future']=None;state['_proposal_warmup']=None;state['_proposal_map_context']=None
        return state
    def reset(self):
        self._proposal_generation=getattr(self,'_proposal_generation',0)+1
        pending=getattr(self,'_proposal_future',None)
        if pending is not None:pending.cancel()
        self._proposal_future=None
        super().reset();self.dynamic=None;self.dynamic_debug={};self.dynamic_completed=0;self.dynamic_pose=None
        if getattr(self,'normal_guard',None) is not None:self.normal_guard.occupancy=SceneMemory(self.traffic_provider)
    def _result(self,*args,**kwargs):
        result=super()._result(*args,**kwargs)
        result.update(phase_d_implementation_sha256=implementation_sha256(),dynamic_debug=self.dynamic_debug,dynamic_completed=self.dynamic_completed,collision_solids_never_removed=True)
        if self.dynamic is not None:result['selected_strategy']='DYNAMIC_OVERTAKE'
        return result
    def _follow(self,reference,groups,pose,kwargs,lead=None,reason='D_TRACK_WARMUP',native_clear=False):
        normal=self.normal_guard.plan(reference,groups,pose,**kwargs)
        if normal['blocked']:return self.stop('D_FOLLOW_'+normal['reason'])
        cap=(self.config.avoid_speed_kmh if native_clear else 5.) if lead is None else following_speed_kmh(lead['gap_m'],lead['forward_speed_mps']*3.6,self.config.avoid_speed_kmh)
        current=normal.get('speed_cap_kmh');cap=cap if current is None else min(cap,current)
        self.dynamic_debug.update(decision='FOLLOW',reason=reason,following_speed_kmh=cap)
        self.state='NORMAL';self._last_pose=pose
        result=self._result(reference,speed_cap=cap);result['active']=False
        return result
    def _forward_uncertainty(self,value,memory,pose):
        forward=np.array((math.cos(pose[2]),math.sin(pose[2])))
        for track in value.get('tracks',[]):
            if track.get('motion') not in ('UNKNOWN','MOVING'):continue
            center=track.get('center_xy')
            if center is None or float((np.asarray(center)-np.asarray(pose[:2]))@forward)>=0.:return True
        for record in getattr(memory,'vehicles',{}).values():
            if record.get('motion') not in ('UNKNOWN','MOVING'):continue
            shape=record.get('shape')
            if shape is None or shape.is_empty:return True
            center=shape.centroid
            if float((np.array((center.x,center.y))-np.asarray(pose[:2]))@forward)>=0.:return True
        return False
    def _lead(self,value,reference,pose):
        index=PolylineIndex(reference);ego=index.project(pose[:2]);candidates=[]
        for t in value.get('tracks',[]):
            if not t['usable'] or t['motion']!='MOVING' or t['missed'] or not t.get('road_motion_conditioned',False):continue
            s=index.project(t['center_xy']);point,tangent=sample(reference,max(0.,min(index.lengths[-1],s)))
            delta=np.asarray(t['center_xy'])-np.asarray(point[:2]);lateral=abs(float(delta@np.array((-tangent[1],tangent[0]))))
            gap=s-ego-t['vehicle_length_model_m']/2-self.body.wheelbase_m-self.body.front_overhang_m
            speed=float(np.asarray(t['velocity_xy_mps'])@np.asarray(tangent))
            if gap>0 and lateral<1. and speed>.3:candidates.append(dict(track=t,gap_m=gap,forward_speed_mps=speed))
        return min(candidates,key=lambda x:x['gap_m']) if candidates else None
    def _rear_proof(self,pair,pose,value,minimum_ego_speed_kmh=None):
        evidence=self.coverage_provider();stamp=value['source_stamp_ns'];now=time.time_ns()
        sensors=evidence.get('rear_sensors')
        if sensors is None:
            sensors=[dict(ray=evidence.get('ray'),ego_source_pose=evidence.get('ego_source_pose'),offset_x_m=1.19,offset_y_m=0.,minimum_belt_z=-.9,maximum_belt_z=-.4,name='original_main_lidar')]
        if evidence.get('auxiliary_ready') is False:return dict(proven=False,reason='D_AUXILIARY_REAR_SENSOR_UNPROVEN')
        sensors=[r for r in sensors if r.get('ray') is not None and r.get('ego_source_pose') is not None and abs(r['ray'].source_ns-stamp)<=200_000_000 and -.05<=(now-r['ray'].source_ns)/1e9<=.6]
        if not sensors:return dict(proven=False,reason='D_RAW_REAR_PAIR_UNAVAILABLE')
        station=pair.original_index.project(pose[:2]);p,tangent=sample(pair.original,station);adj=pair.adjacent_index.project(p[:2]);target=sample(pair.adjacent,adj)[0]
        # Posted-speed D ODD; F must select its own rear-speed bound.
        ego=self.config.avoid_speed_kmh/3.6 if minimum_ego_speed_kmh is None else min(self.config.avoid_speed_kmh,minimum_ego_speed_kmh)/3.6
        if ego<1.:return dict(proven=False,reason='D_REAR_SLOW_EGO_DURATION_UNPROVEN')
        max_rear=30./3.6;duration=(APPROACH_M+transition_length(pair,pose,self.config.avoid_speed_kmh))/ego
        rear=math.ceil(max(12.,(max_rear-ego)*duration+12.)/2)*2;rows=[]
        for distance in np.arange(2.,rear+1.,2.):
            at=adj-distance
            if at<0:
                # Ray checks remain valid in sensor coordinates; native lane
                # existence behind its start is separately unproved.
                center=np.asarray(target[:2])-distance*np.asarray(tangent)
            else:center=np.asarray(sample(pair.adjacent,at)[0][:2])
            road=getattr(self.network,'road_corridor',None)
            if road is None or not road.covers(rectangle(center,math.atan2(tangent[1],tangent[0]),4.93,self.body.width_m)):
                return dict(proven=False,reason='D_REAR_SURVEYED_ROAD_UNPROVEN')
            for offset in (-self.body.width_m/2,0.,self.body.width_m/2):
                xy=center+offset*np.array((-tangent[1],tangent[0]));proof=[]
                for row in sensors:
                    epoch=row['ego_source_pose'];c=math.cos(epoch[2]);s=math.sin(epoch[2]);delta=xy-np.asarray(epoch[:2])
                    sensor=(delta[0]*c+delta[1]*s-row['offset_x_m'],-delta[0]*s+delta[1]*c-row['offset_y_m'])
                    hit=row['ray'].support(sensor,now_ns=now,minimum_belt_z=row['minimum_belt_z'],maximum_belt_z=row['maximum_belt_z']);hit['sensor']=row['name'];proof.append(hit)
                rows.append(dict(proven=any(x['proven'] for x in proof),sensors=proof))
        return dict(proven=bool(rows) and all(x['proven'] for x in rows),rear_distance_m=rear,targets=len(rows),supported=sum(x['proven'] for x in rows),unproven=sum(not x['proven'] for x in rows),sensor_names=[r['name'] for r in sensors],maximum_rear_speed_kmh=30.,assumption='D_POSTED_SPEED_LANE_KEEPING_TRAFFIC')
    def _build_entry(self,reference,groups,pose,lead,value,kwargs,geometry_only=False):
        self._refresh_normal_bank(kwargs);snapshot=self._lane_snapshot;checks=[];self.dynamic_debug['candidates']=checks
        if snapshot is None:return False
        for option,offset in ((o,d) for o in sorted(snapshot.options,key=lambda o:o.hops) for d in sorted(self.lane_clearance_offsets_m,reverse=True)):
            if option.hops!=1 or option.pair is None or option.exit_reason:continue
            check=dict(lane=option.lane_id,clearance_offset_m=offset);checks.append(check)
            if reverse_crossing_reason(self.network,option.crossings):check['rejected']='D_RETURN_PERMISSION';continue
            pair=clearance_pair(option.pair,offset)
            parallel=parallel_clearance(pair,lead['track'],self.body);check['parallel_reservation']=parallel
            if not parallel['proven']:check['rejected']='D_PARALLEL_CLEARANCE_RESERVATION_UNPROVEN';continue
            if not geometry_only:
                rear=self._rear_proof(pair,pose,value,kwargs['speed_kmh']);check['rear']=rear
                if not rear['proven']:check['rejected']='D_REAR_UNPROVEN';continue
            else:check['rear_permission_not_evaluated']=True
            measured_speed=min(kwargs['speed_kmh'],self.config.avoid_speed_kmh)
            try:m=maneuver(pair,pose,kwargs['actual_curvature'],self.config.avoid_speed_kmh,lead['track'],self.body,minimum_handoff_tail_m=self._rollout_distance(self.config.avoid_speed_kmh)+3.,initial_speed_kmh=kwargs['speed_kmh'])
            except ValueError as error:
                check['rejected']=str(error);check['infeasible_context']=getattr(error,'context',{});continue
            m['lane_clearance_offset_m']=offset
            # No mandatory route node may be skipped before this native return.
            ids=[a for a,b in pair.ids];ends=pair.section_ends
            if any(m['entry_end_s']<end<m['return_end_s'] for end in ends[:-1]):check['rejected']='D_INTERMEDIATE_MISSION_NODE';continue
            self.pair=pair;self._road_area=getattr(self.network,'road_corridor',None)
            self.selected=LatticeCandidate(pair.side,m['points'],m['entry_end_s'],m['return_start_s'],m['return_end_s'],m['transition_m'],0.,{},entry_path=m['entry'],return_path=m['back'],kinematic_points=m['points'],complete_return=True)
            self._boundary_check_mode='avoidance';self._observation_curvature_limit=None;self._min_clearance=math.inf
            safe_raw=tuple(self._inflate(p,self.config.observation_padding_m) for p in obstacle_polygons(groups))
            static=tuple(tuple(part.exterior.coords)[:-1] for record in self.normal_guard.occupancy.static.tracks for part in collision_parts(record['shape']) if part.geom_type=='Polygon')
            reason=self._path_safety(m['points'],safe_raw+static,self.config.avoid_speed_kmh)
            if not reason:reason=self._rollout(m['points'],pose,self.config.avoid_speed_kmh,kwargs['actual_curvature'],safe_raw)
            if reason:check['rejected']=reason;self.selected=None;self.pair=None;self._road_area=None;continue
            if not geometry_only:
                risk=assess(m['points'],pose,max(kwargs['speed_kmh'],self.config.avoid_speed_kmh),[t for t in value['tracks'] if t.get('lane') is not None and t.get('heading_rad') is not None],self.body,now_ns=time.time_ns(),source_ns=value['source_stamp_ns'],minimum_speed_kmh=measured_speed)
                check['timed_prediction']=risk
                if not risk['ready']:check['rejected']=risk['reason'];self.selected=None;self.pair=None;self._road_area=None;continue
            else:check['timed_permission_not_evaluated']=True
            self.dynamic=dict(m,lead_id=lead['track']['id'],tracker_session=value['tracker_session_id'],original_ids=ids)
            self._original=pair.original;setattr(self,pair.side+'_lane',pair.adjacent);self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1;self.dynamic_pose=pose
            self.dynamic_debug.update(decision='ENTRY',plan={k:v for k,v in m.items() if k not in ['points','entry','hold','back']},nominal_completion_is_not_live_permission=True)
            return True
        return False
    def _clear_proposed_selection(self):
        self.selected=None;self.pair=None;self._road_area=None;self._boundary_check_mode='diagnostic'
    def _admit_proposal(self,proposal,reference,groups,pose,lead,value,kwargs):
        if proposal['generation']!=self._proposal_generation or proposal['current_link']!=kwargs.get('current_link_id') or proposal['tracker_session']!=value['tracker_session_id'] or proposal['lead_id']!=lead['track']['id']:return False
        d=proposal['dynamic']
        if not proposal['accepted_geometry'] or d is None:return False
        index=PolylineIndex(d['points']);station=index.project(pose[:2]);nearest=sample(d['points'],max(0.,min(index.lengths[-1],station)),index.lengths)
        if math.dist(pose[:2],nearest[0][:2])>self.config.tracking_limit_m:
            self.dynamic_debug['proposal_rejected']='D_PREPARED_GEOMETRY_TRACKING_UNPROVEN';return False
        pair=proposal['pair']
        started=time.perf_counter();timing={};mark=started
        rear=self._rear_proof(pair,pose,value,kwargs['speed_kmh']);self.dynamic_debug['latest_rear']=rear
        timing['rear_ms']=(time.perf_counter()-mark)*1000;mark=time.perf_counter();self.dynamic_debug['admission_timing_ms']=timing
        if not rear['proven']:return False
        self.selected=proposal['selected'];self.pair=pair;self._road_area=getattr(self.network,'road_corridor',None);self._boundary_check_mode='avoidance';self._observation_curvature_limit=None;self._min_clearance=math.inf
        if proposal['owner_map_context']!=self._map_identity() or proposal['config']!=self.config or proposal['body']!=self.body:
            self._clear_proposed_selection();return False
        context=self._geometry_context()
        self._map_sweep_cache.update(((context,first,second),result) for first,second,result in proposal['map_proofs'])
        self._fixed_sweep_cache.update(proposal['fixed_sweeps'])
        timing['map_proof_adoption_ms']=(time.perf_counter()-mark)*1000;mark=time.perf_counter()
        window=control_local_path(d['points'],pose,max(20.,kwargs['speed_kmh']/3.6*3.))
        normal=self.normal_guard.plan(window,groups,pose,**kwargs)
        timing['normal_ms']=(time.perf_counter()-mark)*1000;mark=time.perf_counter()
        if normal['blocked']:
            self._clear_proposed_selection();self.stop('D_ADMISSION_'+normal['reason']);return False
        raw=tuple(self._inflate(p,self.config.observation_padding_m) for p in obstacle_polygons(groups))
        static=tuple(tuple(part.exterior.coords)[:-1] for record in self.normal_guard.occupancy.static.tracks for part in collision_parts(record['shape']) if part.geom_type=='Polygon')
        speed=max(kwargs['speed_kmh'],self.config.avoid_speed_kmh)
        reason=self._path_safety(d['points'],raw+static,speed)
        timing['path_ms']=(time.perf_counter()-mark)*1000;mark=time.perf_counter()
        if not reason:reason=self._rollout(d['points'],pose,speed,kwargs['actual_curvature'],raw)
        timing['rollout_ms']=(time.perf_counter()-mark)*1000;mark=time.perf_counter()
        if reason:
            self.dynamic_debug['proposal_rejected']=reason;self._clear_proposed_selection();return False
        risk=assess(d['points'],pose,speed,[t for t in value['tracks'] if t.get('lane') is not None and t.get('heading_rad') is not None],self.body,now_ns=time.time_ns(),source_ns=value['source_stamp_ns'],minimum_speed_kmh=min(kwargs['speed_kmh'],speed),horizon_s=2.3)
        self.dynamic_debug['latest_admission_prediction']=risk
        timing['timed_ms']=(time.perf_counter()-mark)*1000;timing['total_ms']=(time.perf_counter()-started)*1000
        if reason or not risk['ready'] or (time.time_ns()-value['source_stamp_ns'])/1e9>.5:
            self.dynamic_debug['proposal_rejected']=reason or (risk['reason'] if not risk['ready'] else 'D_ADMISSION_SOURCE_AGE_MARGIN')
            self._clear_proposed_selection();return False
        self.dynamic=d;self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1;self.dynamic_pose=pose;self._last_pose=pose
        self._original=self.pair.original;setattr(self,self.pair.side+'_lane',self.pair.adjacent)
        self.dynamic_debug.update(decision='ENTRY',geometry_prepared_off_callback=True,latest_observations_revalidated=True,plan={k:v for k,v in d.items() if k not in ('points','entry','hold','back')},nominal_completion_is_not_live_permission=True)
        return True
    def _map_identity(self):
        return (self.network,getattr(self.network,'boundaries',None),getattr(self.network,'road_corridor',None),self.config,self.body)
    def _try_entry(self,reference,groups,pose,lead,value,kwargs):
        if not self.lateral_admission_allowed:
            self.dynamic_debug['lateral_admission_deferred_for_safe_follow']=True
            return False
        self._refresh_normal_bank(kwargs)
        pending=self._proposal_future
        if pending is not None and pending.done():
            proposal=pending.result();proposal['owner_map_context']=self._proposal_map_context;self._proposal_future=None
            self.dynamic_debug['prepared_geometry_checks']=proposal['checks']
            if self._admit_proposal(proposal,reference,groups,pose,lead,value,kwargs):return True
            if self.state=='BLOCKED':return False
        if self._proposal_future is None and self._lane_snapshot is not None:
            records=tuple(dict(v) for v in self.normal_guard.occupancy.static.tracks)
            self._proposal_map_context=self._map_identity()
            # Node-bound asynchronous search helpers are not geometry inputs.
            arguments={k:v for k,v in kwargs.items() if k not in ('entry_search','return_search')}
            self._proposal_future=self._proposal_executor.submit(prepare_current,self._lane_snapshot,self.config,records,reference,groups,pose,lead,value,arguments,self._proposal_generation)
        self.dynamic_debug['geometry_preparation_pending']=self._proposal_future is not None
        return False
    def plan(self,reference,groups,pose,*,route_waypoints=(),**kwargs):
        try:return self._plan(reference,groups,pose,route_waypoints=route_waypoints,**kwargs)
        except Exception:
            import traceback
            traceback.print_exc()
            raise
    def _plan(self,reference,groups,pose,*,route_waypoints=(),**kwargs):
        if self.state=='BLOCKED':return self._result()
        if getattr(self,'input_admitter',None) is not None:self.input_admitter()
        coverage=self.coverage_provider()
        if self.dynamic is not None and coverage.get('auxiliary_ready') is False:
            self.dynamic_debug=dict(getattr(self,'dynamic_debug',{}))
            self.dynamic_debug['auxiliary_failure']={k:v for k,v in coverage.items() if k not in ('rear_sensors','auxiliary_groups')}|dict(now_ns=time.time_ns(),sensor_sources=[(row['name'],row['ray'].source_ns) for row in coverage.get('rear_sensors',[])])
            return self.stop('D_AUXILIARY_REAR_SENSOR_STALE')
        groups=tuple(groups)+tuple(coverage.get('auxiliary_groups',[]))
        rear_sources=[dict(name=row['name'],source_ns=row['ray'].source_ns,sha256=row.get('ray_content_sha256')) for row in coverage.get('rear_sensors',[])]
        value=self.traffic_provider();now=time.time_ns();source=value.get('source_stamp_ns');age=(now-source)/1e9 if type(source)is int else math.inf
        valid=bool(value.get('ready') and -.05<=age<=.6 and kwargs.get('observation_valid',False))
        if not valid:
            self.dynamic_debug=dict(decision='WAIT_TRACKING_SOURCE',source_age_s=age if math.isfinite(age) else None)
            path=self.dynamic['points'] if self.dynamic is not None else reference
            result=self._result(path,speed_cap=0.);result['active']=self.dynamic is not None;return result
        if self.dynamic is not None:self.dynamic_debug['admitted_rear_sources']=rear_sources
        # A missing/expired moving track is uncertainty, never a static target.
        traffic_risk=any(t.get('motion') in ('UNKNOWN','MOVING') for t in value.get('tracks',[]))
        memory=getattr(getattr(self,'normal_guard',None),'occupancy',None)
        retained_mover=any(v.get('motion') in ('UNKNOWN','MOVING') for v in getattr(memory,'vehicles',{}).values())
        if self.dynamic is None and self.state!='NORMAL':
            if not (traffic_risk or retained_mover):
                return super().plan(reference,groups,pose,route_waypoints=route_waypoints,**kwargs)
            if self.state!='DETECT':return self.stop('D_TRAFFIC_DURING_COMMITTED_STATIC_MANEUVER')
            # Cancel only uncommitted static search; retain the normal guard,
            # its exact static history and last observed vehicle solids.
            completed=self.dynamic_completed;LaneCandidatePlanner.reset(self);self.dynamic_completed=completed
        lead=self._lead(value,reference,pose)
        if self.dynamic is None and not kwargs.get('motion_authorized',False):
            self._refresh_normal_bank(kwargs)
            return self._follow(reference,groups,pose,kwargs,lead,'D_DISARMED_NO_LATERAL_PREPARATION')
        if self.dynamic is None:
            if lead is None:
                if traffic_risk or retained_mover:
                    ahead=self._forward_uncertainty(value,memory,pose)
                    return self._follow(reference,groups,pose,kwargs,reason='D_MOVING_OR_UNKNOWN_OBSERVATION_UNCERTAIN' if ahead else 'D_TRAFFIC_BEHIND_KEEP_NATIVE',native_clear=not ahead)
                return super().plan(reference,groups,pose,route_waypoints=route_waypoints,**kwargs)
            self.dynamic_debug=dict(lead_id=lead['track']['id'],gap_m=lead['gap_m'],lead_speed_kmh=lead['forward_speed_mps']*3.6,relative_speed_mps=kwargs['speed_kmh']/3.6-lead['forward_speed_mps'])
            if not kwargs.get('motion_authorized',False):return self._follow(reference,groups,pose,kwargs,lead,'D_DRIVE_DISABLED')
            if lead['track']['frames']<25 or lead['track'].get('motion_sigma_mps',1.)>.3:
                return self._follow(reference,groups,pose,kwargs,lead,'D_VELOCITY_WARMUP')
            if not self._try_entry(reference,groups,pose,lead,value,kwargs):
                if self.state=='BLOCKED':return self._result()
                return self._follow(reference,groups,pose,kwargs,lead,'D_PASSING_UNPROVEN')
            return self._result(self.dynamic['points'],speed_cap=self.config.avoid_speed_kmh) # Admission already checked every live gate at this pose.
        d=self.dynamic
        if value['tracker_session_id']!=d['tracker_session']:return self.stop('D_TRACKER_SESSION_CHANGED')
        target=next((t for t in value['tracks'] if t['id']==d['lead_id'] and t['usable']),None)
        if target is None or target['missed']:return self.stop('D_LEAD_TRACK_UNAVAILABLE')
        raw=tuple(self._inflate(p,self.config.observation_padding_m) for p in obstacle_polygons(groups));self._detected_count=len(raw);self._last_pose=pose;self._min_clearance=math.inf
        reason=self._body_safety(self.dynamic_pose or pose,pose,raw);self.dynamic_pose=pose
        if reason:return self.stop('D_MEASURED_'+reason)
        station=self.pair.original_index.project(pose[:2]);self._last_progress=station
        if self.state=='AVOID_ENTRY' and station>=d['entry_end_s']-.4:self.state='AVOID_HOLD'
        if self.state=='AVOID_HOLD' and station>=d['return_start_s']-.5:
            npc=self.pair.original_index.project(target['center_xy']);gap=station-self.body.rear_overhang_m-npc-target['vehicle_length_model_m']/2
            self.dynamic_debug['actual_estimated_front_gap_m']=gap
            if gap<d['required_return_gap_m']:return self.stop('D_RETURN_FRONT_GAP_UNPROVEN')
            self.state='RETURN';self.return_commits+=1
        path=d['points'];index=PolylineIndex(path);progress=index.project(pose[:2]);row=sample(path,max(0.,min(index.lengths[-1],progress)))
        if math.dist(pose[:2],row[0][:2])>self.config.tracking_limit_m:return self.stop('D_TRACKING_ERROR')
        window=control_local_path(path,pose,max(20.,kwargs['speed_kmh']/3.6*3.))
        retained=self.normal_guard.plan(window,groups,pose,**kwargs)
        if retained['blocked']:return self.stop('D_RETAINED_'+retained['reason'])
        reason=self._path_safety(window,raw,kwargs['speed_kmh'],native_reference=True)
        if not reason:reason=self._rollout(path,pose,kwargs['speed_kmh'],kwargs['actual_curvature'],raw)
        risk=assess(path,pose,max(kwargs['speed_kmh'],self.config.avoid_speed_kmh),[t for t in value['tracks'] if t.get('lane') is not None and t.get('heading_rad') is not None],self.body,now_ns=now,source_ns=value['source_stamp_ns'],minimum_speed_kmh=min(kwargs['speed_kmh'],self.config.avoid_speed_kmh));self.dynamic_debug['live_prediction']=risk
        if reason or not risk['ready']:return self.stop('D_LIVE_'+(reason or risk['reason']))
        if self.state=='RETURN' and station>=d['return_end_s']:
            original=sample(self.pair.original,station);lateral=math.dist(pose[:2],original[0][:2]);heading=abs(heading_error(pose[2],math.atan2(original[1][1],original[1][0])))
            if lateral<=.15 and heading<=math.radians(8):
                self.dynamic=None;self.dynamic_completed+=1;self.state='NORMAL';self.selected=None;self.pair=None;self._road_area=None;self.normal_guard.previous_pose=pose
                self.dynamic_debug.update(decision='COMPLETED',actual_return_lateral_m=lateral)
                return self._result(reference)
        self.dynamic_debug['decision']=self.state
        cap=retained.get('speed_cap_kmh');cap=self.config.avoid_speed_kmh if cap is None else min(cap,self.config.avoid_speed_kmh)
        return self._result(path,speed_cap=cap)
