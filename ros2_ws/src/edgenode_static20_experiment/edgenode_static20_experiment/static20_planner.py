"""B adapter: A normal safety, original static gates, waypoint-proven native join."""
import math,time
from copy import copy
from dataclasses import replace
from shapely.geometry import Polygon,LineString
from shapely.strtree import STRtree
from edgenode_planning.lane_candidate_planner import LaneCandidatePlanner
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from edgenode_planning.lane_geometry import PolylineIndex
from edgenode_planning.lattice_path import control_local_path,poses
from edgenode_planning.avoidance_geometry import obstacle_polygons,swept_body,inflate
from edgenode_planning.maneuver_selection import Strategy
from edgenode_planning.search_budget import SearchBudgetExceeded
from edgenode_normal_experiment.normal_safety import NormalSafetyPlanner
from .natural_join import waypoint_join_proof
from .provenance import implementation_sha256

class Static20Planner(LaneCandidatePlanner):
    def __init__(self,network,*,config,weights,body,route_provider):
        self.normal_guard=None;self.requested_avoidance_kmh=config.avoid_speed_kmh
        super().__init__(network,config=config,weights=weights);self.body=body
        self.normal_guard=NormalSafetyPlanner(network,config=config,weights=weights,body=body,route_provider=route_provider)
    def __getstate__(self):
        state=super().__getstate__();state['normal_guard']=None
        return state # Search workers need static proofs, never the node-bound guard.
    def reset(self):
        if hasattr(self,'config') and hasattr(self,'requested_avoidance_kmh'):self.config=replace(self.config,avoid_speed_kmh=self.requested_avoidance_kmh)
        super().reset()
        if getattr(self,'normal_guard',None) is not None:self.normal_guard.reset()
    def _result(self,*args,**kwargs):
        result=super()._result(*args,**kwargs)
        if result['blocked']:result['speed_cap_kmh']=0.
        result.update(phase_b_implementation_sha256=implementation_sha256(),phase_b_mode='STATIC_WITH_PHASE_A_NORMAL_GUARD',requested_avoidance_kmh=self.requested_avoidance_kmh,effective_avoidance_kmh=self.config.avoid_speed_kmh)
        return result
    def _refresh_normal_bank(self,kwargs):
        current=kwargs.get('current_link_id');route=tuple(kwargs.get('route_link_ids',()));occurrence=kwargs.get('route_link_index')
        if occurrence is not None and 0<=occurrence<len(route) and route[occurrence]==current:native_route=route[occurrence:]
        elif route.count(current)==1:native_route=route[route.index(current):]
        else:native_route=(current,)
        if self._lane_bank is not None and current is not None:self._lane_snapshot=self._lane_bank.refresh(current,native_route)
    def _preparation_conflict(self,reference,groups):
        # Start the original two-source-frame confirmation as soon as a
        # CURRENT return conflicts. Requiring the normal guard's stricter
        # surface-shape association first delays braking/search by metres as
        # a distant vehicle is revealed in changing LiDAR fragments.
        # Confirmed static memory also remains a valid preparation trigger.
        # A vanished provisional return alone cannot trigger preparation.
        shapes=[t['shape'] for t in self.normal_guard.occupancy.tracks if t['confirmed']]
        shapes.extend(Polygon(p) for p in self.remembered)
        # Use the SAME observation padding as the normal guard and core.
        # Raw thin LiDAR fragments can fall outside the risk corridor while
        # their admitted uncertainty footprint already intersects it.
        shapes.extend(Polygon(inflate(p,self.config.observation_padding_m))
            for p in obstacle_polygons(groups))
        if not shapes:return False
        tree=STRtree(shapes);path_poses=poses(reference)
        for a,b in zip(path_poses,path_poses[1:]):
            body=Polygon(swept_body(self.body,a,b));axis=LineString((a[:2],b[:2]))
            for solid in tree.query(body.buffer(self.config.obstacle_margin_m)):
                if body.distance(solid)<=self.config.obstacle_margin_m:return True
            for solid in tree.query(axis.buffer(self.config.reference_clearance_m)):
                if axis.distance(solid)<self.config.reference_clearance_m:return True
        return False
    def plan(self,reference,groups,pose,*,route_waypoints=(),**kwargs):
        self._route_waypoints=tuple((identifier,tuple(point)) for identifier,point in route_waypoints)
        guard=self.normal_guard;normal=None;was_normal=self.state=='NORMAL'
        if self.state=='NORMAL' and guard is not None:
            self._refresh_normal_bank(kwargs)
            normal=guard.plan(reference,groups,pose,**kwargs)
            if normal['blocked']:
                self._failure_context=normal.get('failure_context',{});return self.stop('B_NORMAL_'+normal['reason'])
            conflict=normal.get('phase_a_normal_safety',{}).get('first_conflict_distance_m')
            if (conflict is None and not self.remembered) or not self._preparation_conflict(reference,groups):
                self._last_pose=pose;self._detected_count=normal['detected_object_count'];self._path_obstacle_count=normal['path_obstacle_count']
                result=self._result(reference);result.update(active=False,speed_cap_kmh=normal['speed_cap_kmh'],
                    phase_a_normal_guard_implementation_sha256=normal['phase_a_implementation_sha256'],
                    phase_a_normal_safety=normal['phase_a_normal_safety'],clearance_m=normal['clearance_m'])
                return result
        elif guard is not None and len(reference)>=2 and kwargs.get('observation_valid',True):
            index=PolylineIndex(reference);ego=max(0.,min(index.lengths[-1],index.project(pose[:2])))
            guard.occupancy.update(obstacle_polygons(groups),index,ego,self.body.rear_overhang_m,self.config.observation_padding_m,
                pose=pose,stamp=kwargs.get('observation_stamp_ns'),now=kwargs.get('now_sec',0.))
        result=super().plan(reference,groups,pose,route_waypoints=route_waypoints,**kwargs)
        if guard is not None:guard.previous_pose=pose
        if self.state=='NORMAL':
            self.config=replace(self.config,avoid_speed_kmh=self.requested_avoidance_kmh)
            result['effective_avoidance_kmh']=self.config.avoid_speed_kmh
            if normal is not None:
                cap=normal['speed_cap_kmh'];previous=result.get('speed_cap_kmh')
                if cap is not None:result['speed_cap_kmh']=cap if previous is None else min(cap,previous)
                result['phase_a_normal_guard_implementation_sha256']=normal['phase_a_implementation_sha256']
                result['phase_a_normal_safety']=normal['phase_a_normal_safety']
            # NORMAL may confirm an obstruction just outside the speed-based
            # detection trigger, then temporarily lose its current LiDAR face.
            # Preserve that TWO-SOURCE-FRAME proof until preparation begins;
            # deleting it forces a dangerously late second detection. Single
            # frame convex history remains excluded from NORMAL memory.
            if was_normal:
                self.remembered=tuple(t['solid'] for t in self._normal_tracks if t['confirmed'])
            else:
                # A completed fixed maneuver has already passed its captured
                # obstruction. A owns exact static history for the next NORMAL
                # segment; the old core confirmation episode must not leak.
                self.remembered=();self._normal_tracks=[]
                self._normal_detection_frames=self._confirmed_path_obstacles=0
        return result
    def _natural_waypoints_preserved(self,chosen,option,pose):
        proof=waypoint_join_proof(chosen.entry_path,option.continuation,self._global_path,pose,
            getattr(self,'_route_waypoints',()),self.config.tracking_limit_m)
        if proof is None:return False
        self._entry_context['natural_join_waypoint_proof']=proof
        return True

    def _search_entry(self,reference,pose,speed_kmh,actual_curvature,current_link_id,
                      route_link_ids,route_link_index,distance,body_cache=None,*,_native_only=False):
        self._complete_offsets={}
        self._complete_offset_proofs={}
        self._return_component_cache={}
        self._entry_proof_origin=(tuple(pose),actual_curvature)
        if (self.network.links.get(current_link_id,{}).get('link_type') not in ('6',6) or
                self.selected is not None and self.selected.mode=='road_offset'):
            if _native_only:return self._block('NO_VALID_ADJACENT_LANE')
            # Keep the existing offset implementation and its return proof.
            result=self._offset_speed_search(reference,pose,speed_kmh,actual_curvature,current_link_id,
                route_link_ids,route_link_index,distance,body_cache)
            return self._adopt_complete_offset(result)
        snapshot=self._lane_snapshot
        if snapshot is None:return self._block('LANE_BANK_PREPARING')
        self._entry_context=dict(current_link_id=current_link_id,pose=tuple(pose),
            candidate_source='PRECOMPUTED_MGEO_NATIVE_LANES',candidate_count=len(snapshot.options),
            precompute_ms=snapshot.elapsed_ms,candidate_checks=[])
        options=[o for o in snapshot.options if o.pair is not None and
                 current_link_id in {a for a,b in o.pair.ids}]
        options.sort(key=lambda o:(o.hops,o.continuation.merge_station_m is None,o.lane_id))
        generated=[]
        for option in options:
            check=dict(lane_id=option.lane_id,hops=option.hops,reason=option.exit_reason)
            self._entry_context['candidate_checks'].append(check)
            if option.exit_reason:continue
            try:guide=control_local_path(option.points,pose,self.config.prediction_distance_m)
            except ValueError:check['reason']='NATIVE_GUIDE_UNAVAILABLE';continue
            if self._normal_hazards(guide,self.remembered):
                check['reason']='NATIVE_LANE_BLOCKED';continue
            front=min(option.pair.original_index.project(p) for o in self._hazards for p in o)
            candidates=self._generate_entry(option.pair,pose,speed_kmh,actual_curvature,front,body_cache)
            generated.extend(candidates)
            self.candidates=tuple(generated)
            safe=[c for c in candidates if not c.rejection]
            if not safe:check['reason']='ENTRY_GEOMETRY_REJECTED';continue
            chosen=min(safe,key=lambda c:c.cost);original_chosen=chosen
            # Mission waypoints must remain reachable after the diversion.
            # Prove a complete native return now, rather than cruising past
            # intermediate route nodes in a parallel lane. Standalone callers
            # without waypoint constraints retain the existing continuation.
            reserve=(self.config.lookahead_min_m+max(speed_kmh,self.config.avoid_speed_kmh)/3.6*4.+
                     max(self.config.transition_lengths_m)+self._rollout_distance(speed_kmh)+10.)
            if ((getattr(self,'_route_waypoints',()) and not self._natural_waypoints_preserved(chosen,option,pose)) or
                    option.continuation.merge_station_m is None and
                    option.pair.length-chosen.entry_end_s<=reserve):
                for proposed in sorted(safe,key=lambda c:c.cost):
                    proof=self._complete_return_proof(proposed,option.pair,speed_kmh)
                    if proof is not None:
                        chosen=self._complete_offsets.pop(id(proposed));original_chosen=proposed
                        self._entry_context['native_return_proof']=proof
                        break
                else:
                    check['reason']='NATIVE_RETURN_OPENING_UNPROVEN'
                    generated=[replace(c,rejection='NATIVE_RETURN_OPENING_UNPROVEN')
                               if any(c is item for item in safe) else c for c in generated]
                    continue
            # Commit one validated lane; future bank updates cannot change it.
            self.selected=chosen;self.pair=option.pair
            self._strategy=Strategy.LANE_STATIC;self._road_area=getattr(self.pair,'road_area',None)
            if chosen.complete_return and self._road_area is None:
                self._road_area=getattr(self.network,'road_corridor',None)
            self._original=self.pair.original;self._last_progress=None;self._entry_wait_path=()
            setattr(self,self.pair.side+'_lane',self.pair.adjacent)
            self.candidates=tuple(chosen if c is original_chosen else c for c in generated)
            self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1
            self._entry_context.update(committed_lane_token=option.lane_id,
                lane_selection_reason='CLEAR_NATIVE_CENTERLINE_AND_VALIDATED_SMOOTH_ENTRY',
                natural_join_prepared=option.continuation.merge_station_m is not None)
            check['reason']='SELECTED'
            return self._result(self.selected.entry_path)
        self.candidates=tuple(generated)
        if _native_only:return self._block('NO_VALID_ADJACENT_LANE')
        # Ordinary roads can have one lane, or all legal adjacent lanes can
        # be occupied. Topology alone must not suppress a proved within-road
        # offset. Run it only after native candidates are exhausted; execution
        # still has one owner and every shared map/LiDAR/return gate applies.
        native_context=self._entry_context
        try:
            result=self._offset_speed_search(reference,pose,speed_kmh,actual_curvature,current_link_id,
                route_link_ids,route_link_index,distance,body_cache,lane_search_exhausted=True)
        finally:
            self._entry_context['native_lane_search']=native_context
        self.candidates=tuple(generated)+self.candidates
        return self._adopt_complete_offset(self._result(result['points'],speed_cap=result['speed_cap_kmh']))


    def _offset_speed_search(self,*args,**kwargs):
        requested=self.requested_avoidance_kmh
        speeds=tuple(dict.fromkeys([requested]+[v for v in (15.,10.) if v<requested]))
        deadline=getattr(self,'_search_deadline_sec',None)
        if deadline is None:deadline=time.perf_counter()+12.
        probes=[];last=None
        for index,speed in enumerate(speeds):
            if time.perf_counter()>=deadline:break
            remaining=deadline-time.perf_counter()
            # Prefer the request, but do not consume the final complete-proof
            # window on intermediate speed attempts after a curved approach.
            # This reallocates the SAME episode deadline; it never extends it.
            window=min(3.,max(0.,remaining-4.)) if index<len(speeds)-1 else remaining
            if window<=0.:
                probes.append(dict(speed_kmh=speed,elapsed_ms=0.,proven=False,
                    reason='BUDGET_RESERVED_FOR_LOWER_COMPLETE_PROOF',
                    classification='BOUNDED_SEARCH_NOT_A_PHYSICAL_IMPOSSIBILITY_PROOF'))
                continue
            probe=copy(self);probe.config=replace(self.config,avoid_speed_kmh=speed)
            probe._search_deadline_sec=min(deadline,time.perf_counter()+window)
            probe_args=list(args)
            # A lower-speed proof is CONDITIONAL on adoption at that speed.
            # EntrySearch waits for measured speed <= selected+.2, then runs
            # the original current-pose/speed/curvature validation. Proving it
            # at the old pre-braking speed needlessly rejects viable curves.
            probe_args[2]=min(probe_args[2],speed)
            began=time.perf_counter()
            try:
                if index>0 and kwargs.get('lane_search_exhausted'):
                    # A20 native return rejection does not reject the native
                    # lane at15/10. Couple speed and strategy before forcing
                    # all lower-speed trials into the road-offset fallback.
                    result=probe._search_entry(*probe_args,_native_only=True)
                    if result['blocked']:
                        tier_deadline=probe._search_deadline_sec
                        probe=copy(self);probe.config=replace(self.config,avoid_speed_kmh=speed)
                        probe._search_deadline_sec=tier_deadline
                        result=StaticObstaclePlanner._search_entry(probe,*probe_args,**kwargs)
                else:result=StaticObstaclePlanner._search_entry(probe,*probe_args,**kwargs)
            except SearchBudgetExceeded:result=probe._block('ENTRY_SEARCH_BUDGET_EXHAUSTED')
            probes.append(dict(speed_kmh=speed,elapsed_ms=(time.perf_counter()-began)*1000,
                proven=not result['blocked'],reason=result['reason'],selected_mode=getattr(probe.selected,'mode',None),classification='BOUNDED_SEARCH_NOT_A_PHYSICAL_IMPOSSIBILITY_PROOF'))
            last=(result,probe)
            if not result['blocked']:
                retained={k:getattr(self,k,None) for k in ('normal_guard','_lane_bank','_preview_executor','_preview_future')}
                self.__dict__.update(probe.__dict__)
                for key,value in retained.items():setattr(self,key,value)
                self._search_deadline_sec=deadline
                self._entry_context['offset_speed_probes']=probes
                self._entry_context['strategy_speed_probes']=probes
                self._entry_context['requested_avoidance_kmh']=requested
                self._entry_context['selected_avoidance_kmh']=speed
                return self._result(result['points'],speed_cap=result.get('speed_cap_kmh'))
            if result['reason'] not in ('NO_SAFE_ENTRY','OFFSET_RETURN_UNPROVEN','ENTRY_SEARCH_BUDGET_EXHAUSTED','NO_VALID_ADJACENT_LANE'):break
        if last is not None:
            self._entry_context=last[1]._entry_context
            self._entry_context['offset_speed_probes']=probes
            return self._block(last[0]['reason'])
        self._entry_context['offset_speed_probes']=probes
        return self._block('ENTRY_SEARCH_BUDGET_EXHAUSTED')
