"""Optional native-lane candidate selection; shared S4 execution safety gates."""
import math
import time
from copy import copy
from dataclasses import replace
from bisect import bisect_left
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

from .lane_candidate_bank import LaneCandidateBank,reverse_crossing_reason
from .lane_geometry import PolylineIndex,sample,curvature_at,arc
from .lattice_path import control_local_path,center_suffix,poses
from .maneuver_selection import Strategy
from .static_obstacle_planner import StaticObstaclePlanner


class LaneCandidatePlanner(StaticObstaclePlanner):
    complete_offset_search=True

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if not self.config.continuous_lane_follow:
            raise ValueError('lane_candidates requires continuous_lane_follow')
        if not self.config.integrated_strategy:
            raise ValueError('lane_candidates requires integrated_strategy for the junction offset fallback')
        self._lane_bank=LaneCandidateBank(self.network)
        self._lane_snapshot=None
        self._preview_executor=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'))
        self._preview_future=None;self._preview_key=None;self._preview_result=None;self._preview_next=0.

    def __getstate__(self):
        state=dict(self.__dict__);state['_lane_bank']=None
        state['_preview_executor']=None;state['_preview_future']=None
        return state

    def close(self):
        if self._lane_bank is not None:self._lane_bank.close()
        if self._preview_executor is not None:self._preview_executor.shutdown(wait=False,cancel_futures=True)

    def reset(self):
        super().reset()
        future=getattr(self,'_preview_future',None)
        if future is not None:future.cancel()
        self._preview_future=None;self._preview_key=None;self._preview_result=None;self._preview_next=0.

    def plan(self,reference,groups,pose,**kwargs):
        current=kwargs.get('current_link_id');route=tuple(kwargs.get('route_link_ids',()))
        occurrence=kwargs.get('route_link_index')
        if occurrence is not None and 0<=occurrence<len(route) and route[occurrence]==current:
            native_route=route[occurrence:]
        elif route.count(current)==1:native_route=route[route.index(current):]
        else:
            following=self._continuation or getattr(self.pair,'bank_continuation',None)
            ids=following.link_ids if following is not None else ()
            native_route=ids[ids.index(current):] if ids.count(current)==1 else (current,)
        if self._lane_bank is not None and current is not None:
            self._lane_snapshot=self._lane_bank.refresh(current,native_route)
        return super().plan(reference,groups,pose,**kwargs)

    def _result(self,*args,**kwargs):
        result=super()._result(*args,**kwargs)
        snapshot=getattr(self,'_lane_snapshot',None)
        result.update(static_avoidance_method='lane_candidates',
            lane_candidate_bank_ready=snapshot is not None,
            lane_candidate_count=len(snapshot.options) if snapshot else 0,
            lane_candidate_ids=[o.lane_id for o in snapshot.options] if snapshot else [],
            lane_candidate_prepare_ms=snapshot.elapsed_ms if snapshot else None,
            committed_lane_token=self._entry_context.get('committed_lane_token',''))
        result['selected_complete_offset']=bool(self.selected and self.selected.complete_offset)
        result['selected_complete_return']=bool(self.selected and self.selected.complete_return)
        if self.selected is not None and self.selected.mode=='lane':result['selected_strategy']='LANE_CANDIDATES'
        elif self.state=='NORMAL':result['selected_strategy']='NORMAL'
        return result

    def lane_candidates_view(self,pose):
        snapshot=self._lane_snapshot
        values=[]
        if snapshot is not None:
            for option in snapshot.options:
                try:points=control_local_path(option.points,pose,self.config.prediction_distance_m)
                except ValueError:points=()
                values.append(dict(lane_id=option.lane_id,hops=option.hops,points=points,
                    natural_join=option.continuation is not None and option.continuation.merge_station_m is not None,
                    exit_reason=option.exit_reason))
        return dict(source='MGEO_DIRECTED_NATIVE_LANES',ready=snapshot is not None,
            count=len(values),candidates=values,obstacle_truth_used=False,
            collision_authorization='LATEST_LIDAR_AND_SHARED_S4_SWEEP_ROLLOUT_REQUIRED')

    def _search_entry(self,reference,pose,speed_kmh,actual_curvature,current_link_id,
                      route_link_ids,route_link_index,distance,body_cache=None):
        self._complete_offsets={}
        if (self.network.links.get(current_link_id,{}).get('link_type') not in ('6',6) or
                self.selected is not None and self.selected.mode=='road_offset'):
            # Keep the existing offset implementation and its return proof.
            result=super()._search_entry(reference,pose,speed_kmh,actual_curvature,current_link_id,
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
            safe=[c for c in candidates if not c.rejection]
            if not safe:check['reason']='ENTRY_GEOMETRY_REJECTED';continue
            chosen=min(safe,key=lambda c:c.cost);original_chosen=chosen
            # Near a lane end, reverse permission alone is insufficient. Keep
            # a complete return through the actual opening before entering.
            # Long clear lanes and natural joins retain their cruise behavior.
            reserve=(self.config.lookahead_min_m+max(speed_kmh,self.config.avoid_speed_kmh)/3.6*4.+
                     max(self.config.transition_lengths_m)+self._rollout_distance(speed_kmh)+10.)
            if (option.continuation.merge_station_m is None and
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
        # Ordinary roads can have one lane, or all legal adjacent lanes can
        # be occupied. Topology alone must not suppress a proved within-road
        # offset. Run it only after native candidates are exhausted; execution
        # still has one owner and every shared map/LiDAR/return gate applies.
        native_context=self._entry_context
        result=super()._search_entry(reference,pose,speed_kmh,actual_curvature,current_link_id,
            route_link_ids,route_link_index,distance,body_cache,lane_search_exhausted=True)
        self._entry_context['native_lane_search']=native_context
        self.candidates=tuple(generated)+self.candidates
        return self._adopt_complete_offset(self._result(result['points'],speed_cap=result['speed_cap_kmh']))

    def _adopt_complete_offset(self,result):
        if not result['blocked'] and self.state=='AVOID_ENTRY' and self.selected.mode=='road_offset':
            previous=self.selected;self.selected=self._complete_offsets[id(previous)]
            self.candidates=tuple(self.selected if c is previous else c for c in self.candidates)
            self._complete_offsets={}
            self._maneuver_selection.path_id=id(self.selected.entry_path)
            self._entry_context['offset_execution']='FIXED_ENTRY_PASSAGE_RETURN_LATEST_INPUT_VALIDATION'
            return self._result(self.selected.entry_path)
        return result

    def _offset_return_proof(self,candidate,pair,speed):
        return self._complete_return_proof(candidate,pair,speed)

    def _complete_return_proof(self,candidate,pair,speed):
        """Retain the proved return instead of searching again after entry.

        Joining changes PP preview near the apex, so recheck the entire fixed
        out-and-back path with the production controller before admitting it.
        No actor position or guessed road polygon enters this proof.
        """
        probe=copy(self);probe.pair=pair;probe.selected=candidate
        probe._road_area=getattr(pair,'road_area',None);probe._validated_paths={}
        if candidate.mode=='lane' and probe._road_area is None:
            # An authorized lane crossing does not prove the road interior at
            # a short lane's return. Use the already enabled surveyed surface
            # union, including its exact missing caps/holes, before committing.
            # This adds evidence; it never creates a road from a lane width.
            probe._road_area=getattr(self.network,'road_corridor',None)
        probe._planning_tracking_reserve=self.config.tracking_limit_m
        probe._complete_return_proof_active=True
        probe._obstacle_bounds=dict(self._obstacle_bounds)
        probe._map_sweep_cache=type(self._map_sweep_cache)()
        probe._fixed_sweep_cache=type(self._fixed_sweep_cache)()
        entry=candidate.entry_path
        last=pair.original_index.project(entry[-1])
        rear=max(pair.original_index.project(p) for o in self._hazards for p in o)
        start=max(candidate.entry_end_s,rear+self.body.rear_overhang_m+self.config.return_margin_m)
        if candidate.mode=='lane':
            # Native entry candidates end in a controller tail, which can be
            # BEFORE the parked object's far end. Extend the known native hold
            # guide to the proposed return anchor before proving the composite.
            # The added guide still receives whole-body and full PP checks.
            end=min(pair.length-self.body.wheelbase_m-self.body.front_overhang_m-1.,max(last,start+8.))
            row=pair.at(end)
            if row is not None and end>last:
                at=pair.adjacent_index.project(entry[-1])
                entry=entry+center_suffix(pair.adjacent,at,row['adjacent_s'])[1:]
                last=pair.original_index.project(entry[-1])
        rejected=self._entry_context.setdefault('offset_complete_rejections',{})
        def reject(reason):rejected[reason]=rejected.get(reason,0)+1
        for advance in range(0,9,2):
            station=start+advance
            if station>last:break
            row=pair.at(station)
            if row is None:continue
            target=sample(pair.adjacent,row['adjacent_s'])
            pose=(*target[0][:2],math.atan2(target[1][1],target[1][0]))
            if not probe._return_clearance_ready(pose):reject('RETURN_CLEARANCE');continue
            reason=probe._generate_return(pose,speed,curvature_at(pair.adjacent,row['adjacent_s']),{})
            if reason:reject(reason);continue
            entry_index=PolylineIndex(entry)
            clip=entry_index.project(pose[:2])
            # Preserve the validated Hermite sample spacing. Subdividing an
            # already sampled curve inserts collinear half-edges and doubles
            # the three-point curvature at every retained original vertex.
            endpoint=sample(entry,clip)[0]
            prefix=entry[:bisect_left(entry_index.lengths,clip)]+(tuple(endpoint),)
            if math.dist(prefix[-1][:2],probe._return_path[0][:2])>1e-6:reject('OFFSET_RETURN_JOIN_MISMATCH');continue
            path=prefix+probe._return_path[1:]
            origin=poses(path)[0]
            # Each component already has a whole-body proof. This additional
            # proof includes the join and the controller's changed lookahead.
            reason=probe._path_safety(path,self.remembered,speed)
            if reason:reject('COMPLETE_PATH:'+reason);continue
            velocity=max(speed,self.config.avoid_speed_kmh)/3.6
            # The tail supports PP's target/front body after return completion;
            # it is not an instruction to drive past the validated path end.
            duration=max(self.config.rollout_seconds,
                (arc(path)[-1]-self._rollout_distance(speed))/velocity+self.config.rollout_seconds)
            reason=probe._rollout(path,origin,speed,curvature_at(path,0.),self.remembered,duration_s=duration)
            if reason:reject('COMPLETE_PP:'+reason);continue
            self._complete_offsets[id(candidate)]=replace(candidate,points=path,entry_path=path,
                return_path=probe._return_path,kinematic_points=path,
                return_start_s=station,return_end_s=probe.selected.return_end_s,
                complete_offset=candidate.mode=='road_offset',complete_return=True)
            return dict(source='LIVE_LIDAR_AND_VERIFIED_MAP',station_m=station,
                transition_length_m=probe.selected.transition_length_m,
                complete_path_length_m=arc(path)[-1],return_end_s=probe.selected.return_end_s,
                whole_maneuver_pp_proven=True)
        return None

    def _follow_complete_maneuver(self,normal_reference,pose,speed,curvature,progress,body_cache):
        path=self.selected.entry_path
        reason=self._validate_active(path,pose,speed,curvature,body_cache)
        if reason:return self._block(reason)
        if self.state!='RETURN' and progress>=self.selected.return_start_s:
            self.state='RETURN';self.reason=('PREVALIDATED_OFFSET_RETURN' if self.selected.complete_offset
                else 'PREVALIDATED_LANE_RETURN');self.confirmations=0
            self.return_commits+=1;self._return_path=path
        elif self.state=='AVOID_ENTRY' and self._acquired(pose,self.pair.adjacent,self.selected.entry_end_s,progress):
            self.state='AVOID_HOLD';self.reason='FIXED_OBSTACLE_PASSAGE';self.confirmations=0
        if self.state=='RETURN':
            station=PolylineIndex(self._global_path).project(pose[:2])
            if self._acquired(pose,self._global_path,self.selected.return_end_s,station):
                self.state='NORMAL';self.reason='';self.remembered=();self.selected=None;self.pair=None
                self.candidates=();self._hold_path=();self._return_path=();self._hazards=();self._road_area=None
                self._hold_started=None;self._last_progress=None;self._validated_paths.clear()
                return self._result(normal_reference)
        return self._result(path)

    def _hold(self,pose,speed,actual_curvature,body_cache):
        following=getattr(self.pair,'bank_continuation',None)
        if following is not None and self._continuation is None:
            self._continuation=following;self._continuation_index=PolylineIndex(following.points)
        return super()._hold(pose,speed,actual_curvature,body_cache)

    @staticmethod
    def _far_lane_scan(network,config,body,index,station,obstacles):
        started=time.perf_counter();end=min(index.lengths[-1],station+config.prediction_distance_m)
        if end<=station:return None,(time.perf_counter()-started)*1000.
        guide=center_suffix(index.points,station,end)
        probe=StaticObstaclePlanner(network,config=config);probe.body=body
        hazards=probe._normal_hazards(guide,obstacles)
        front=min((index.project(p) for solid in hazards for p in solid),default=None)
        return front,(time.perf_counter()-started)*1000.

    def _lane_obstacle_preview(self,pose,speed):
        # This is early maneuver preparation only. The inherited current body,
        # stopping/control-window sweep and measured PP rollout still consume
        # all latest solids every execution tick, including while this runs.
        if self.selected.mode!='lane':return super()._lane_obstacle_preview(pose,speed)
        index=self._continuation_index or self.pair.adjacent_index
        key=id(index.points);now=time.monotonic();at=max(0.,index.project(pose[:2]))
        if self._preview_future is not None and self._preview_future.done():
            result=self._preview_future.result()
            if self._preview_key==key:self._preview_result=(key,*result,now)
            self._preview_future=None
        if self._preview_future is None and now>=self._preview_next:
            self._preview_key=key
            self._preview_future=self._preview_executor.submit(self._far_lane_scan,
                None,self.config,self.body,index,at,self.remembered)
            self._preview_next=now+.5
        reserve=(max(speed,self.config.avoid_speed_kmh)/3.6*4.+
                 max(self.config.transition_lengths_m)+self._rollout_distance(speed))
        cached=self._preview_result
        self._entry_context.update(lane_preview_pending=self._preview_future is not None,
                                   lane_obstacle_replan_distance_m=reserve)
        if cached is None or cached[0]!=key:return None
        _,front,elapsed,finished=cached
        self._entry_context.update(lane_preview_elapsed_ms=elapsed,lane_preview_age_s=now-finished)
        if front is None:return None
        gap=front-at-self.body.wheelbase_m-self.body.front_overhang_m-self.config.obstacle_margin_m
        self._entry_context['lane_obstacle_gap_m']=gap
        return gap if gap<=reserve else None

    def _return_permission(self,start,end):
        sections=getattr(self.pair,'bank_crossings',None)
        if sections is None:return super()._return_permission(start,end)
        if start<0 or end>self.pair.length:return 'RETURN_OUTSIDE_NATIVE_SUPPORT'
        relevant=[];begin=0.
        for chain,finish in zip(sections,self.pair.section_ends):
            if finish>=start and begin<=end:relevant.append(chain)
            begin=finish
        return reverse_crossing_reason(self.network,relevant)
