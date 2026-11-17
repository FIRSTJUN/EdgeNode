"""Optional native-lane candidate selection; shared S4 execution safety gates."""
import math
import time
from concurrent.futures import ThreadPoolExecutor

from .lane_candidate_bank import LaneCandidateBank,reverse_crossing_reason
from .lane_geometry import PolylineIndex
from .lattice_path import control_local_path,center_suffix
from .maneuver_selection import Strategy
from .static_obstacle_planner import StaticObstaclePlanner


class LaneCandidatePlanner(StaticObstaclePlanner):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if not self.config.continuous_lane_follow:
            raise ValueError('lane_candidates requires continuous_lane_follow')
        if not self.config.integrated_strategy:
            raise ValueError('lane_candidates requires integrated_strategy for the junction offset fallback')
        self._lane_bank=LaneCandidateBank(self.network)
        self._lane_snapshot=None
        self._preview_executor=ThreadPoolExecutor(max_workers=1)
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
        if (self.network.links.get(current_link_id,{}).get('link_type') not in ('6',6) or
                self.selected is not None and self.selected.mode=='road_offset'):
            # Keep the existing offset implementation and its return proof.
            return super()._search_entry(reference,pose,speed_kmh,actual_curvature,current_link_id,
                route_link_ids,route_link_index,distance,body_cache)
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
            # Commit one validated lane; future bank updates cannot change it.
            self.selected=min(safe,key=lambda c:c.cost);self.pair=option.pair
            self._strategy=Strategy.LANE_STATIC;self._road_area=getattr(self.pair,'road_area',None)
            self._original=self.pair.original;self._last_progress=None;self._entry_wait_path=()
            setattr(self,self.pair.side+'_lane',self.pair.adjacent)
            self.candidates=tuple(generated);self.state='AVOID_ENTRY';self.reason='';self.entry_commits+=1
            self._entry_context.update(committed_lane_token=option.lane_id,
                lane_selection_reason='CLEAR_NATIVE_CENTERLINE_AND_VALIDATED_SMOOTH_ENTRY',
                natural_join_prepared=option.continuation.merge_station_m is not None)
            check['reason']='SELECTED'
            return self._result(self.selected.entry_path)
        self.candidates=tuple(generated)
        return self._block('NO_SAFE_NATIVE_LANE_CANDIDATE')

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
                self.network,self.config,self.body,index,at,self.remembered)
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
