"""Precompute the actual directed native lanes, independently of execution."""
from collections import OrderedDict,deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import time

from .lane_geometry import LanePair,PolylineIndex,arc,cross,sample
from .lane_continuation import continuation


@dataclass(frozen=True)
class LaneOption:
    lane_id: str
    points: tuple
    route_ids: tuple
    crossings: tuple
    pair: object = None
    continuation: object = None
    exit_reason: str = ''

    @property
    def hops(self):return len(self.crossings[0])-1 if self.crossings else 0


@dataclass(frozen=True)
class LaneBankSnapshot:
    current_link: str
    mission_ids: tuple
    options: tuple
    elapsed_ms: float
    reason: str = ''


def concatenate(network,ids):
    points=[];previous=None;ends=[]
    for key in ids:
        link=network.links[key]
        if previous is not None and (previous['to_node_idx']!=link['from_node_idx'] or
                math.dist(points[-1][:2],link['points'][0][:2])>1e-7):
            raise ValueError('discontinuous native lane')
        points.extend(tuple(p) for p in link['points'][bool(points):]);previous=link
        ends.append(arc(points)[-1])
    return tuple(points),tuple(ends)


def combined_pair(network,edges):
    sections=[]
    for root,target in edges[0].ids:
        chain=[root,target]
        for edge in edges[1:]:
            targets=dict(edge.ids)
            if chain[-1] not in targets:break
            chain.append(targets[chain[-1]])
        if len(chain)!=len(edges)+1:break
        sections.append(tuple(chain))
    if not sections:return None
    original,ends=concatenate(network,[s[0] for s in sections])
    adjacent,_=concatenate(network,[s[-1] for s in sections])
    p,tangent=sample(original,min(1.,arc(original)[-1]/2))
    target=sample(adjacent,max(0.,min(arc(adjacent)[-1],PolylineIndex(adjacent).project(p))))
    if target is None or sum(a*b for a,b in zip(tangent,target[1]))<=0:return None
    delta=cross(tangent,(target[0][0]-p[0],target[0][1]-p[1]))
    if abs(delta)<=1e-7:return None
    pair=LanePair(original,adjacent,[(s[0],s[-1]) for s in sections],
                  'left' if delta>0 else 'right',ends)
    pair.bank_crossings=tuple(sections)
    return pair


def reverse_crossing_reason(network,sections):
    for chain in sections:
        for source,target in zip(chain,chain[1:]):
            side='right' if network.links[source].get('left_lane_change_dst_link_idx')==target else 'left'
            if network.neighbor(target,side)!=source:
                return network.permission_reason(target,side) or 'RETURN_LINK_MISMATCH'
    return ''


def build_lane_bank(network,current,mission_ids):
    started=time.perf_counter();mission_ids=tuple(mission_ids)
    if not mission_ids or mission_ids[0]!=current or current not in network.valid:
        return LaneBankSnapshot(current,mission_ids,(),0.,'INVALID_NATIVE_ROUTE')
    points,_=concatenate(network,mission_ids)
    options=[LaneOption(current,points,mission_ids,())]
    if network.links[current].get('link_type') not in ('6',6):
        return LaneBankSnapshot(current,mission_ids,tuple(options),(time.perf_counter()-started)*1000.,'AMBIGUOUS_LANES_USE_OFFSET')
    queue=deque([(current,mission_ids,())]);visited={current}
    while queue:
        lane,lane_route,edges=queue.popleft()
        for side in ('left','right'):
            neighbor=network.neighbor(lane,side)
            if neighbor is None or neighbor in visited:continue
            visited.add(neighbor)
            if network.links[neighbor].get('link_type') not in ('6',6):continue
            edge=network.pair(lane,side,list(lane_route),route_link_index=0)
            if edge is None:continue
            combined=combined_pair(network,edges+(edge,))
            if combined is None:continue
            following=continuation(network,combined,mission_ids,0)
            exit_reason='' if following.merge_station_m is not None else reverse_crossing_reason(network,combined.bank_crossings)
            combined.bank_continuation=following
            combined.bank_lane_id=neighbor
            route=tuple(b for a,b in edge.ids)
            options.append(LaneOption(neighbor,tuple(combined.adjacent),route,
                combined.bank_crossings,combined,following,exit_reason))
            queue.append((neighbor,route,edges+(edge,)))
    return LaneBankSnapshot(current,mission_ids,tuple(options),(time.perf_counter()-started)*1000.)


class LaneCandidateBank:
    """Map-only preparation; no obstacle oracle, publisher, or control state."""
    def __init__(self,network,executor=None):
        self.network=network
        self.executor=executor if executor is not None else ThreadPoolExecutor(max_workers=1)
        self.future=None;self.pending_key=None;self.cache=OrderedDict();self.error=''

    def refresh(self,current,mission_ids):
        key=(current,tuple(mission_ids))
        if self.future is not None and self.future.done():
            try:
                result=self.future.result();self.cache[self.pending_key]=result
                self.cache.move_to_end(self.pending_key)
                while len(self.cache)>4:self.cache.popitem(last=False)
                self.error=''
            except (ValueError,KeyError,TypeError) as error:self.error=str(error)
            self.future=self.pending_key=None
        if key in self.cache:
            self.cache.move_to_end(key);return self.cache[key]
        if self.future is None:
            self.pending_key=key;self.future=self.executor.submit(build_lane_bank,self.network,current,tuple(mission_ids))
        return None

    def close(self):self.executor.shutdown(wait=False,cancel_futures=True)
