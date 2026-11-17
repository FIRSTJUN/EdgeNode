"""Native lane continuation and exact-node natural joins; no invented edges."""
from dataclasses import dataclass
import math

from .lane_geometry import arc


@dataclass(frozen=True)
class LaneContinuation:
    points: tuple
    link_ids: tuple
    merge_station_m: float | None
    merge_link_id: str
    reason: str


def continuation(network,pair,mission_ids,route_index=None):
    points=[tuple(p) for p in pair.adjacent]
    ids=[b for a,b in pair.ids]
    base_points=tuple(points);base_ids=tuple(ids)
    last_original=pair.ids[-1][0]
    occurrences=[i for i,key in enumerate(mission_ids) if key==last_original
                 and (route_index is None or i>=route_index)]
    if len(occurrences)!=1:
        return LaneContinuation(tuple(points),tuple(ids),None,'','AMBIGUOUS_MISSION_JOIN')
    last=network.links[ids[-1]];merge=None;merge_id=''
    for key in mission_ids[occurrences[0]+1:]:
        target=network.links.get(key)
        if target is None:break
        if target['from_node_idx']==last['to_node_idx']:
            merge=arc(points)[-1];merge_id=key
            # The saved suffix itself is the continuation after this exact
            # directed graph join. Never use geometric proximity as an edge.
            for future in mission_ids[mission_ids.index(key,occurrences[0]+1):]:
                link=network.links[future]
                if (last['to_node_idx']!=link['from_node_idx'] or
                        math.dist(points[-1][:2],link['points'][0][:2])>1e-7):
                    return LaneContinuation(tuple(points),tuple(ids),None,'','DISCONTINUOUS_MISSION_JOIN')
                points.extend(tuple(p) for p in link['points'][1:]);ids.append(future);last=link
            return LaneContinuation(tuple(points),tuple(ids),merge,merge_id,'NATURAL_MISSION_JOIN')
        adjacent=target.get(pair.side+'_lane_change_dst_link_idx')
        candidates=[adjacent] if adjacent in network.valid else [k for k in network.valid
            if network.links[k]['from_node_idx']==last['to_node_idx']
            and network.links[k]['to_node_idx']==target['to_node_idx']]
        if len(candidates)!=1:break
        link=network.links[candidates[0]]
        if (link['from_node_idx']!=last['to_node_idx'] or
                math.dist(points[-1][:2],link['points'][0][:2])>1e-7):break
        points.extend(tuple(p) for p in link['points'][1:]);ids.append(candidates[0]);last=link
    # Extensions are used only with a complete natural-join witness. Otherwise
    # preserve the supplied pair's known return-permission support and schedule
    # a maneuver before that support ends.
    return LaneContinuation(base_points,base_ids,None,'','RETURN_REQUIRED_BEFORE_LANE_END')
