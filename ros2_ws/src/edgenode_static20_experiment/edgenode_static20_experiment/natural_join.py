"""Prove required waypoint order on an existing exact directed native join."""
import math
from edgenode_planning.lane_geometry import PolylineIndex,sample,arc
from edgenode_planning.lattice_path import center_suffix

def waypoint_join_proof(entry,follow,global_reference,pose,waypoints,tracking_limit):
    if follow is None or follow.reason!='NATURAL_MISSION_JOIN' or follow.merge_station_m is None:return None
    if len(entry)<2 or len(global_reference)<2:return None
    join=sample(follow.points,follow.merge_station_m)
    if join is None:return None
    global_index=PolylineIndex(global_reference);progress=global_index.project(pose[:2]);merge=global_index.project(join[0][:2])
    if not progress<merge<=global_index.lengths[-1]:return None
    on_global=sample(global_reference,merge)
    if on_global is None or math.dist(on_global[0][:2],join[0][:2])>1e-6:return None
    native_index=PolylineIndex(follow.points);begin=native_index.project(entry[-1][:2])
    if not 0<=begin<=follow.merge_station_m:return None
    anchor=sample(follow.points,begin)
    if anchor is None or math.dist(anchor[0][:2],entry[-1][:2])>1e-6:return None
    witness=tuple(entry)+center_suffix(follow.points,begin,follow.merge_station_m)[1:]
    index=PolylineIndex(witness);tolerance=max(0.,1.-tracking_limit);checked=[];previous=-math.inf
    for identifier,point in waypoints:
        station=global_index.project(point[:2]);on_route=sample(global_reference,max(0.,min(global_index.lengths[-1],station)))
        if on_route is None or math.dist(on_route[0][:2],point[:2])>.05:return None
        if not progress+.01<station<=merge+.01:continue
        along=index.project(point[:2]);nearest=sample(witness,max(0.,min(index.lengths[-1],along)))
        if nearest is None or along<previous-.01:return None
        distance=math.dist(nearest[0][:2],point[:2])
        if distance>tolerance:return None
        previous=along;checked.append(dict(id=identifier,distance_m=distance,witness_station_m=along))
    return dict(source='EXACT_DIRECTED_NATIVE_JOIN_AND_ORDERED_MISSION_WAYPOINTS',merge_link_id=follow.merge_link_id,
        merge_global_station_m=merge,merge_native_station_m=follow.merge_station_m,checked_waypoints=checked,
        waypoint_distance_limit_m=1.,reserved_tracking_m=tracking_limit,path_distance_limit_m=tolerance,
        whole_lateral_entry_and_live_execution_gates_unchanged=True)
