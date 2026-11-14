"""Resume an ordered mission without inventing connectors to passed waypoints."""
import math


def mission_suffix(planner, saved_start, waypoints, current, occurrence=-1):
    full=planner.plan_route(saved_start,waypoints)
    if full is None:raise ValueError('SAVED_ROUTE_UNAVAILABLE')
    indices=[i for i,k in enumerate(full['link_ids']) if k==current]
    if not indices:raise ValueError('CURRENT_LINK_OUTSIDE_SAVED_ROUTE')
    if occurrence<0:
        if len(indices)!=1:raise ValueError('AMBIGUOUS_ROUTE_OCCURRENCE')
        at=indices[0]
    elif occurrence<len(indices):at=indices[occurrence]
    else:raise ValueError('INVALID_ROUTE_OCCURRENCE')
    # Keep intentional future revisits. Only the passed prefix is removed.
    ids=full['link_ids'][at:];points=[]
    for link_id in ids:
        for point in planner.links_by_id[link_id]['points']:
            if not points or points[-1]!=point:points.append(point.copy())
    cursor=0;remaining=[]
    for waypoint in waypoints:
        cursor=full['node_ids'].index(waypoint,cursor)
        if cursor>at:remaining.append(waypoint)
    result=dict(full,start_link_id=current,start_node_id=planner.links_by_id[current]['to_node_idx'],
        node_ids=full['node_ids'][at:],link_ids=ids,points=points,
        total_cost_m=sum(planner._link_costs[k] for k in ids),
        remaining_waypoints=remaining,mission_link_index=at)
    return result


def link_index_at_progress(route, planner, current, progress_m):
    """Disambiguate a revisited link using the protected global path progress."""
    station=0.;candidates=[]
    for index,link_id in enumerate(route['link_ids']):
        points=planner.links_by_id[link_id]['points']
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
        if link_id==current and station-4.<=progress_m<=station+length+4.:candidates.append(index)
        station+=length
    return candidates[0] if len(candidates)==1 else None
