"""Speed bound from known static 2D occluders along a curved control corridor.

Unoccluded rays are not a certificate of free space. This optional bound only
adds caution where existing LiDAR solids actually block the view; it never
replaces sensor freshness, obstacle/body gates or DistanceSafety evidence.
"""
import math
from .avoidance_geometry import obstacle_polygons,inflate,intersects,polygon_bounds
from .lane_geometry import arc,sample,PolylineIndex


def occlusion_limit(reference,groups,pose,body,*,deceleration_mps2,source_age_s,
                    sensor_forward_m,tracking_margin_m=.15,response_time_s=1.1,spacing_m=2.,native_geometry=False):
    values=(*pose,deceleration_mps2,source_age_s,sensor_forward_m,tracking_margin_m,response_time_s,spacing_m)
    if (len(reference)<2 or not all(math.isfinite(v) for v in values) or
            min(deceleration_mps2,spacing_m,response_time_s)<=0 or min(source_age_s,tracking_margin_m)<0):
        raise ValueError('invalid occlusion model input')
    solids=obstacle_polygons(groups)
    if not solids:return dict(limited=False,speed_cap_kmh=None,reason='NO_OBSERVED_OCCLUDER')
    obstacles=[(inflate(solid,.05),polygon_bounds(inflate(solid,.05))) for solid in solids]
    if native_geometry:
        from shapely.geometry import LineString
        from shapely.ops import unary_union
        from shapely.prepared import prep
        from .native_obstacle_geometry import shape
        native=[shape(solid) for solid,_ in obstacles]
        observed_union=unary_union(native);prepared=prep(observed_union)
    lengths=arc(reference);station=max(0.,PolylineIndex(reference).project(pose[:2]))
    origin=(pose[0]+sensor_forward_m*math.cos(pose[2]),pose[1]+sensor_forward_m*math.sin(pose[2]))
    start=body.wheelbase_m+body.front_overhang_m+spacing_m
    for distance in (start+i*spacing_m for i in range(max(0,math.floor((lengths[-1]-station-start)/spacing_m)+1))):
        p,t=sample(reference,station+distance,lengths)
        footprint=inflate(body.polygon(p[0],p[1],math.atan2(t[1],t[0])),tracking_margin_m)
        for target in footprint:
            bounds=polygon_bounds((origin,target))
            ray=LineString((origin,target)) if native_geometry else None
            # An exact union answers the common clear-ray case in native
            # code. Scan individual identities only for the first blocked ray
            # to retain the original occluder evidence and stopping bound.
            if native_geometry and not prepared.intersects(ray) and ray.distance(observed_union)>1e-9:continue
            for i,(solid,(x0,x1,y0,y1)) in enumerate(obstacles):
                if x1<bounds[0] or bounds[1]<x0 or y1<bounds[2] or bounds[3]<y0:continue
                blocked=(ray.intersects(native[i]) or ray.distance(native[i])<=1e-9) if native_geometry else intersects((origin,target),solid)
                if not blocked:continue
                visible=max(0.,distance-spacing_m)
                # The ray target already includes the front overhang and turn
                # sweep. This distance is rear-axle travel to the visible body
                # pose, so subtracting the overhang again would double count it.
                available=max(0.,visible-1.)
                reaction=response_time_s+source_age_s
                v=math.sqrt((deceleration_mps2*reaction)**2+2*deceleration_mps2*available)-deceleration_mps2*reaction
                return dict(limited=True,speed_cap_kmh=3.6*v,reason='OBSERVED_CORRIDOR_OCCLUSION',
                    visible_distance_m=visible,available_stopping_m=available,
                    geometry_model='FULL_BODY_AT_FUTURE_REAR_AXLE_POSES',
                    response_time_s=reaction,source_age_s=source_age_s,
                    occluder=solid,origin_xy=origin,target_xy=target)
    return dict(limited=False,speed_cap_kmh=None,reason='NO_OBSERVED_CORRIDOR_OCCLUSION')
