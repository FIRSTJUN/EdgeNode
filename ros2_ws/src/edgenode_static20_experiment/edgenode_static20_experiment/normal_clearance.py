"""B-only ordinary reference centering inside the unchanged measured road.

No obstacle or actor input is used. Endpoints and endpoint derivatives remain
those of the saved native link. A candidate replaces an optional failed normal
centering only after its entire swept body plus tracking margin is proved.
Unproved links retain their original reference and every live safety gate.
"""
import math

from edgenode_planning.avoidance_geometry import inflate, swept_body
from edgenode_planning.lane_geometry import arc, curvature_at, sample
from edgenode_planning.lattice_path import curvatures, poses


def smooth_window(station, length, ramp=18.):
    def smooth(value):
        value=max(0.,min(1.,value))
        return value**3*(10.-15.*value+6.*value*value)
    return smooth(station/ramp)*smooth((length-station)/ramp)


def candidate_path(points, offset, *, spacing=.25):
    import numpy as np
    from scipy.interpolate import make_interp_spline
    lengths=arc(points);total=lengths[-1]
    if total<35.:raise ValueError('insufficient native support')
    count=max(6,math.ceil(total/5.))
    stations=np.linspace(0.,total,count+1);targets=[]
    for station in stations:
        point,tangent=sample(points,float(station),lengths)
        shift=offset*smooth_window(float(station),total)
        targets.append([point[0]-tangent[1]*shift,point[1]+tangent[0]*shift])
    conditions=[]
    for station in (0.,total):
        _,tangent=sample(points,station,lengths);k=curvature_at(points,station)
        conditions.append([(1,list(tangent)),(2,[-k*tangent[1],k*tangent[0]])])
    spline=make_interp_spline(stations,np.array(targets),k=5,bc_type=tuple(conditions))
    dense=np.linspace(0.,total,math.ceil(total/spacing)+1)
    path=tuple((float(x),float(y),float(sample(points,float(s),lengths)[0][2]))
               for s,(x,y) in zip(dense,spline(dense)))
    # Only floating point spline roundoff is snapped, never a vehicle state.
    return (tuple(points[0]),)+path[1:-1]+(tuple(points[-1]),)


def clearance_reason(path, boundaries, road, body, margin):
    previous=None
    for pose in poses(path):
        polygon=inflate(swept_body(body,previous or pose,pose),margin)
        previous=pose
        reason=boundaries.swept_reason(polygon)
        if reason:return reason
        if not road.covers(polygon):return 'MEASURED_ROAD_CONTAINMENT_UNPROVEN'
    return ''


def centered_reference(points, boundaries, road, body, margin=.15,
                       steering_curvature_limit=math.tan(.7*.610865)/2.84):
    """Finite small-offset search with full proof, including unchanged ends."""
    if road is None:return None,'MEASURED_ROAD_UNAVAILABLE'
    if not clearance_reason(points,boundaries,road,body,margin):
        return None,'ORIGINAL_ALREADY_HAS_TRACKING_CLEARANCE'
    original_peak=max(map(abs,curvatures(points)),default=math.inf)
    # The normal speed planner still limits lateral acceleration from the
    # resulting curvature. Never demand a sharper reference than the original
    # curve (above the ordinary .08 floor), nor exceed the measured steering
    # geometry. A .08 optimization preference is not a vehicle steering limit.
    curvature_bound=min(steering_curvature_limit,max(.08,original_peak))
    rejected={}
    for offset in (0.,-.1,.1,-.2,.2,-.3,.3,-.4,.4,-.5,.5,-.6,.6):
        try:
            candidate=candidate_path(points,offset)
            peak=max(map(abs,curvatures(candidate)),default=math.inf)
            reason='NORMAL_CENTER_CURVATURE' if peak>curvature_bound else clearance_reason(
                candidate,boundaries,road,body,margin)
        except ValueError as error:
            reason=str(error)
        if reason:
            rejected[reason]=rejected.get(reason,0)+1
            continue
        lengths=arc(points);candidate_lengths=arc(candidate)
        # A bound on the actual reference displacement, including smoothing.
        from edgenode_planning.lane_geometry import PolylineIndex
        index=PolylineIndex(points)
        shift=max(math.dist(p[:2],sample(points,max(0.,min(lengths[-1],index.project(p[:2]))),lengths)[0][:2]) for p in candidate)
        if shift>.75:
            rejected['NORMAL_CENTER_SHIFT']=rejected.get('NORMAL_CENTER_SHIFT',0)+1
            continue
        return candidate,dict(source=road.source,algorithm='B_MAP_ONLY_C2_NORMAL_CENTER',
            requested_offset_m=offset,max_center_shift_m=shift,tracking_margin_m=margin,
            max_curvature_inv_m=peak,original_max_curvature_inv_m=original_peak,
            curvature_bound_inv_m=curvature_bound,endpoint_preserved=True,endpoint_derivatives_preserved=True,
            full_link_swept_body_proved=True,unchanged_caps_are_not_reclassified=False,
            original_length_m=lengths[-1],corrected_length_m=candidate_lengths[-1],
            actor_data_used=False,rejections_before_selection=rejected)
    return None,rejected


def extend_normal_cache(network,body,margin=.15,*,steering_curvature_limit=math.tan(.7*.610865)/2.84):
    """Reuse the existing route correction seam without changing v61 or A."""
    cache=getattr(network,'_normal_center_cache',{})
    audit=[]
    for key,cached in list(cache.items()):
        identifier,model,tracking=key
        if model!=body or tracking!=margin:continue
        if cached is not None and cached[0] is not None:continue
        link=network.links[identifier]
        if link.get('link_type') not in ('6',6):continue
        path,certificate=centered_reference(link['points'],network.boundaries,
            network.road_corridor,body,margin,steering_curvature_limit)
        if path is not None:
            certificate=dict(certificate,link=identifier)
            cache[key]=(path,certificate)
        audit.append(dict(link=identifier,selected=path is not None,result=certificate))
    return audit
