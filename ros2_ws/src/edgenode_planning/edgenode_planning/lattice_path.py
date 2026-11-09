"""Fixed world-frame Hermite lattice to actual MGeo lane centerlines."""
from dataclasses import dataclass
from bisect import bisect_right
import math

from .lane_geometry import PolylineIndex, arc, curvature_at, sample
from .local_path_planner import LocalPathPlanner


def heading_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def poses(points):
    result = []
    for i, point in enumerate(points):
        a, b = points[max(0,i-1)], points[min(len(points)-1,i+1)]
        result.append((*point[:2], math.atan2(b[1]-a[1], b[0]-a[0])))
    return result


def curvatures(points):
    clean=[]
    for p in points:
        if not all(math.isfinite(v) for v in p):
            raise ValueError('nonfinite path')
        if not clean or math.dist(clean[-1][:2],p[:2])>1e-9:
            clean.append(p)
    points=clean
    result = []
    for a, b, c in zip(points, points[1:], points[2:]):
        denominator = math.dist(a[:2],b[:2])*math.dist(b[:2],c[:2])*math.dist(a[:2],c[:2])
        if denominator <= 1e-12:
            raise ValueError('degenerate path')
        u=(b[0]-a[0],b[1]-a[1]);v=(c[0]-b[0],c[1]-b[1])
        if u[0]*v[0]+u[1]*v[1]<0 and abs(u[0]*v[1]-u[1]*v[0])<=1e-12:
            raise ValueError('collinear reversal')
        result.append(2*((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))/denominator)
    return result


def transition(position, yaw, actual_curvature, target_points, target_station, length, spacing=.25):
    """Quintic x/y Hermite: pose/tangent/turn at both ends; generated once.

    These are world terminal constraints, not a virtual constant-offset lane.
    Target position, direction and terminal curvature come from supplied MGeo.
    """
    target = sample(target_points, target_station)
    if target is None or length <= 0:
        raise ValueError('target station unavailable')
    center, tangent = target
    end_curvature = curvature_at(target_points, target_station)
    forward = (math.cos(yaw),math.sin(yaw))
    coefficients = []
    for k in range(2):
        normal0, normal1 = (-forward[1],forward[0]),(-tangent[1],tangent[0])
        a0 = position[k];a1 = length*forward[k];a2 = .5*length**2*actual_curvature*normal0[k]
        d0 = center[k]-a0-a1-a2
        d1 = length*tangent[k]-a1-2*a2
        d2 = length**2*end_curvature*normal1[k]-2*a2
        coefficients.append((a0,a1,a2,10*d0-4*d1+.5*d2,-15*d0+7*d1-d2,6*d0-3*d1+.5*d2))
    points = []
    for i in range(math.ceil(length/spacing)+1):
        u = i/math.ceil(length/spacing)
        xy = [sum(a*u**j for j,a in enumerate(row)) for row in coefficients]
        points.append((*xy, float(center[2]) if len(center)>2 else 0.))
    # Snap only the algebraic endpoint to its exact measured centerline value;
    # vehicle state is never snapped by planner or closed-loop simulation.
    points[-1] = (center[0], center[1], float(center[2]) if len(center)>2 else 0.)
    return tuple(points)


def center_suffix(points, start, end, spacing=.25):
    if end <= start:
        raise ValueError('insufficient path support')
    first,last=sample(points,start),sample(points,end)
    if first is None or last is None:
        raise ValueError('reference range')
    raw=[first[0]]+[p for s,p in zip(arc(points),points) if start<s<end]+[last[0]]
    xyz=[(p[0],p[1],float(p[2]) if len(p)>2 else 0.) for p in raw]
    # Reuse the production corner-preserving per-segment subdivision. A global
    # sampling grid merged with native arc stations creates microscopic extra
    # segments and meaningless steering-rate spikes.
    local=LocalPathPlanner().extract(xyz,*xyz[0][:2],horizon_m=end-start,
                                    resample_spacing_m=spacing)
    if local is None:
        raise ValueError('invalid reference suffix')
    return tuple(tuple(p) for p in local['points'])


def fixed_path_suffix(points, station):
    """Clip progress without subdividing/refitting approved geometry."""
    lengths=arc(points);row=sample(points,station,lengths)
    if row is None:
        raise ValueError('fixed path range')
    return (tuple(row[0]),)+tuple(points[bisect_right(lengths,station):])


def control_local_path(fixed_points, pose, horizon_m=20.):
    """Publish/validate the same corner-preserving window of a fixed maneuver.

    Only progress clipping changes each cycle. No endpoint fitting, smoothing,
    virtual lane or new maneuver trajectory is created by this function.
    """
    local=LocalPathPlanner().extract(fixed_points,*pose[:2],horizon_m=horizon_m,
                                    resample_spacing_m=2.)
    if local is None or len(local['points'])<2:
        raise ValueError('invalid control local path')
    return local['points']


@dataclass(frozen=True)
class LatticeCandidate:
    side: str
    points: tuple
    entry_end_s: float
    return_start_s: float
    return_end_s: float
    transition_length_m: float
    cost: float
    cost_terms: dict
    rejection: str = ''
    entry_path: tuple = ()
    return_path: tuple = ()
    return_control_reserve_m: float = 0.
    return_preview_extension_m: float = 0.


@dataclass(frozen=True)
class CostWeights:
    reference: float = 1.
    adjacent: float = 1.5
    curvature: float = .2
    lane_switch: float = .2
    progress: float = .1

    def __post_init__(self):
        if (any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0
                for v in self.__dict__.values()) or min(self.reference,self.adjacent)<=0):
            raise ValueError('finite positive lane preference and nonnegative other costs required')


def reference_cost(points, reference, adjacent, weights=CostWeights(), *, projection_cache=None):
    """Distance integrals normalized over a COMMON prediction support.

    Positive reference/adjacent costs disfavor extended neighboring-lane travel.
    Safety never enters a weighted sum. Two switches are charged for a complete
    obstacle maneuver; fixed-direction commitment supplies hysteresis.
    """
    if projection_cache is not None:
        context=(id(reference),id(adjacent),tuple(map(tuple,reference)),tuple(map(tuple,adjacent)))
        if projection_cache.get('context')!=context:
            projection_cache.clear();projection_cache['context']=context
            projection_cache['indices']=(PolylineIndex(reference),PolylineIndex(adjacent))
        index,target_index=projection_cache['indices']
    else:
        index=PolylineIndex(reference);target_index=PolylineIndex(adjacent)
    lengths = arc(points)
    deviation = []
    occupancy=[]
    for p in points:
        key=tuple(p);cached=projection_cache.get(key) if projection_cache is not None else None
        if cached is None:
            row = sample(reference,index.project(p),index.lengths)
            if row is None:
                raise ValueError('cost reference range')
            distance=math.dist(p[:2],row[0][:2])
            target=sample(adjacent,target_index.project(p),target_index.lengths)
            if target is None:
                raise ValueError('cost adjacent reference range')
            cached=(distance,math.dist(p[:2],target[0][:2])<distance)
            if projection_cache is not None:projection_cache[key]=cached
        deviation.append(cached[0])
        # Nearest-center lane association is a SOFT occupancy cost, never a
        # fabricated lane boundary or permission to approve road containment.
        occupancy.append(cached[1])
    total = lengths[-1]
    support=index.project(points[-1])-index.project(points[0])
    if support<=0:
        raise ValueError('non-forward prediction support')
    lateral = sum(.5*(a*a+b*b)*(s1-s0) for a,b,s0,s1 in
                  zip(deviation,deviation[1:],lengths,lengths[1:]))/support
    adjacent_cost = sum(.5*(a+b)*(s1-s0) for a,b,s0,s1 in
                        zip(occupancy,occupancy[1:],lengths,lengths[1:]))/support
    ks = curvatures(points)
    bending=sum(.5*(a*a+b*b)*(s1-s0) for a,b,s0,s1 in
                zip(ks,ks[1:],lengths[1:],lengths[2:]))/support
    terms = dict(reference=lateral, adjacent=adjacent_cost,
                 curvature=bending, lane_switch=1., progress=total/support-1.)
    return sum(getattr(weights,k)*v for k,v in terms.items()), terms
