"""Causal ideal geometric tracker; no future bag poses, ROS or MORAI dynamics.

Advance by v*dt along the approved analytic quintic (or actual map suffix).
This is an ideal curvature-following vehicle, not Pure Pursuit or actuator
simulation. Optional bounded pose/yaw perturbations are applied each cycle.
"""
import math
from edgenode_planning.lane_geometry import sample,arc


def advance(planner,result,position,ds):
    curve=planner._world_transition
    if curve is not None:
        start=curve.project_parameter(position)
        # Simpson quadrature + bisection converts physical XY arc to the
        # polynomial parameter. Parameter derivatives are not vehicle m/s.
        def length(end):
            intervals=8;h=(end-start)/intervals
            speeds=[math.hypot(*curve.state(start+i*h)[1]) for i in range(intervals+1)]
            return h/3*(speeds[0]+speeds[-1]+4*sum(speeds[1:-1:2])+2*sum(speeds[2:-1:2]))
        remaining=length(curve.length)
        if ds<remaining:
            lo,hi=start,curve.length
            for _ in range(32):
                mid=(lo+hi)/2
                if length(mid)<ds:lo=mid
                else:hi=mid
            point,tangent,_=curve.state((lo+hi)/2)
            return point,math.atan2(tangent[1],tangent[0])
        ds-=remaining
        pair=planner._lane_pair
        geometry=pair.original if result['status']=='RECOVERING' else pair.adjacent
        index=pair.original_index if result['status']=='RECOVERING' else pair.adjacent_index
        end=index.project(curve.end)
        point,tangent=sample(geometry,min(arc(geometry)[-1],end+ds))
        return point[:2],math.atan2(tangent[1],tangent[0])
    # HOLD zero-length optimization publishes the exact map center suffix.
    points=result['points'];lengths=arc(points)
    point,tangent=sample(points,min(lengths[-1],ds))
    return point[:2],math.atan2(tangent[1],tangent[0])


def perturb(pair,position,yaw,magnitude,cycle):
    if not magnitude:return position,yaw
    S=pair.original_index.project(position);row=pair.at(S)
    if row is None:return position,yaw
    a,b=row['original'][0],row['adjacent'][0]
    vx,vy=(a[0]+b[0])/2-position[0],(a[1]+b[1])/2-position[1]
    distance=math.hypot(vx,vy)
    # Bounded tracking disturbance directed into the centerline strip. An
    # outward perturbation outside the gate must instead remain BLOCKED.
    if distance>1e-12:
        scale=magnitude*math.tanh(distance/.1)/distance
        position=[position[0]+scale*vx,position[1]+scale*vy]
    return position,yaw+min(.001,magnitude)*math.sin(cycle*.7)
