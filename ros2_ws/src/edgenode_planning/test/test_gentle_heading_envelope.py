"""A CRUISE classification must not bypass sustained gentle-bend physics."""
import math
from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.longitudinal_speed_planner import (RouteProfileBuilder,
    LongitudinalSpeedPlanner, RouteSpeedProfile, CurveSpan)


def profile(points):
    local=SpeedPlanner(60.,40.,30.,10.,.025,.07)
    builder=RouteProfileBuilder(points,local)
    result=None
    while result is None:result=builder.advance(8)
    return local,result


def gentle_polyline():
    # Three 3.5deg turns distributed over 5m. Dense straight vertices between
    # them make the 2m curvature median zero; net heading still proves a bend.
    points=[[0.,0.,0.]]
    for heading,length in [(0.,5.),(3.5,2.5),(7.,2.5),(10.5,160.)]:
        for _ in range(round(length/.5)):
            x,y,_=points[-1]
            points.append([x+.5*math.cos(math.radians(heading)),
                           y+.5*math.sin(math.radians(heading)),0.])
    return points


def test_gentle_heading_support_limits_speed_without_changing_mode_or_path():
    points=gentle_polyline();before=[p[:] for p in points]
    local,route=profile(points)
    spans=[s for s in route.spans if s.level==0 and s.evidence&2]
    assert spans, 'sustained low curvature must reach the physical envelope'
    span=max(spans,key=lambda s:s.curvature)
    assert 0<span.curvature<local.curve_curvature_threshold
    speed=LongitudinalSpeedPlanner(route,local).plan(span.entry_m,30.,60.)
    assert speed['target_speed_kmh']<40.
    assert speed['target_speed_kmh']<=3.6*math.sqrt(1.5/span.curvature)
    assert speed['mode']=='CRUISE'
    assert points==before


def test_straight_still_accelerates_to_the_full_sixty_ceiling():
    points=[[i*.5,0.,0.] for i in range(601)]
    local,route=profile(points)
    result=LongitudinalSpeedPlanner(route,local).plan(50.,60.,60.)
    assert result['target_speed_kmh']==60.
    assert result['curve_id'] is None
    assert not route.spans


def test_small_distributed_heading_turns_are_physical_without_entering_curve_mode():
    points=[[0.,0.,0.]]
    for heading,length in [(0.,5.),(1.5,2.5),(3.,2.5),(4.5,160.)]:
        for _ in range(round(length/.5)):
            x,y,_=points[-1]
            points.append([x+.5*math.cos(math.radians(heading)),
                           y+.5*math.sin(math.radians(heading)),0.])
    local,route=profile(points)
    assert local.plan(points)['speed_mode']=='CRUISE'
    span=max(route.spans,key=lambda s:s.curvature)
    assert span.level==0
    speed=LongitudinalSpeedPlanner(route,local).plan(span.entry_m,40.,60.)
    assert 28.<speed['target_speed_kmh']<60.
    assert speed['mode']=='CRUISE'


def test_small_persistent_tangent_change_is_not_discarded_as_an_impulse():
    points=[[i*.5,0.,0.] for i in range(41)]
    for i in range(1,401):
        points.append([20.+i*.5*math.cos(math.radians(4.)),
                       i*.5*math.sin(math.radians(4.)),0.])
    local,route=profile(points)
    assert local.plan(points)['speed_mode']=='CRUISE'
    rescued=[s for s in route.spans if s.evidence&4]
    assert rescued
    assert all(s.level==0 and s.curvature<.025 for s in rescued)
    span=max(rescued,key=lambda s:s.curvature)
    result=LongitudinalSpeedPlanner(route,local).plan(span.entry_m,35.,60.)
    assert 28.<result['target_speed_kmh']<40.


def test_nonbinding_gentle_geometry_does_not_block_clear_road_recovery():
    local=SpeedPlanner(60.,40.,30.,10.,.025,.07)
    route=RouteSpeedProfile(300.,(CurveSpan(90.,150.,0,1,.003,.03,4),))
    planner=LongitudinalSpeedPlanner(route,local)
    result=planner.plan(110.,50.,60.,time_sec=0.,local_mode='CRUISE')
    assert result['target_speed_kmh']==60.
    assert result['curve_id'] is None
