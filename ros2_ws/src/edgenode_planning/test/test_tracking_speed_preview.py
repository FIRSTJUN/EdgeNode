"""NORMAL retains its straight target and anticipates controller-limited turns."""
import math
import pytest

from edgenode_planning.speed_planner import SpeedPlanner


def road(straight,radius,length=120.):
    points=[]
    for i in range(round(length*2)+1):
        s=i*.5
        if s<=straight:points.append((s,0.,0.))
        else:
            turn=(s-straight)/radius
            points.append((straight+radius*math.sin(turn),radius*(1-math.cos(turn)),0.))
    return points


def planner():
    return SpeedPlanner(preview_distance_m=80.,anticipatory=True,
        curve_curvature_threshold=.02,sharp_curvature_threshold=.03,physical_lateral_acceleration_mps2=9.,
        tracking_lateral_acceleration_mps2=3.)


def test_straight_keeps_sixty_and_supported_curve_has_early_deceleration():
    p=planner()
    assert p.plan(road(120.,40.),measured_speed_kmh=60.)['target_speed_kmh']==60.
    distant=p.plan(road(70.,40.),measured_speed_kmh=60.)
    approach=p.plan(road(25.,40.),measured_speed_kmh=60.)
    on_curve=p.plan(road(0.,40.),measured_speed_kmh=40.)
    assert distant['target_speed_kmh']==60.
    assert 39.<on_curve['target_speed_kmh']<approach['target_speed_kmh']<60.
    assert approach['constraint_reason']=='TRACKING_CURVATURE'


@pytest.mark.parametrize('radius',[40.,-40.])
def test_curve_limits_are_direction_independent_and_measured_steering_is_respected(radius):
    p=planner();result=p.plan(road(0.,radius),measured_speed_kmh=60.)
    assert result['target_speed_kmh']==pytest.approx(3.6*math.sqrt(3.*abs(radius)),rel=.01)
    unwind=p.plan(road(120.,40.),measured_speed_kmh=50.,measured_curvature=1/abs(radius))
    assert unwind['constraint_reason']=='TRACKING_CURVATURE'
    assert unwind['target_speed_kmh']<50.
