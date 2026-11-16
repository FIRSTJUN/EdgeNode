"""NORMAL response compensation and pedal anti-windup preserve S4 gates."""
import math

import pytest
from std_msgs.msg import String

from edgenode_control.control_node import PID
from edgenode_control.pure_pursuit import PurePursuit
import test_control_node as control_tests


def test_zero_prediction_is_exactly_the_existing_pp():
    pp=PurePursuit();points=[(0.,1.,0.),(20.,1.,0.)]
    assert pp.compute_compensated(points,0.,0.,0.,speed_kmh=30.,measured_curvature=0.,
        prediction_time_sec=0.)==pp.compute(points,0.,0.,0.)


def test_prediction_preserves_measured_tracking_error_and_exact_arc_motion():
    points=[(float(x),0.,0.) for x in range(41)]
    result=PurePursuit().compute_compensated(points,0.,1.,0.,speed_kmh=36.,
        measured_curvature=.1,prediction_time_sec=.2)
    assert result['nearest_distance_m']==pytest.approx(1.)
    assert result['predicted_nearest_distance_m']>1.
    assert result['prediction_pose']==pytest.approx((10*math.sin(.2),1+10*(1-math.cos(.2)),.2))


@pytest.mark.parametrize('time',[-.1,.6,math.nan,True])
def test_invalid_prediction_never_produces_steering(time):
    assert PurePursuit().compute_compensated([(0.,0.,0.),(20.,0.,0.)],0.,0.,0.,
        speed_kmh=30.,measured_curvature=0.,prediction_time_sec=time) is None


@pytest.mark.parametrize('error',[30.,-30.])
def test_saturated_pedals_do_not_accumulate_windup(error):
    pid=PID(.14,.015,.02,12.)
    for _ in range(100):pid.update(error,.033,output_limits=(-.8,.6))
    assert pid.integral==0.
    pid.update(.1,.033,output_limits=(-.8,.6))
    # A sudden sign change can saturate the derivative for one sample.
    pid.update(.1,.033,output_limits=(-.8,.6))
    assert 0<pid.integral<.01


@pytest.fixture
def control():
    case=control_tests.ControlNodeTest();case.setUp()
    try:yield case
    finally:case.doCleanups()


def test_prediction_is_normal_only_and_stale_state_falls_back(control):
    case=control;n=case.node;case.parameters['normal_prediction_time_sec']=.1
    n.current_speed_kmh=20.;n.target_speed_kmh=20.
    n.avoidance_state_cb(String(data='NORMAL'))
    case.command();assert n.pure_pursuit.compute_compensated.call_count==1
    n.avoidance_state_cb(String(data='AVOID_ENTRY'))
    case.command();assert n.pure_pursuit.compute_compensated.call_count==1
    n.avoidance_state_cb(String(data='NORMAL'))
    n._avoidance_state_stamp_ns=case.now-700_000_000
    case.command();assert n.pure_pursuit.compute_compensated.call_count==1


def test_normal_physical_steering_does_not_raise_s4_limit(control):
    case=control;n=case.node;case.parameters['normal_max_front_steer_normalized']=1.
    n.target_speed_kmh=10.;case.set_path([(5.*math.sin(i*.05),5.*(1-math.cos(i*.05)),0.) for i in range(41)])
    n.avoidance_state_cb(String(data='NORMAL'));normal=case.command()
    assert .7<abs(normal.front_steer)<=1.
    n.avoidance_state_cb(String(data='AVOID_ENTRY'));avoid=case.command()
    assert abs(avoid.front_steer)<=.7
