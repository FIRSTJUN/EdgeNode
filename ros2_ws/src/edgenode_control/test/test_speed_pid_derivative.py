"""D filtering removes sample-rate impulses without raising a speed target."""
import math
import pytest
from edgenode_control.control_node import PID


def test_zero_filter_preserves_legacy_pid_arithmetic():
    pid=PID(.14,.015,.02,12.)
    previous=None;integral=0.
    for error in [.3,.8,-.4,-3.,1.]:
        dt=1/30;integral=max(-12.,min(12.,integral+error*dt))
        derivative=0. if previous is None else (error-previous)/dt
        assert pid.update(error,dt)==pytest.approx(.14*error+.015*integral+.02*derivative)
        previous=error


def test_sampled_speed_and_target_steps_have_smaller_derivative_impulses():
    raw=PID(0.,0.,.02,12.);filtered=PID(0.,0.,.02,12.)
    old=[];new=[]
    for i in range(180):
        # 20Hz target / 30Hz control, with a stepwise measured speed response.
        target=20.-.18*math.floor(i*2/3)
        measured=20.-.18*math.floor((i-2)*2/3) + (.1 if i%4==0 else 0.)
        old.append(raw.update(target-measured,1/30))
        new.append(filtered.update(target-measured,1/30,.10))
    assert sum(v*v for v in new)<.15*sum(v*v for v in old)
    assert max(abs(v) for v in new)<max(abs(v) for v in old)*.4


def test_constant_error_slope_keeps_derivative_dc_response_and_stop_clears_state():
    pid=PID(0.,0.,1.,12.)
    for i in range(120):out=pid.update(i/30,1/30,.1)
    assert out==pytest.approx(1.,abs=1e-10)
    pid.reset()
    assert pid.raw_derivative==pid.filtered_derivative==0.
    assert pid.update(30.,1/30,.1)==0.


@pytest.mark.parametrize('tau',[math.nan,math.inf,-.1,2.1,True,'0.1'])
def test_invalid_filter_does_not_mutate_history(tau):
    pid=PID(.14,.015,.02,12.)
    pid.update(1.,.1,.1)
    before=vars(pid).copy()
    assert pid.update(.2,.1,tau) is None
    assert vars(pid)==before


def test_saturated_start_does_not_store_a_long_throttle_bias():
    pid=PID(.14,.015,.02,12.)
    for _ in range(100):pid.update(20.,1/30,.1,output_limits=(-.8,.6))
    assert pid.integral==0.
    assert pid.update(0.,1/30,.1,output_limits=(-.8,.6))<=0.


def test_braking_saturation_unwinds_opposite_history_without_winding_negative():
    pid=PID(.14,.015,0.,12.);pid.integral=8.
    for _ in range(20):pid.update(-20.,.1,.1,output_limits=(-.8,.6))
    assert pid.integral==0.
    pid.update(.2,.1,.1,output_limits=(-.8,.6))
    assert pid.integral>0.


@pytest.mark.parametrize('limits',[(0.,.6),(-.8,0.),(-.8,math.nan),(-.8,True)])
def test_invalid_actuator_limits_fail_before_mutating_pid(limits):
    pid=PID(.14,.015,.02,12.);before=vars(pid).copy()
    assert pid.update(1.,.1,.1,output_limits=limits) is None
    assert vars(pid)==before
