import math
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from std_msgs.msg import Float32
from edgenode_control.control_node import ControlNode, PID


def control():
    p = {'command_timeout_sec': .6, 'status_timeout_sec': .6,
         'max_front_steer_normalized': .85, 'steering_sign': -1.,
         'steering_filter_sec': .1, 'steering_rate_limit': 1.5,
         'max_accel_cmd': .25, 'max_brake_cmd': .8}
    c = SimpleNamespace(now_ns=lambda: 1_000_000_000,
                        last_control_ns=966_666_667,
                        error_stamp_ns=990_000_000, speed_stamp_ns=990_000_000,
                        steering_stamp_ns=990_000_000, status_stamp_ns=990_000_000,
                        get_parameter=lambda name: SimpleNamespace(value=p[name]),
                        publish_safe_stop=Mock(), steer_pid=PID(.8, 0, .12, .5),
                        speed_pid=PID(.14, .015, 0, 12), target_steering=.3,
                        last_steering=0., target_speed_kmh=3., current_speed_kmh=2.,
                        ctrl_pub=Mock(), get_clock=lambda: SimpleNamespace(
                            now=lambda: SimpleNamespace(to_msg=lambda: __import__('builtin_interfaces.msg',fromlist=['Time']).Time())))
    return c


@pytest.mark.parametrize('stream', ['error_stamp_ns','speed_stamp_ns','steering_stamp_ns','status_stamp_ns'])
def test_one_stale_stream_cannot_be_hidden_by_other_messages(stream):
    c=control();setattr(c,stream,100_000_000)
    ControlNode.control_loop(c)
    c.publish_safe_stop.assert_called_once()
    c.ctrl_pub.publish.assert_not_called()


@pytest.mark.parametrize('callback,stamp', [('target_error_cb','error_stamp_ns'),
                                           ('target_speed_cb','speed_stamp_ns'),
                                           ('target_steering_cb','steering_stamp_ns')])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_commands_invalidate_stream(callback,stamp,value):
    c=control();getattr(ControlNode,callback)(c,Float32(data=value))
    assert getattr(c,stamp)==0
    ControlNode.control_loop(c)
    c.publish_safe_stop.assert_called_once()


def test_left_pursuit_maps_to_negative_morai_command_and_rate_is_bounded():
    c=control();ControlNode.control_loop(c)
    cmd=c.ctrl_pub.publish.call_args.args[0]
    assert -.051 <= cmd.front_steer < 0
    assert cmd.accel > 0 and cmd.brake==0


def test_stop_command_brakes_and_clears_speed_integral():
    c=control();c.target_speed_kmh=0.;c.speed_pid.integral=12.
    ControlNode.control_loop(c)
    cmd=c.ctrl_pub.publish.call_args.args[0]
    assert cmd.accel==0 and cmd.brake>=.75
    assert c.speed_pid.integral==0.


def test_reduced_speed_target_resets_old_integral():
    c=control();c.speed_pid.integral=10.
    ControlNode.target_speed_cb(c,Float32(data=1.0))
    assert c.speed_pid.integral==0.


def test_stored_integral_cannot_accelerate_when_overspeeding():
    c=control();c.current_speed_kmh=4.;c.speed_pid.integral=12.
    ControlNode.control_loop(c)
    cmd=c.ctrl_pub.publish.call_args.args[0]
    assert cmd.accel==0. and cmd.brake>=.08


def test_pedal_saturation_does_not_accumulate_integral():
    c=control();c.current_speed_kmh=0.;c.target_speed_kmh=6.
    ControlNode.control_loop(c)
    assert c.speed_pid.integral==pytest.approx(0.)
