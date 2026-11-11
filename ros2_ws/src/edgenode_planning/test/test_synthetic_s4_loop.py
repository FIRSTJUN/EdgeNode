"""Production Control loop and causal plant, including post-stop vehicle motion."""
from functools import lru_cache

import pytest

from synthetic_s4_loop import Scene, simulate
from edgenode_control.control_node import ControlNode
from synthetic_s4_loop import OfflineControl


@lru_cache(maxsize=16)
def outcome(**values):
    return simulate(Scene(**values))


@pytest.mark.parametrize('radius', [0., 300.])
def test_pid_pp_vehicle_avoids_returns_holds_normal_and_brakes(radius):
    result = outcome(radius_m=radius)
    assert result['success'], (result['reason'], result['transitions'])
    assert [r['state'] for r in result['transitions']] == [
        'NORMAL', 'DETECT', 'AVOID_ENTRY', 'AVOID_HOLD', 'RETURN', 'NORMAL']
    assert result['post_return_duration_s'] >= 2.
    assert result['safe_model_stop'] and result['final_speed_kmh'] == 0.
    assert result['stop_hold_s'] >= 1.-1e-7
    assert result['min_obstacle_clearance_m'] > .25
    assert result['max_lateral_error_m'] <= .30
    assert result['max_wheel_rate_radps'] <= .6+1e-8
    assert result['return_error_m'] <= .15
    assert any(r['accel'] > 0 for r in result['rows'])
    assert any(r['brake'] > 0 and r['speed_kmh'] > 0 for r in result['rows'])
    assert 'GOAL_REACHED' == result['planning_fsm'][-1]['state']


@pytest.mark.parametrize('speed', [3., 10., 20.])
def test_current_operational_constraints_never_accept_synthetic_approval(speed):
    result = outcome(experiment='B', initial_speed_kmh=speed)
    assert not result['success'] and result['blocked'] and result['safe_model_stop']
    assert result['reason'] == 'S4_DISTANCE_OBSERVATION_EVIDENCE_UNAVAILABLE'
    assert result['transitions'][-1]['state'] == 'BLOCKED'
    assert result['observation']['roi_ceiling_m'] == 21.19
    assert not result['observation']['synthetic_direct_injection']
    assert result['candidates'] == []
    assert result['stop_time_s'] > 0 and result['stop_distance_m'] > 0
    assert result['rows'][0]['speed_kmh'] > 0  # no speed snap at BLOCKED
    assert result['final_speed_kmh'] == 0.


@pytest.mark.parametrize('fault', ['sensor_dropout', 'tracking_failure', 'compute_timeout', 'return_obstruction'])
def test_faults_latch_blocked_and_vehicle_actually_stops(fault):
    result = outcome(fault=fault, obstacle_front_m=35.)
    assert not result['success'] and result['blocked'] and result['safe_model_stop'], result['reason']
    assert result['stop_hold_s'] >= 1.-1e-7
    assert all(r['state'] == 'BLOCKED' for r in result['rows'] if r['t'] >= result['transitions'][-1]['t'])
    if fault == 'compute_timeout':
        assert result['modeled_budget_overruns'] == 1
        assert 'S4_SENSOR_' in result['reason'] or 'PLANNING_BUDGET_EXCEEDED' in result['reason']


def test_harness_binds_actual_production_control_methods():
    assert OfflineControl.control_loop is ControlNode.control_loop
    assert OfflineControl.publish_safe_stop is ControlNode.publish_safe_stop
