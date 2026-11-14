"""Offline continuation of S4: production steering and braking speed envelope.

These models never initialize ROS or authorize a MORAI speed increase.
"""
import math
import json
from pathlib import Path

import pytest

from edgenode_planning.speed_planner import SpeedPlanner
from edgenode_planning.morai_speed_safety import stopping_speed_cap
from offline_static_avoidance import run
from test_speed_planner import curvature_profile_path


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('radius', [math.inf, 300.])
def test_s4_fifteen_to_ten_avoids_and_returns_with_configured_steering_limit(side, radius):
    result = run(15., side, radius=radius, brake_to_default=True,
        reference_horizon_m=80., duration=85., config_overrides=dict(
            avoid_speed_kmh=10., steering_slew_limit_radps=.6,
            normal_obstacle_confirmations=2, normal_detection_latency_s=1.6))
    assert result['success'], result['failure']
    assert result['phases'] == ['NORMAL', 'DETECT', 'AVOID_ENTRY', 'AVOID_HOLD', 'RETURN', 'NORMAL']
    assert result['entry_commits'] == result['return_commits'] == 1
    assert result['entry_path_objects'] == result['return_path_objects'] == 1
    assert result['max_tracking_m'] <= .30
    assert result['min_body_clearance_m'] > .25
    assert result['max_wheel_rate_radps'] <= .6 + 1e-9
    assert result['clamp_count'] == 0
    assert all(row['speed_kmh'] <= 10.2 for row in result['rows']
               if row['state'] in ('AVOID_ENTRY', 'AVOID_HOLD', 'RETURN'))


def speed_planner(**overrides):
    parameters = dict(cruise_speed_kmh=50., curve_speed_kmh=25., sharp_curve_speed_kmh=15.,
                      preview_distance_m=80., anticipatory=True)
    parameters.update(overrides)
    return SpeedPlanner(**parameters)


@pytest.mark.parametrize('curvature,mode,ceiling', [
    (0., 'CRUISE', 50.), (.05, 'CURVE', 25.), (.12, 'SHARP_CURVE', 15.),
])
def test_current_curve_respects_class_speed_and_existing_lateral_acceleration(curvature, mode, ceiling):
    points = curvature_profile_path([(curvature, 80., .25)])
    result = speed_planner().plan(points, measured_speed_kmh=50.)
    assert result['speed_mode'] == mode
    assert 0. < result['target_speed_kmh'] <= ceiling
    if curvature:
        assert (result['target_speed_kmh'] / 3.6) ** 2 * curvature <= 1.5 + 1e-8
    else:
        assert result['target_speed_kmh'] == 50.


@pytest.mark.parametrize('curvature', [.05, .12])
def test_curve_approach_brakes_progressively_and_keeps_the_declared_envelope(curvature):
    planner = speed_planner()
    speeds = []
    for distance in (70., 50., 30., 15., 5., 0.):
        points = curvature_profile_path([(0., distance, .25), (curvature, 40., .25)])
        result = planner.plan(points, measured_speed_kmh=50.)
        speeds.append(result['target_speed_kmh'])
        if result['target_speed_kmh'] < 50.:
            available = max(0., result['braking_constraint_distance_m'] - 50. / 3.6 * .6)
            assert (result['target_speed_kmh'] / 3.6) ** 2 <= (
                (result['constraint_speed_kmh'] / 3.6) ** 2 + 2. * 2. * available + 1e-8)
    assert speeds[0] == 50.
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] < speeds[-2] < 50.


def test_s4_speed_profile_does_not_treat_a_fifty_kmh_request_as_driving_approval():
    recommendation = speed_planner().plan(curvature_profile_path([(0., 80., .5)]),
                                          measured_speed_kmh=15.)['target_speed_kmh']
    assert recommendation == 50.
    # Existing independent limits remain 15 km/h and 20 m verified range.
    admitted = min(recommendation, 15., stopping_speed_cap(20.))
    assert admitted == 15.


def test_recorded_short_corner_requests_braking_before_the_actual_tracking_failure():
    data = json.loads((Path(__file__).parent/'fixtures/kcity_short_corner_speed.json').read_text())
    result = speed_planner().plan(data['points'], measured_speed_kmh=data['measured_speed_kmh'])
    assert data['previous_recommendation_kmh'] == 50.
    assert result['speed_mode'] == 'SHARP_CURVE'
    assert 0. < result['target_speed_kmh'] < 15.
    assert 0. < result['braking_constraint_distance_m'] < 5.


@pytest.mark.parametrize('height', [-1., 1.])
def test_anticipatory_turn_envelope_keeps_a_cancelling_point_spike_out_of_the_profile(height):
    points = [(i*.25, height if i == 160 else 0., 0.) for i in range(321)]
    result = speed_planner().plan(points, measured_speed_kmh=50.)
    assert result['speed_mode'] == 'CRUISE'
    assert result['target_speed_kmh'] == 50.


@pytest.mark.parametrize('curvature', [-.16, .16])
def test_curve_exit_waits_for_measured_steering_to_unwind_before_accelerating(curvature):
    points = curvature_profile_path([(0., 80., .5)])
    result = speed_planner().plan(points, measured_speed_kmh=7., measured_curvature=curvature)
    assert result['speed_mode'] == 'SHARP_CURVE'
    assert result['target_speed_kmh'] == pytest.approx(3.6*math.sqrt(1.5/.16))
    assert speed_planner().plan(points, measured_speed_kmh=7., measured_curvature=0.)['target_speed_kmh'] == 50.
