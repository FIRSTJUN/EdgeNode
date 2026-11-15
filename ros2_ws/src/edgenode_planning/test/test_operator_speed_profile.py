"""Operator targets remain the only running speed profile; safety still stops."""
import pytest
from edgenode_planning.speed_planner import SpeedPlanner
from test_speed_planner import curvature_profile_path


@pytest.mark.parametrize('normal,sharp', [(60., 30.), (70., 40.), (90., 50.)])
@pytest.mark.parametrize('curvature', [0., .02, .05, .09])
def test_ordinary_curves_keep_operator_normal_speed(normal, sharp, curvature):
    planner = SpeedPlanner(cruise_speed_kmh=normal, curve_speed_kmh=3.,
        sharp_curve_speed_kmh=sharp, anticipatory=True, preview_distance_m=80.)
    result = planner.plan(curvature_profile_path([(curvature, 80., .25)]),
        measured_speed_kmh=normal, measured_curvature=curvature)
    assert result['speed_mode'] == 'NORMAL'
    assert result['target_speed_kmh'] == normal


@pytest.mark.parametrize('normal,sharp', [(60., 30.), (70., 40.)])
def test_sharp_target_is_configurable_and_not_laterally_reduced(normal, sharp):
    planner = SpeedPlanner(cruise_speed_kmh=normal, sharp_curve_speed_kmh=sharp,
        anticipatory=True, preview_distance_m=80.)
    result = planner.plan(curvature_profile_path([(.12, 80., .25)]),
        measured_speed_kmh=normal, measured_curvature=.12)
    assert result['speed_mode'] == 'SHARP_CURVE'
    assert result['target_speed_kmh'] == sharp


def test_sharp_approach_uses_only_declared_target_and_braking_distance():
    planner = SpeedPlanner(anticipatory=True, preview_distance_m=80.)
    speeds = [planner.plan(curvature_profile_path([(0., distance, .25), (.12, 40., .25)]),
        measured_speed_kmh=60.)['target_speed_kmh'] for distance in (70., 40., 20., 5., 0.)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[0] == 60. and speeds[-1] == pytest.approx(30.)
    assert min(speeds) >= 30.
