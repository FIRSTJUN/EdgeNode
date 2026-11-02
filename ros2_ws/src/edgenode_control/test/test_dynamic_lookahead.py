"""Measured-speed preview distances and invalid-input handling without ROS."""

import math

import pytest

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance


@pytest.mark.parametrize('speed_kmh, expected_m', [
    (0.0, 2.5),
    (20.0, 2.5),
    (25.0, 3.125),
    (30.0, 3.75),
    (40.0, 5.0),
    (50.0, 6.25),
    (1000.0, 10.0),
])
def test_baseline_preview_and_upper_clamp(speed_kmh, expected_m):
    assert calculate_lookahead_distance(speed_kmh) == pytest.approx(expected_m)


@pytest.mark.parametrize('speed_kmh', [
    None, '50', True, [], -1.0, math.nan, math.inf, -math.inf, 10 ** 400,
])
def test_invalid_speed_fails_closed(speed_kmh):
    assert calculate_lookahead_distance(speed_kmh) is None


@pytest.mark.parametrize('parameter', [
    'lookahead_min_m', 'lookahead_max_m', 'lookahead_preview_time_sec',
])
def test_invalid_parameter_types_and_nonfinite_values_fail_closed(parameter):
    for value in (None, '2.5', True, [], math.nan, math.inf, -math.inf, 10 ** 400):
        assert calculate_lookahead_distance(50.0, **{parameter: value}) is None


@pytest.mark.parametrize('parameters', [
    {'lookahead_min_m': 0.0},
    {'lookahead_min_m': -1.0},
    {'lookahead_min_m': 11.0},
    {'lookahead_max_m': 0.0},
    {'lookahead_max_m': 2.0},
    {'lookahead_preview_time_sec': 0.0},
    {'lookahead_preview_time_sec': -0.45},
])
def test_invalid_parameter_ranges_fail_closed(parameters):
    assert calculate_lookahead_distance(50.0, **parameters) is None


def test_configurable_preview_and_bounds():
    parameters = dict(lookahead_min_m=3.0, lookahead_max_m=8.0,
                      lookahead_preview_time_sec=0.6)
    assert calculate_lookahead_distance(0.0, **parameters) == 3.0
    assert calculate_lookahead_distance(30.0, **parameters) == 5.0
    assert calculate_lookahead_distance(100.0, **parameters) == 8.0


def test_equal_bounds_produce_constant_distance():
    for speed_kmh in (0.0, 20.0, 50.0, 1000.0):
        assert calculate_lookahead_distance(
            speed_kmh, lookahead_min_m=2.5, lookahead_max_m=2.5) == 2.5
