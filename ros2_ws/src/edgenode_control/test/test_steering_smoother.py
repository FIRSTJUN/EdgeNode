"""Response, time/rate bounds and stop recovery for actual steering shaping."""
import math

import pytest

from edgenode_control.steering_smoother import SteeringSmoother


def test_step_is_monotonic_and_rate_is_bounded_at_different_control_rates():
    for dt in (0.01, 1 / 30, 0.05):
        smoother = SteeringSmoother()
        commands = [0.0] + [smoother.update(0.7, dt, 0.08, 2.0) for _ in range(200)]
        assert all(0 <= b - a <= 2.0 * dt + 1e-12 for a, b in zip(commands, commands[1:]))
        assert commands[-1] == pytest.approx(0.7, abs=1e-8)


def test_alternating_sample_noise_is_attenuated_without_steady_turn_bias():
    smoother = SteeringSmoother()
    commands = [smoother.update(0.3 + 0.03 * (-1)**i, 1 / 30, 0.08, 2.0)
                for i in range(300)]
    assert max(commands[-100:]) - min(commands[-100:]) < 0.015
    assert sum(commands[-100:]) / 100 == pytest.approx(0.3)


def test_filter_response_is_time_based():
    outputs = []
    for dt in (0.01, 0.02, 0.05):
        smoother = SteeringSmoother()
        for _ in range(round(0.2 / dt)):
            command = smoother.update(0.5, dt, 0.08, 0.0)
        outputs.append(command)
    assert outputs == pytest.approx([0.5 * (1 - math.exp(-2.5))] * 3)


def test_reversal_clock_gap_reset_and_lower_absolute_limit():
    smoother = SteeringSmoother()
    smoother.update(0.7, 0.1)
    assert smoother.update(-0.7, 1 / 30, 0.08, 2.0) >= 0.7 - 2 / 30
    before = smoother.command
    assert smoother.update(-0.7, 100.0, 0.08, 2.0) >= before - 0.2
    assert smoother.update(-0.7, 0.0, 0.08, 2.0) == smoother.command
    assert abs(smoother.update(0.7, 0.01, 0.08, 2.0, 0.2)) <= 0.2
    smoother.reset()
    assert smoother.command == 0.0
    assert smoother.update(0.7, 0.01, 0.08, 2.0) <= 0.02


@pytest.mark.parametrize('values', [(math.nan, .03, .08, 2.), (.7, math.inf, .08, 2.),
    (.7, .03, -1., 2.), (.7, .03, .08, -1.), (True, .03, .08, 2.),
    (1.1, .03, .08, 2.), (.7, .03, .08, 2., 0.), (10**1000, .03, .08, 2.)])
def test_invalid_configuration_fails_closed(values):
    assert SteeringSmoother().update(*values) is None
