"""Preserve the unsmoothed pursuit response through steering reversals."""
import math
import pytest

from edgenode_control.target_smoothing_gate import TargetSmoothingGate


def test_reversal_hold_depends_on_distance_and_survives_the_zero_deadband():
    gate = TargetSmoothingGate()
    assert gate.update(.3, .1)
    assert gate.update(.01, .1)
    assert gate.update(-.3, .1) is False
    assert gate.remaining_m == 6.
    assert gate.update(-.3, 0.) is False
    assert gate.update(0., 2.) is False
    assert gate.update(-.2, 3.9) is False
    assert gate.update(-.2, .1)


def test_another_material_reversal_restarts_the_hold_and_stop_resets_it():
    gate = TargetSmoothingGate()
    gate.update(.3, 0.)
    gate.update(-.3, 0.)
    gate.update(-.3, 5.)
    assert gate.update(.2, .1) is False
    assert gate.remaining_m == 6.
    gate.reset()
    assert gate.remaining_m == 0.
    assert gate.update(-.3, 0.)


@pytest.mark.parametrize('steer,distance,hold', [
    (math.nan, .1, 6.), (.2, math.inf, 6.), (.2, -.1, 6.),
    (True, .1, 6.), (.2, .1, -1.), (.2, .1, 21.),
])
def test_invalid_measurement_does_not_change_history(steer, distance, hold):
    gate = TargetSmoothingGate()
    gate.update(.3, 0.)
    before = (gate.sign, gate.remaining_m)
    assert gate.update(steer, distance, hold) is None
    assert (gate.sign, gate.remaining_m) == before
