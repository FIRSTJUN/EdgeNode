"""Local friction constraints are explicit and never an independent speed cap."""
import math

import pytest

from edgenode_planning.speed_planner import SpeedPlanner


def circle(radius,straight=0.):
    start=[(i*.5,0.,0.) for i in range(round(straight*2)+1)]
    return start+[(straight+radius*math.sin(i*.02),radius*(1-math.cos(i*.02)),0.)
                  for i in range(1,101)]


def planner(**kwargs):
    return SpeedPlanner(anticipatory=True,preview_distance_m=80.,curve_curvature_threshold=.02,
        sharp_curvature_threshold=.03,physical_lateral_acceleration_mps2=9.,**kwargs)


def test_60_straight_and_feasible_30_curve_targets_are_preserved():
    assert planner().plan([(float(x),0.,0.) for x in range(81)])['target_speed_kmh']==60.
    result=planner().plan(circle(8.),measured_speed_kmh=30.)
    assert result['target_speed_kmh']==pytest.approx(30.)
    assert result['constraint_reason']!='LATERAL_FRICTION'


def test_tight_real_curve_reports_quantified_physical_limit():
    result=planner().plan(circle(4.),measured_speed_kmh=30.)
    assert result['target_speed_kmh']==pytest.approx(3.6*math.sqrt(9.*4.),rel=.01)
    assert result['constraint_reason']=='LATERAL_FRICTION'
    assert result['constraint_speed_kmh']<30.


def test_future_physical_constraint_has_anticipatory_braking_distance():
    result=planner().plan(circle(4.,30.),measured_speed_kmh=60.)
    assert 21.<result['target_speed_kmh']<60.
    assert result['braking_constraint_distance_m']>20.
    assert result['constraint_reason']=='LATERAL_FRICTION'


def test_disabled_physical_model_preserves_legacy_configured_target():
    result=SpeedPlanner(anticipatory=True,preview_distance_m=80.).plan(circle(4.))
    assert result['target_speed_kmh']==pytest.approx(30.)
    assert 'constraint_reason' not in result


@pytest.mark.parametrize('value',[-1.,math.nan,math.inf])
def test_invalid_physical_budget_fails_closed(value):
    p=planner();p.physical_lateral_acceleration_mps2=value
    assert p.plan(circle(8.)) is None
