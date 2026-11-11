import pytest
from test_live_test import tool, parameters


@pytest.mark.parametrize('budget',['nan','inf','0','2.1'])
def test_invalid_budget_cannot_prepare_or_drive(budget):
    with pytest.raises(SystemExit):tool.parse_args(['--prepare','--lateral-budget-mps2',budget])


def test_increased_budget_requires_a_separate_actual_lateral_monitor():
    with pytest.raises(SystemExit):tool.parse_args(['--run','--lateral-budget-mps2','1.8'])
    with pytest.raises(SystemExit):tool.parse_args(['--run','--lateral-budget-mps2','1.8','--max-lateral-accel-mps2','4'])
    a=tool.parse_args(['--run','--max-speed-kmh','60','--lateral-budget-mps2','1.8','--max-lateral-accel-mps2','3'])
    assert a.max_speed_kmh==60. and a.lateral_budget_mps2==1.8


def test_experimental_budget_never_enables_drive_or_changes_the_speed_profile():
    p=tool.preparation_parameters(60.,False,1.8)
    assert p['enable_drive'] is False
    assert [p[k] for k in ['cruise_speed_kmh','curve_speed_kmh','sharp_curve_speed_kmh']]==[60.,40.,30.]
    assert p['max_drive_speed_kmh']==60.


def test_existing_default_remains_conservative_and_run_budget_must_match():
    p=parameters(60.);p['speed_plan_lateral_accel_mps2']=1.8
    with pytest.raises(RuntimeError):tool.verify_parameters(p,60.)
    tool.verify_parameters(p,60.,False,1.8)
    with pytest.raises(RuntimeError):tool.verify_parameters(p,60.,False,1.7)
    assert 'speed_plan_lateral_accel_mps2' not in tool.preparation_parameters(60.)
