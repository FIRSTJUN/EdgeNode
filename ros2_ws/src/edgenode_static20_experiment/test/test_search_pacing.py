import math
import pytest
from edgenode_static20_experiment.search_pacing import approach_cap


def test_first_slow_search_does_not_spend_a_seven_second_approach_in_two_seconds():
    # Actual v11 first Offset:11.11m reserved,8.834s episode remaining.
    cap,horizon=approach_cap(11.111111,.4,4.,0.,8.83442)
    assert horizon==8.83442 and cap<5.
    # Once the initial braking ends, an ideal follower conserves distance
    # through the recorded7.344s solve instead of exhausting it after2s.
    remaining=6.;elapsed=1.5
    while elapsed<7.344:
        v,_=approach_cap(remaining,.4,4.,0.,8.83442-elapsed)
        remaining-=v/3.6*.02;elapsed+=.02
    assert remaining>.5 and v>.15


def test_zero_or_short_support_never_grants_motion_past_the_original_brake_bound():
    assert approach_cap(0.,.4,4.,0.,8.)[0]==0.
    for remaining in (.001,.1,1.,20.):
        cap,horizon=approach_cap(remaining,.4,4.,0.,8.)
        velocity=cap/3.6
        assert velocity*.4+velocity**2/(2*4.)<=remaining+1e-12
        assert velocity*horizon<=remaining+1e-12


def test_original_unbudgeted_horizon_and_invalid_input_behavior():
    assert approach_cap(10.,.4,4.,0.)[1]==2.
    assert approach_cap(10.,.4,4.,7000.,3.)[1]==8.
    for values in [(-1.,.4,4.,0.),(1.,.4,0.,0.),(math.nan,.4,4.,0.)]:
        with pytest.raises(ValueError):approach_cap(*values)
