import math
import pytest
from edgenode_planning.goal_approach import GoalApproach


ROUTE=((0.,0.,0.),(100.,0.,0.))


def test_braking_reserves_response_and_finite_deceleration():
    goal=GoalApproach()
    state=goal.update(ROUTE,65.,(35.,0.,0.),60.,1.,armed=True)
    velocity=state['speed_cap_kmh']/3.6
    assert velocity < 60/3.6
    assert velocity*.8+velocity**2/(2*1.5) == pytest.approx(65.-.35)


def test_loop_proximity_cannot_claim_arrival():
    goal=GoalApproach()
    for t in (1.,4.,8.):
        state=goal.update(ROUTE,2400.,(100.,0.,0.),0.,t,armed=True)
        assert not state['reached'] and state['phase']=='CRUISE'


@pytest.mark.parametrize('pose,speed,armed', [((99.7,0.,.3),0.,True),
    ((99.7,0.,0.),.2,True),((99.7,0.,0.),0.,False),((98.7,0.,0.),0.,True)])
def test_position_heading_speed_and_arming_all_required(pose,speed,armed):
    goal=GoalApproach()
    for t in (1.,4.):
        assert not goal.update(ROUTE,.3,pose,speed,t,armed=armed)['reached']


def test_measured_hold_latches_until_explicit_mission_reset():
    goal=GoalApproach()
    assert not goal.update(ROUTE,.3,(99.7,0.,0.),0.,1.,armed=True)['reached']
    state=goal.update(ROUTE,.3,(99.7,0.,0.),0.,3.1,armed=True)
    assert state['reached'] and state['speed_cap_kmh']==0.
    # Drift cannot silently authorize a restart after successful arrival.
    assert goal.update(ROUTE,1.,(99.,0.,0.),0.,4.,armed=True)['speed_cap_kmh']==0.
    goal.reset()
    assert not goal.update(ROUTE,50.,(50.,0.,0.),0.,5.,armed=True)['reached']


def test_clock_regression_is_rejected():
    goal=GoalApproach();goal.update(ROUTE,10.,(90.,0.,0.),2.,2.,armed=True)
    with pytest.raises(ValueError):goal.update(ROUTE,10.,(90.,0.,0.),2.,1.,armed=True)


def test_missing_terminal_tangent_remains_invalid_input():
    with pytest.raises(ValueError,match='terminal tangent'):
        GoalApproach().update(((0.,0.,0.),(0.,0.,0.)),0.,(0.,0.,0.),0.,1.,armed=True)
