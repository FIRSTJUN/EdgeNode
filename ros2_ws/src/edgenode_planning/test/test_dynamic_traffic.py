from types import SimpleNamespace
import numpy as np
import pytest

from edgenode_planning.dynamic_traffic import assess_trajectory,following_speed_kmh,longitudinal_kinematics
from edgenode_planning.obstacle_tracking import Observation
from edgenode_planning.vehicle_geometry import sonata_geometry


def vehicle(identifier,x,y,v):
    piece=tuple((x+a,y+b) for a,b in ((-2.,-.9),(2.,-.9),(2.,.9),(-2.,.9)))
    return SimpleNamespace(identifier=identifier,state=np.array((x,y,v,0.)),
        covariance=np.diag((.01,.01,.005,.005)),observed_ns=1_000_000_000,
        observation=Observation((x,y),(piece,)),motion='MOVING')


def changing_path():
    result=[]
    for i in range(161):
        x=i*.25;u=min(1.,x/20.);y=3.5*(10*u**3-15*u**4+6*u**5)
        result.append((x,y,0.))
    return result


def assess(tracks):
    return assess_trajectory(changing_path(),(0.,0.,0.),10.,tracks,sonata_geometry(),
        completion_distance_m=22.,now_ns=1_000_000_000,source_ns=1_000_000_000)


def test_slow_lead_and_empty_adjacent_lane_allow_pass_but_fast_rear_arrival_rejects_it():
    assert assess([vehicle(1,35.,0.,1.5)]).ready
    blocked=assess([vehicle(1,35.,0.,1.5),vehicle(2,-25.,3.5,8.)])
    assert not blocked.ready and blocked.conflict_track_id==2
    # A later, genuinely observed gap permits reconsideration of the same
    # maneuver. The solver does not permanently blacklist that direction.
    assert assess([vehicle(1,35.,0.,1.5),vehicle(2,25.,3.5,8.)]).ready


def test_waiting_for_gap_uses_following_speed_and_does_not_force_a_stop():
    assert 0.<following_speed_kmh(8.,6.,10.)<6.
    assert following_speed_kmh(30.,6.,10.)==10.
    assert following_speed_kmh(0.,0.,10.)==0.


def test_measured_lead_speed_and_relative_future_gaps_keep_units_and_lateral_scope():
    result=longitudinal_kinematics([(0.,0.,0.),(80.,0.,0.)],(0.,0.,0.),10.8,
        [vehicle(1,35.,0.,1.5),vehicle(2,20.,5.,.5)],now_ns=1_000_000_000,source_ns=1_000_000_000)
    assert result['ready'] and result['lead_candidate_id']==1
    lead=result['tracks'][0]
    assert lead['forward_speed_mps']==1.5 and lead['closing_speed_mps']==pytest.approx(1.5)
    assert lead['future_gaps'][0]['nominal_gap_m']==pytest.approx(lead['longitudinal_gap_m']-3.)
    assert lead['future_gaps'][0]['lower_bound_m']<lead['future_gaps'][0]['nominal_gap_m']
    assert not result['tracks'][1]['intersects_body_corridor']
    assert not result['driving_policy_enabled'] and not lead['lane_membership_verified']


def test_faster_rear_track_has_a_closing_gap_without_becoming_a_lead():
    result=longitudinal_kinematics([(0.,0.,0.),(80.,0.,0.)],(0.,0.,0.),10.8,
        [vehicle(2,-8.,0.,6.)],now_ns=1_000_000_000,source_ns=1_000_000_000)
    rear=result['tracks'][0]
    assert result['lead_candidate_id'] is None and rear['relation']=='BEHIND'
    assert rear['closing_speed_mps']==pytest.approx(3.) and rear['future_gaps'][0]['nominal_gap_m']<0.


def test_stale_source_and_nonfinite_ego_cannot_create_current_traffic_kinematics():
    points=[(0.,0.,0.),(80.,0.,0.)]
    assert not longitudinal_kinematics(points,(0.,0.,0.),10.,[],
        now_ns=1_700_000_000,source_ns=1_000_000_000)['ready']
    with pytest.raises(ValueError):
        longitudinal_kinematics(points,(0.,0.,0.),float('nan'),[],now_ns=1,source_ns=1)
