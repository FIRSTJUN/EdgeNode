import numpy as np
import pytest

from edgenode_planning.obstacle_tracking import ObstacleTracking,Observation,closing_ttc,observations_from_frame


def observed(x,y):return Observation((x,y),(((x-.5,y-.5),(x+.5,y-.5),(x+.5,y+.5),(x-.5,y+.5)),))


def test_source_world_motion_is_tracked_without_actor_positions_or_ids():
    tracker=ObstacleTracking()
    for i in range(31):
        tracks=tracker.update(1_000_000_000+i*100_000_000,[observed(10.+i*.25,2.),observed(30.,-3.)])
    moving,static=sorted(tracks,key=lambda t:t.identifier)
    assert moving.motion=='MOVING' and moving.state[2]==pytest.approx(2.5,abs=.1)
    assert static.motion=='STATIC' and static.speed_mps<.01
    assert moving.observation.footprints and np.linalg.eigvalsh(moving.covariance).min()>=0.


def test_expired_tracks_and_replayed_frames_cannot_refresh_confidence():
    tracker=ObstacleTracking();tracker.update(1_000_000_000,[observed(0.,0.)])
    with pytest.raises(ValueError):tracker.update(1_000_000_000,[observed(0.,0.)])
    assert tracker.update(2_000_000_000,[])==()
    assert closing_ttc(20.,10.,5.)==4.
    assert closing_ttc(20.,5.,10.)==float('inf')


def test_component_tiles_are_reassembled_without_mistaking_tile_boundaries_for_object_ids():
    piece=[(-1.,-1.),(0.,-1.),(1.,-1.),(1.,0.),(1.,1.),(0.,1.),(-1.,1.),(-1.,0.)]
    points=piece+[(x+2.,y) for x,y in piece]
    envelope=dict(points=points,objects=[dict(center_xy=(1.,0.),footprint_indices=[0,1])])
    observations=observations_from_frame(envelope)
    assert len(observations)==1 and len(observations[0].footprints)==2
    assert observations[0].center_xy==(1.,0.)
    envelope['objects'][0]['footprint_indices']=[0]
    with pytest.raises(ValueError):observations_from_frame(envelope)


def test_slow_vehicle_motion_uses_displacement_confidence_without_hiding_prediction_uncertainty():
    tracker=ObstacleTracking()
    for i in range(31):
        tracks=tracker.update(1_000_000_000+i*100_000_000,[observed(i*.055,0.),observed(20.,.08*np.sin(i))])
    moving,static=sorted(tracks,key=lambda t:t.identifier)
    assert moving.motion=='MOVING' and moving.speed_mps==pytest.approx(.55,abs=.05)
    assert moving.motion_sigma<moving.velocity_sigma
    assert static.motion=='STATIC'
