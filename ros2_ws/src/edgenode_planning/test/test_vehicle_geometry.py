"""Synthetic geometry only: no test dimensions are Sonata specifications."""
import math
import pytest
from edgenode_planning.vehicle_geometry import VerifiedVehicleGeometry


def geometry(**changes):
    values=dict(width_m=1.,length_m=2.,wheelbase_m=1.5,front_overhang_m=.3,rear_overhang_m=.2,
                rear_axle_from_pose_forward_m=0.,rear_axle_from_pose_left_m=0.,
                pose_reference='synthetic rear axle',source='unit-test synthetic geometry')
    values.update(changes)
    return VerifiedVehicleGeometry(**values)


def test_verified_body_polygon_respects_reference_and_rotation():
    body=geometry()
    assert body.polygon(0.,0.,0.)==[(-.2,-.5),(1.8,-.5),(1.8,.5),(-.2,.5)]
    for point,expected in zip(body.polygon(0.,0.,math.pi/2),[(.5,-.2),(.5,1.8),(-.5,1.8),(-.5,-.2)]):
        assert point==pytest.approx(expected)


@pytest.mark.parametrize('changes',[{'width_m':None},{'length_m':math.nan},{'length_m':3.},
                                  {'pose_reference':''},{'source':''},{'wheelbase_m':False}])
def test_missing_or_unverified_geometry_cannot_create_a_body(changes):
    with pytest.raises(ValueError):geometry(**changes)
