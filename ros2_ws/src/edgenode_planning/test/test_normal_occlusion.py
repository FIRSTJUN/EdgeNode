import math
import pytest
from shapely.geometry import LineString,Polygon
from edgenode_planning.normal_occlusion import occlusion_limit
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope


def box(x,y,r=.25):
    return [(x-r,y-r),(x,y-r),(x+r,y-r),(x+r,y),(x+r,y+r),(x,y+r),(x-r,y+r),(x-r,y)]


def limit(reference,objects,age=.1):
    return occlusion_limit(reference,objects,(0.,0.,0.),morai_sonata_collision_envelope(),
        deceleration_mps2=4.,source_age_s=age,sensor_forward_m=1.19)


def test_roadside_objects_with_clear_straight_rays_do_not_reduce_cruise():
    result=limit([(float(x),0.,0.) for x in range(81)],[box(30.,10.)])
    assert not result['limited'] and result['speed_cap_kmh'] is None


def test_a_known_inner_curve_occluder_limits_speed_without_occupying_the_reference():
    reference=[(30*math.sin(a/100),30*(1-math.cos(a/100)),0.) for a in range(151)]
    obstacle=box(26.5*math.sin(.7),30-26.5*math.cos(.7))
    assert LineString(reference).distance(Polygon(obstacle))>3.
    result=limit(reference,[obstacle])
    assert result['limited'] and 0<result['speed_cap_kmh']<60.
    v=result['speed_cap_kmh']/3.6
    assert v*result['response_time_s']+v*v/8.==pytest.approx(result['available_stopping_m'])
    assert limit(reference,[obstacle],age=.6)['speed_cap_kmh']<result['speed_cap_kmh']


def test_finite_goal_endpoint_is_not_treated_as_an_occluder_or_extended():
    reference=[(0.,0.,0.),(.3,0.,0.)]
    assert limit(reference,[box(20.,2.)])['speed_cap_kmh'] is None
    assert reference[-1]==(.3,0.,0.)


def test_invalid_age_is_rejected_instead_of_relabelled_fresh():
    with pytest.raises(ValueError):limit([(0.,0.,0.),(80.,0.,0.)],[],age=-1.)


@pytest.mark.parametrize('radius',[20.,30.,60.])
def test_native_ray_checks_preserve_the_full_body_visibility_bound(radius):
    reference=[(radius*math.sin(a/100),radius*(1-math.cos(a/100)),0.) for a in range(151)]
    objects=[box((radius-3.5)*math.sin(a),radius-(radius-3.5)*math.cos(a)) for a in (.3,.6,.9)]
    kwargs=dict(deceleration_mps2=4.,source_age_s=.2,sensor_forward_m=1.19,tracking_margin_m=.4)
    old=occlusion_limit(reference,objects,(0.,0.,0.),morai_sonata_collision_envelope(),**kwargs)
    native=occlusion_limit(reference,objects,(0.,0.,0.),morai_sonata_collision_envelope(),native_geometry=True,**kwargs)
    assert native['limited']==old['limited']
    assert native['speed_cap_kmh']==pytest.approx(old['speed_cap_kmh'])
    assert native['visible_distance_m']==old['visible_distance_m']
