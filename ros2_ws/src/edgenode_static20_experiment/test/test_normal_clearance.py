import math
from types import SimpleNamespace

from shapely.geometry import LineString,Polygon
from shapely.ops import unary_union

from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope
from edgenode_static20_experiment.normal_clearance import (
    candidate_path,centered_reference,clearance_reason,extend_normal_cache)


class Boundaries:
    def __init__(self,extra=()):
        self.geometry=unary_union([LineString([(0,1.5),(20,1.15),(60,1.15),(80,1.5)]),
            LineString([(0,-2),(80,-2)]),*extra])
    def swept_reason(self,polygon):
        return 'FORBIDDEN_OR_PHYSICAL_BOUNDARY' if Polygon(polygon).intersects(self.geometry) else ''


class Road:
    source='synthetic measured road fixture'
    def __init__(self,shape=None):self.shape=shape if shape is not None else Polygon([(-5,-2),(85,-2),(85,1.5),(-5,1.5)])
    def covers(self,polygon):return self.shape.covers(Polygon(polygon))


POINTS=tuple((float(x),0.,0.) for x in range(81))
BODY=morai_sonata_collision_envelope()


def test_map_only_centering_preserves_endpoint_and_proves_whole_body_tracking_margin():
    b=Boundaries();road=Road()
    assert clearance_reason(POINTS,b,road,BODY,.15)
    path,certificate=centered_reference(POINTS,b,road,BODY)
    assert path and path[0]==POINTS[0] and path[-1]==POINTS[-1]
    assert certificate['actor_data_used'] is False
    assert certificate['full_link_swept_body_proved']
    assert certificate['tracking_margin_m']==.15
    assert not clearance_reason(path,b,road,BODY,.15)
    assert abs(math.atan2(path[1][1]-path[0][1],path[1][0]-path[0][0]))<.001


def test_no_road_hole_or_unobserved_surface_can_be_filled_by_centering():
    road=Road(Polygon([(-5,-2),(85,-2),(85,1.5),(-5,1.5)],
        holes=[[(39,-2),(41,-2),(41,1.5),(39,1.5)]]).buffer(0))
    path,_=centered_reference(POINTS,Boundaries(),road,BODY)
    assert path is None
    assert centered_reference(POINTS,Boundaries(),None,BODY)==(None,'MEASURED_ROAD_UNAVAILABLE')


def test_physical_barrier_still_blocks_even_with_road_mask_on_both_sides():
    path,_=centered_reference(POINTS,Boundaries([LineString([(40,-3),(40,3)])]),Road(),BODY)
    assert path is None


def test_vehicle_steering_bound_cannot_be_bypassed_by_reference_correction():
    path,_=centered_reference(POINTS,Boundaries(),Road(),BODY,steering_curvature_limit=1e-8)
    assert path is None


def test_existing_successful_a_cache_is_preserved_and_only_optional_failed_entry_replaced():
    b=Boundaries();road=Road();original=(POINTS,dict(max_center_shift_m=.3))
    cache={('existing',BODY,.15):original,('tight',BODY,.15):None}
    network=SimpleNamespace(_normal_center_cache=cache,boundaries=b,road_corridor=road,
        links={k:dict(link_type='6',points=POINTS) for k in ('existing','tight')})
    audit=extend_normal_cache(network,BODY)
    assert cache[('existing',BODY,.15)] is original
    assert cache[('tight',BODY,.15)][0] is not None
    assert len(audit)==1 and audit[0]['selected']
