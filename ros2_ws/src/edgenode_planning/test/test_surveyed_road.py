from types import SimpleNamespace
import math
import pytest
from edgenode_planning.surveyed_road import centered_path
from edgenode_planning.lane_geometry import PolylineIndex
from edgenode_planning.road_corridor import VerifiedRoadCorridor
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope


def test_centering_keeps_endpoints_and_entire_body_inside_actual_unequal_sides():
    points=[(float(x),0.,0.) for x in range(201)]
    boundaries=SimpleNamespace(cells={},forbidden=[])
    network=SimpleNamespace(links={'L':{'points':points}})
    ring=((0.,-2.3),(200.,-2.3),(200.,1.5),(0.,1.5))
    road=VerifiedRoadCorridor([ring],source='synthetic surveyed edges',link_ids=['L'],allowed_sides=['right'])
    item=dict(link='L',low=0.,high=200.,left=PolylineIndex([(0.,1.5,0.),(200.,1.5,0.)]),
        right=PolylineIndex([(0.,-2.3,0.),(200.,-2.3,0.)]),road=road,source=road.source,marking_id='left',curb_id='right')
    path,certificate=centered_path(boundaries,network,morai_sonata_collision_envelope(),item)
    assert path[0]==points[0] and path[-1]==points[-1]
    assert min(p[1] for p in path)==pytest.approx(-.4,abs=.001)
    assert certificate['unchanged_caps_are_not_reclassified']
    # A centre point can fit while its body cannot: preserve the body gate.
    narrow=VerifiedRoadCorridor([((0.,-.5),(200.,-.5),(200.,.5),(0.,.5))],source='narrow measured road',link_ids=['L'],allowed_sides=['right'])
    with pytest.raises(ValueError,match='body outside'):
        centered_path(boundaries,network,morai_sonata_collision_envelope(),dict(item,road=narrow))
