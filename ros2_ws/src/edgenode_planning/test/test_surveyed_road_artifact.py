import hashlib,json,pickle
from types import SimpleNamespace
from unittest.mock import patch
import pytest

from edgenode_planning.road_corridor import VerifiedRoadCorridor
from edgenode_planning.surveyed_road_artifact import load_artifact
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope
from edgenode_planning.hdmap_boundaries import HDMapBoundaries


def fixture(tmp_path,center_y=-3.):
    outer=((0.,-5.),(20.,-5.),(20.,5.),(0.,5.));hole=((8.,-.5),(10.,-.5),(10.,.5),(8.,.5))
    base=VerifiedRoadCorridor([outer],source='observed-test-road',link_ids={'N'},allowed_sides={'left','right'},exact_union_backend=True)
    boundaries=HDMapBoundaries.__new__(HDMapBoundaries)
    boundaries.directory=tmp_path;boundaries.origin=(0.,0.,0.);boundaries.forbidden=[];boundaries.cells={}
    network=SimpleNamespace(road_corridor=base,boundaries=boundaries,
        links={'N':dict(points=[(0.,center_y,0.),(20.,center_y,0.)],lazy_init=False)})
    raw=tmp_path/'original-map';raw.write_text('immutable source map')
    data=dict(schema='edgenode.surveyed-local-road.v2',actor_data_used=False,
        map_files_sha256={str(raw):hashlib.sha256(raw.read_bytes()).hexdigest()},
        patches=[dict(ring=outer,source_ids=['L','R'],holes=[])],
        negative_islands=[dict(ring=hole,source_ids=['I'],driving_center_intersections=0)])
    file=tmp_path/'proof.json';file.write_text(json.dumps(data));digest=hashlib.sha256(file.read_bytes()).hexdigest()
    def feature(key,points):return dict(attributes=dict(ID=key,Type='4',LowHigh='2'),parts=[tuple((*p,0.) for p in points)])
    features=[feature('L',[(0.,5.),(20.,5.)]),feature('R',[(0.,-5.),(20.,-5.)]),feature('I',hole+(hole[0],))]
    reader=lambda path:features if path.name=='C3_VEHICLEPROTECTIONSAFETY' else []
    return network,file,digest,reader,raw


def test_surveyed_island_remains_excluded_from_overlapping_positive_surfaces_and_pickle(tmp_path):
    network,file,digest,reader,_=fixture(tmp_path)
    with patch('edgenode_planning.hdmap_audit.read_layer',side_effect=reader):road=load_artifact(file,digest,network)
    body=morai_sonata_collision_envelope()
    for candidate in [road,pickle.loads(pickle.dumps(road))]:
        assert candidate.covers(body.polygon(3.,0.,0.))
        assert not candidate.covers(body.polygon(8.,0.,0.))
        assert not any(low<=0<=high for low,high in candidate.cross_section((9.,0.),(0.,1.)))


def test_a_closed_boundary_with_an_actual_driving_center_is_not_declared_an_island(tmp_path):
    network,file,digest,reader,_=fixture(tmp_path,center_y=0.)
    with patch('edgenode_planning.hdmap_audit.read_layer',side_effect=reader):
        with pytest.raises(ValueError,match='native driving center'):load_artifact(file,digest,network)


def test_original_map_and_artifact_pins_are_both_enforced(tmp_path):
    network,file,digest,reader,raw=fixture(tmp_path)
    with pytest.raises(ValueError,match='pin mismatch'):load_artifact(file,'0'*64,network)
    raw.write_text('changed source')
    with pytest.raises(ValueError,match='input map changed'):load_artifact(file,digest,network)


def test_adjacent_positive_surfaces_never_erase_an_interior_physical_curb(tmp_path):
    network,file,_,reader,_=fixture(tmp_path)
    data=json.loads(file.read_text());data['negative_islands']=[]
    data['patches']=[dict(ring=[(0.,-5.),(10.,-5.),(10.,5.),(0.,5.)],source_ids=['L','R','B']),
                     dict(ring=[(10.,-5.),(20.,-5.),(20.,5.),(10.,5.)],source_ids=['L','R','B'])]
    file.write_text(json.dumps(data));digest=hashlib.sha256(file.read_bytes()).hexdigest()
    original=reader(network.boundaries.directory/'C3_VEHICLEPROTECTIONSAFETY')[:2]
    barrier=dict(attributes=dict(ID='B',Type='4',LowHigh='2'),parts=[((10.,-5.,0.),(10.,5.,0.))])
    with patch('edgenode_planning.hdmap_audit.read_layer',side_effect=lambda p:original+[barrier] if p.name=='C3_VEHICLEPROTECTIONSAFETY' else []):
        road=load_artifact(file,digest,network)
    polygon=morai_sonata_collision_envelope().polygon(8.,0.,0.)
    assert road.covers(polygon)
    assert network.boundaries.swept_reason(polygon)=='PHYSICAL_BOUNDARY_COLLISION:B'
    assert pickle.loads(pickle.dumps(network.boundaries)).swept_reason(polygon)=='PHYSICAL_BOUNDARY_COLLISION:B'
