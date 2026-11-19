"""Frozen 4c80691 results, including numeric ties/holes, not timing thresholds."""
from dataclasses import replace
import gzip
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

from edgenode_planning.avoidance_geometry import polygon_distance
from edgenode_planning.lane_geometry import arc, sample
from edgenode_planning.lattice_path import reference_cost
from edgenode_planning.static_obstacle_planner import StaticObstaclePlanner
from avoidance_world import world


@pytest.fixture(scope='module')
def golden():
    file=Path(__file__).parent/'fixtures/s4_stage5c_geometry_golden.json.gz'
    result=json.loads(gzip.decompress(file.read_bytes()))
    assert result['baseline_commit']=='4c80691'
    assert result['data_kind']=='SYNTHETIC_TEST_ONLY_NOT_RUNTIME'
    return result






def test_distance_reuses_constants_without_changing_collision_or_minimum_distance(golden):
    for i,row in enumerate(golden['distances']):
        assert polygon_distance(tuple(map(tuple,row['a'])),tuple(map(tuple,row['b'])))==row['expected'],i


def test_normal_hazard_geometry_reuse_preserves_live_obstacles_and_body_changes():
    network,point,obstacle=world(radius=80.)
    planner=StaticObstaclePlanner(network)
    path=tuple(point(i*.5,0.) for i in range(100))
    groups=(tuple(obstacle(20.,24.)),)
    original=planner._body_geometry
    with patch.object(planner,'_body_geometry',wraps=original) as geometry:
        expected=planner._normal_hazards(path,groups)
        clearance=planner._min_clearance;count=geometry.call_count
        assert expected and count==len(path)-1
        planner._min_clearance=math.inf
        assert planner._normal_hazards(path,groups)==expected
        assert planner._min_clearance==clearance and geometry.call_count==count
        # Cached vehicle geometry never caches the obstacle decision.
        far=(tuple((x+100.,y+100.) for x,y in groups[0]),)
        assert not planner._normal_hazards(path,far)
        changed=path[:20]+(point(10.,.1),)+path[21:]
        planner._normal_hazards(changed,groups)
        assert geometry.call_count>count
        count=geometry.call_count
        planner.body=replace(planner.body,width_m=planner.body.width_m+.2)
        planner._normal_hazards(path,groups)
        assert geometry.call_count==count+len(path)-1


@pytest.mark.parametrize('width',[.4,1.86,3.8])
def test_normal_hazard_blocks_preserve_v15_hits_and_exact_clearance_at_both_floors(width):
    from edgenode_planning.lattice_path import poses
    network,point,obstacle=world()
    path=tuple(point(i*.5,0.) for i in range(140))
    # Include equality and either side of the body and reference floors, far
    # returns, and obstacles extending beyond a block's segment endpoints.
    body_floor=width/2+.25
    offsets=(-20.,-.900000001,-.9,-.899999999,0.,.899999999,.9,.900000001,
             body_floor-1e-9,body_floor,body_floor+1e-9,20.)
    groups=tuple(tuple(obstacle(7.+i*3.,9.+i*3.,low=y,high=y+.05)) for i,y in enumerate(offsets))
    expected=StaticObstaclePlanner(network);actual=StaticObstaclePlanner(network)
    expected.body=actual.body=replace(actual.body,width_m=width)
    path_poses=poses(path)
    segments=tuple((a,b,expected._body_geometry(a,b)) for a,b in zip(path_poses,path_poses[1:]))
    # Frozen v15 exhaustive scan is the oracle for this broad-phase change.
    hits=tuple(o for o in groups if any(expected._obstacle_safety(a,b,(o,),prepared=g)
        in ('BODY_COLLISION','REFERENCE_CLEARANCE') for a,b,g in segments))
    assert actual._normal_hazards(path,groups)==hits
    assert actual._min_clearance==expected._min_clearance


def test_binary_station_lookup_preserves_baseline_eps_ties_and_tiny_segments(golden):
    for path_id,row in enumerate(golden['sampling']):
        points=row['points'];lengths=arc(points)
        for query in row['queries']:
            result=sample(points,query['s'],lengths)
            actual=None if result is None else [list(result[0]),list(result[1])]
            assert actual==query['expected'],(path_id,query['s'])


@pytest.mark.parametrize('change',['path','obstacles','body','config','map_object'])
def test_body_cache_inputs_invalidate_and_clearance_is_reapplied(change):
    network, point, obstacle =world();planner=StaticObstaclePlanner(network)
    points=tuple((i*.25,0.,0.) for i in range(17));objects=(((2.,2.),(3.,2.),(3.,3.),(2.,3.)),)
    cache={}
    with patch.object(planner,'_body_safety',wraps=planner._body_safety) as checks:
        assert planner._path_safety(points,objects,3.,body_cache=cache)==''
        original_count=checks.call_count;clearance=planner._min_clearance
        planner._min_clearance=math.inf
        assert planner._path_safety(points,objects,3.,body_cache=cache)==''
        assert checks.call_count==original_count and planner._min_clearance==clearance
        if change=='path':points=tuple((p[0],9.,0.) for p in points)
        elif change=='obstacles':objects=(((2.,-.5),(3.,-.5),(3.,.5),(2.,.5)),)
        elif change=='body':planner.body=replace(planner.body,width_m=3.8)
        # This object is over 1 m from the corrected straight body sweep.
        # Raise the margin past the actual clearance to exercise rejection.
        elif change=='config':planner.config=replace(planner.config,obstacle_margin_m=1.5,hold_clearance_m=1.5)
        else:planner.network=world()[0]
        reason=planner._path_safety(points,objects,3.,body_cache=cache)
        assert checks.call_count>original_count
        assert (reason=='') is (change in ('path','map_object'))


def test_projection_cache_tracks_geometry_edits_without_stale_costs():
    reference=[(float(i),0.,0.) for i in range(40)]
    adjacent=[(float(i),3.5,0.) for i in range(40)]
    points=[(float(i),1.,0.) for i in range(20)]
    cache={}
    assert reference_cost(points,reference,adjacent,projection_cache=cache)==reference_cost(points,reference,adjacent)
    adjacent[:]=[(p[0],-3.5,0.) for p in adjacent]
    assert reference_cost(points,reference,adjacent,projection_cache=cache)==reference_cost(points,reference,adjacent)
    reference[:]=[(p[0],.25,0.) for p in reference]
    assert reference_cost(points,reference,adjacent,projection_cache=cache)==reference_cost(points,reference,adjacent)
