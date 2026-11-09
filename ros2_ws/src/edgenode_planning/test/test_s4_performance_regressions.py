"""Frozen 4c80691 results, including numeric ties/holes, not timing thresholds."""
from dataclasses import replace
import gzip
import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

from edgenode_planning.avoidance_geometry import DrivableArea, _half_plane, _split_half_plane, polygon_distance
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


@pytest.mark.parametrize('group',range(7))
def test_spatial_index_and_joint_clipping_preserve_every_baseline_coverage_decision(golden,group):
    fixture=golden['coverage'][group]
    area=DrivableArea(tuple(tuple(tuple(p) for p in c) for c in fixture['cells']),'synthetic golden')
    for i,query in enumerate(fixture['queries']):
        polygon=tuple(tuple(p) for p in query['polygon'])
        assert area.covers(polygon) is query['expected'],(group,i,query)


def test_both_clipping_sides_are_exact_including_touches_and_point_order(golden):
    for i,row in enumerate(golden['clipping']):
        polygon=tuple(tuple(p) for p in row['polygon'])
        inside,outside=_split_half_plane(polygon,row['a'],row['b'])
        assert list(map(list,inside))==row['inside'],i
        assert list(map(list,outside))==row['outside'],i
        assert _half_plane(polygon,row['a'],row['b'],True)==inside
        assert _half_plane(polygon,row['a'],row['b'],False)==outside


def test_distance_reuses_constants_without_changing_collision_or_minimum_distance(golden):
    for i,row in enumerate(golden['distances']):
        assert polygon_distance(tuple(map(tuple,row['a'])),tuple(map(tuple,row['b'])))==row['expected'],i


def test_binary_station_lookup_preserves_baseline_eps_ties_and_tiny_segments(golden):
    for path_id,row in enumerate(golden['sampling']):
        points=row['points'];lengths=arc(points)
        for query in row['queries']:
            result=sample(points,query['s'],lengths)
            actual=None if result is None else [list(result[0]),list(result[1])]
            assert actual==query['expected'],(path_id,query['s'])


@pytest.mark.parametrize('change',['path','obstacles','body','config','boundary','map_object'])
def test_body_cache_inputs_invalidate_and_clearance_is_reapplied(change):
    network,area,point,obstacle=world();planner=StaticObstaclePlanner(network,drivable_area=area)
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
        elif change=='config':planner.config=replace(planner.config,obstacle_margin_m=1.)
        elif change=='boundary':planner.drivable_area=DrivableArea((((-10.,-1.),(140.,-1.),(140.,1.),(-10.,1.)),),'different synthetic boundary')
        else:planner.network=world()[0]
        reason=planner._path_safety(points,objects,3.,body_cache=cache)
        assert checks.call_count>original_count
        assert (reason=='') is (change=='map_object')


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


def test_replacing_boundary_rechecks_whole_future_reservation_before_short_rollout():
    network,area,point,obstacle=world();planner=StaticObstaclePlanner(network,drivable_area=area)
    normal=[point(float(x),0.) for x in range(21)];preview=[point(float(x),0.) for x in range(81)]
    kwargs=dict(speed_kmh=3.,current_link_id='O0',route_link_ids=['O0','O1'],hazard_reference=preview)
    first=planner.plan(normal,[obstacle()],(0.,0.,0.),**kwargs)
    assert first['state']=='AVOID_ENTRY' and planner._validated_paths
    planner.drivable_area=DrivableArea((((-10.,-1.75),(9.,-1.75),(9.,5.25),(-10.,5.25)),
                                      ((11.,-1.75),(140.,-1.75),(140.,5.25),(11.,5.25))),
                                     'new version with future road hole')
    second=planner.plan(normal,[obstacle()],(0.,0.,0.),**kwargs)
    assert second['blocked'] and second['reason']=='DRIVABLE_AREA'
