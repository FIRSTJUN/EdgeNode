"""Nonzero-boundary continuation from the 23.480 s live-bag failure."""
import gzip
import json
import math
import struct
from pathlib import Path
from unittest.mock import patch

import pytest

from edgenode_planning.adjacent_lane_planner import WorldTransition
from edgenode_planning.corridor_scene import planner_from_scene
from edgenode_planning.lane_geometry import arc, corridor_covers
from test_adjacent_lane_planner import network, plan


@pytest.fixture(scope='module')
def continuation_scene():
    path=Path(__file__).parent/'fixtures/s4_entry_fix_live_01_continuation.json.gz'
    return json.loads(gzip.decompress(path.read_bytes()))


def candidate_context(scene):
    planner=planner_from_scene(scene)
    inputs=scene['inputs']
    pair=planner._lane_pair
    ego_s=pair.original_index.project(inputs['current_position'])
    reference_arc=arc(inputs['reference_points'])
    ego_local=planner._project_reference(inputs['reference_points'],reference_arc,inputs['current_position'])[0]
    return planner,dict(pair=pair,position=inputs['current_position'],yaw=inputs['current_heading'],
        ego_s=ego_s,progress=inputs['road_progress_m'],
        front_s=pair.original_index.project(planner._front_point)-ego_s,
        rear_s=pair.original_index.project(planner._rear_point)-ego_s,
        obstacles=inputs['obstacles'],speed_kmh=inputs['speed_kmh'],
        horizon=reference_arc[-1]-ego_local,recovery=False)


def test_fixture_preserves_the_original_longer_only_blocked_result(continuation_scene):
    previous=continuation_scene['result']
    assert previous['status']=='BLOCKED'
    assert previous['blocked_reason']=='NO_SAFE_FEASIBLE_CANDIDATE'
    assert previous['evaluated_candidate_reject_reason']=='DRIVABLE_CORRIDOR'
    assert previous['entry_length']==pytest.approx(1.7262131149722362)
    assert struct.unpack('f',struct.pack('f',previous['entry_length']))[0]==continuation_scene[
        'provenance']['recorded_entry_length_float32_m']
    assert previous['maximum_curvature']<previous['usable_curvature_limit']
    evaluation=previous['corridor_evaluation']
    assert not corridor_covers(evaluation['points'],evaluation['quads'])
    assert previous['corridor_diagnostics']['failed_index']==6
    assert previous['corridor_diagnostics']['max_violation_m']==pytest.approx(8.059661790770394e-5)


def test_live_failure_frame_selects_a_safe_shorter_continuation(continuation_scene):
    candidate_planner,options=candidate_context(continuation_scene)
    candidate=candidate_planner._candidate(**options)
    assert candidate['feasible']
    assert candidate['road_safe']
    assert candidate['reject_reason']==''
    planner=planner_from_scene(continuation_scene)
    original=planner._world_transition
    completion=planner._entry_completion_progress
    inputs=continuation_scene['inputs']
    delta=original.project_parameter(inputs['current_position'])
    _,first,second=original.state(delta)
    rotation=inputs['current_heading']-math.atan2(first[1],first[0])
    def rotate(v):
        return [math.cos(rotation)*v[0]-math.sin(rotation)*v[1],
                math.sin(rotation)*v[0]+math.cos(rotation)*v[1]]
    preferred=planner._transition_end_station-planner._lane_pair.original_index.project(inputs['current_position'])
    result=planner.plan(**inputs)
    assert result['status']=='AVOIDING'
    assert result['lane_change_phase']=='ENTRY'
    assert result['evaluated_candidate_reject_reason']==''
    assert max(.25,preferred/2)<=result['entry_length']<preferred
    assert result['drivable_safe']
    assert result['maximum_curvature']<=planner.usable_curvature_limit
    assert result['minimum_clearance_m']>=planner.required_clearance_m
    assert result['entry_length']==pytest.approx(candidate['transition_length'])
    assert planner._entry_completion_progress==completion
    start,actual_first,actual_second=planner._world_transition.state(0.)
    assert start==pytest.approx(inputs['current_position'])
    assert actual_first==pytest.approx(rotate(first),abs=1e-10)
    assert actual_second==pytest.approx(rotate(second),abs=1e-10)
    projected=[planner._lane_pair.original_index.project(p) for p in result['points']]
    assert all(b>=a-1e-6 for a,b in zip(projected,projected[1:]))
    tangent=planner._lane_pair.at(projected[0])['original'][1]
    for i in range(101):
        velocity=planner._world_transition.state(planner._world_transition.length*i/100)[1]
        assert sum(a*b for a,b in zip(velocity,tangent))>0.
    # The known 1.25 m solution also passes, but the search keeps a feasible
    # endpoint closer to the previously approved one rather than hardcoding it.
    alternative_planner,options=candidate_context(continuation_scene)
    alternative_planner._transition_end_station=options['ego_s']+1.25
    alternative=alternative_planner._candidate(**options)
    assert alternative['feasible']
    assert alternative['transition_length']==pytest.approx(1.25)
    assert abs(result['entry_length']-preferred)<abs(alternative['transition_length']-preferred)


def test_preferred_safe_continuation_is_tried_once_and_kept(continuation_scene):
    planner,options=candidate_context(continuation_scene)
    previous=planner._world_transition
    delta=previous.project_parameter(options['position'])
    position,velocity,_=previous.state(delta)
    options.update(position=position,yaw=math.atan2(velocity[1],velocity[0]),
                   ego_s=planner._lane_pair.original_index.project(position))
    preferred=planner._transition_end_station-options['ego_s']
    with patch('edgenode_planning.adjacent_lane_planner.WorldTransition',wraps=WorldTransition) as constructor:
        result=planner._candidate(**options)
    assert result['feasible']
    assert result['transition_length']==pytest.approx(preferred)
    assert constructor.call_count==1


@pytest.mark.parametrize('bound', ['front','horizon'])
def test_bidirectional_search_is_bounded_and_prefers_nearby_candidates(continuation_scene,bound):
    planner,options=candidate_context(continuation_scene)
    preferred=planner._transition_end_station-options['ego_s']
    if bound=='front':options['front_s']=1.6
    else:options['horizon']=1.6
    # A point at Ego rejects every candidate, exposing the entire real search
    # without replacing any safety predicate or changing a vehicle limit.
    options['obstacles']=options['obstacles']+[options['position']]
    with patch('edgenode_planning.adjacent_lane_planner.WorldTransition',wraps=WorldTransition) as constructor:
        result=planner._candidate(**options)
    lengths=[call.args[-1] for call in constructor.call_args_list]
    assert not result['feasible'] and 'CLEARANCE' in result['reject_reason']
    family_size=len(set(lengths))
    assert 25<=family_size<=27
    assert len(lengths)==4*family_size
    assert all(lengths[i:i+family_size]==lengths[:family_size]
               for i in range(0,len(lengths),family_size))
    assert lengths[0]==pytest.approx(preferred)
    assert min(lengths)==pytest.approx(max(.25,preferred/2))
    assert max(lengths)<=options['horizon']
    assert max(lengths)<options['front_s']
    distances=[abs(length-preferred) for length in lengths]
    assert all(distances[i:i+family_size]==sorted(distances[i:i+family_size])
               for i in range(0,len(distances),family_size))


@pytest.mark.parametrize('bound', ['front','horizon'])
def test_shorter_search_does_not_bypass_an_unreachable_entry_deadline(continuation_scene,bound):
    planner,options=candidate_context(continuation_scene)
    preferred=planner._transition_end_station-options['ego_s']
    options['front_s' if bound=='front' else 'horizon']=preferred-.01
    result=planner._candidate(**options)
    assert not result['feasible']
    assert 'ENTRY_DEADLINE' in result['reject_reason']
    assert result['transition_length']<options['front_s']
    assert result['transition_length']<=options['horizon']


def test_too_short_curves_still_fail_the_unchanged_curvature_gate(continuation_scene):
    planner,options=candidate_context(continuation_scene)
    planner._transition_end_station=options['ego_s']+.5
    options['horizon']=.5
    with patch('edgenode_planning.adjacent_lane_planner.WorldTransition',wraps=WorldTransition) as constructor:
        result=planner._candidate(**options)
    lengths=[call.args[-1] for call in constructor.call_args_list]
    assert min(lengths)>=.25 and max(lengths)<=.5
    assert not result['feasible']
    assert result['curvature']>planner.usable_curvature_limit
    assert 'CURVATURE' in result['reject_reason']


def test_new_entry_keeps_the_existing_speed_dependent_duration_policy():
    from edgenode_planning.adjacent_lane_planner import AdjacentLanePlanner
    planner=AdjacentLanePlanner(network(('right',)))
    result=plan(planner,speed=20.)
    assert result['status']=='AVOIDING'
    assert result['entry_length']>=20./3.6*planner.min_transition_time_sec


@pytest.mark.parametrize('fault', ['outside_corridor','reversed_heading'])
def test_shorter_search_still_rejects_invalid_corridor_or_direction(continuation_scene,fault):
    planner,options=candidate_context(continuation_scene)
    if fault=='outside_corridor':
        point,tangent=planner._lane_pair.at(options['ego_s'])['adjacent']
        options['position']=[point[0]+.01*tangent[1],point[1]-.01*tangent[0]]
    else:
        options['yaw']+=math.pi
    result=planner._candidate(**options)
    assert not result['feasible']
    assert ('DRIVABLE_CORRIDOR' if fault=='outside_corridor' else 'DIRECTION_CONTINUITY') in result['reject_reason']
