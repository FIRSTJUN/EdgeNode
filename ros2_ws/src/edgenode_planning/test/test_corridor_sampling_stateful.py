"""Exact map anchors, moving cap/grid phase, and recorded consecutive cycles."""
import gzip
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.lane_geometry import LanePair, corridor_covers
from s4_stateful_helpers import live_sequences, replay_recorded


def moving_scenes():
    p=Path(__file__).parent/'fixtures/s4_moving_envelope_scenes.json.gz'
    return json.loads(gzip.decompress(p.read_bytes()))


def lane_pair(scene):
    return LanePair(**{k:scene[k] for k in ('original','adjacent','ids','side','section_ends')})


@pytest.mark.parametrize('scene',moving_scenes(),ids=['NEW106','NEW118'])
def test_actual_inverted_sampling_scene_preserves_boundaries(scene):
    pair=lane_pair(scene)
    assert scene['before_failure']['reason']=='NONCONVEX_QUAD'
    mesh=pair.corridor_envelope(scene['start'],scene['end'])
    assert mesh['failure'] is None
    assert corridor_covers(scene['candidate'],mesh['quads'])
    # Every real vertex in the clipped boundaries remains an exact cell vertex.
    vertices={tuple(p) for q in mesh['quads'] for p in q}
    assert all(tuple(p[:2]) in vertices for s,p in zip(pair.original_arc,pair.original)
               if scene['start']<s<scene['end'])
    a,b=pair.at(scene['start'])['adjacent_s'],pair.at(scene['end'])['adjacent_s']
    assert all(tuple(p[:2]) in vertices for s,p in zip(pair.adjacent_arc,pair.adjacent) if a<s<b)
    outside=pair.at(scene['station'])['adjacent'][0][:2]
    tangent=pair.at(scene['station'])['adjacent'][1]
    outside=[outside[0]+.01*tangent[1],outside[1]-.01*tangent[0]]
    assert not corridor_covers([outside,outside],mesh['quads'])


def test_moving_horizon_caps_sweep_across_actual_map_corners():
    scene=moving_scenes()[0];pair=lane_pair(scene)
    # Millimetre phase steps plus exact/near-exact map vertices and caps.
    stations={16.25+i*.001 for i in range(1001)}
    stations.update(s-20.+delta for s in pair.original_arc for delta in (-1e-6,0.,1e-6)
                    if 16.25<s-20.<17.25)
    for s in sorted(stations):
        mesh=pair.corridor_envelope(s-.5,s+20.)
        assert mesh['failure'] is None,(s,mesh['failure'])
        assert all(b>a for a,b in zip(mesh['stations'],mesh['stations'][1:]))
        center=pair.suffix('adjacent',pair.at(s)['adjacent_s'],pair.at(s+20.)['adjacent_s'])
        assert corridor_covers(center,mesh['quads']),s


def test_new_90_through_130_preserves_state_without_sampling_stops():
    fixture=live_sequences();run=next(r for r in fixture['runs'] if r['name']=='NEW')
    _,history=replay_recorded(fixture,{**run,'frames':[f for f in run['frames'] if f['index']<=130]})
    relevant=[(f,r) for f,r in history if f['index']>=90]
    assert len(relevant)==41
    assert all(r['status']=='AVOIDING' for f,r in relevant),[(f['index'],r['blocked_reason']) for f,r in relevant if r['blocked']]
    assert all(r['drivable_safe'] for f,r in relevant)
