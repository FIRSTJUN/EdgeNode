"""Route/ROI fallback policy, actual detector schema, and K-City catalog wiring."""
import copy
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.route_stopline_fallback import MODE, associations_for_route
from edgenode_planning.traffic_light_gate import TrafficLightGate
from edgenode_planning.dijkstra_planner import DijkstraPlanner

WAYPOINTS=['waypoint1','waypoint2']
ROUTE=dict(points=[[0.,0.,0.],[200.,0.,0.]],link_ids=['link'],
    node_ids=['start',*WAYPOINTS],goal_node_id=WAYPOINTS[-1])


def config():
    return dict(schema=3,mode=MODE,fallback_enabled=True,verified=False,route_node_ids=WAYPOINTS,
        stop_policy=dict(braking_verified=False,deceleration_mps2=1.5,reaction_sec=1.,reference_stop_margin_m=5.,approach_m=100.),
        ego_roi=dict(camera_topic='/camera/image/compressed',camera_frame_id='Camera-1',image_size=[640,480],roi_xyxy_norm=[.45,0.,.70,.45],min_confidence=.25),
        stop_lines=[dict(stop_line_id='test_line_100',stop_line_xy=[[100.,-4.],[100.,4.]],reference_route_heading_rad=0.,approach_link_ids=['link'],related_nodes=[],map_signal_candidates=[])])


def sample(now,color='red',box=None):
    x,y,w,h=box or [350.,15.,20.,20.]
    return json.dumps(dict(state=color.upper(),stamp=dict(sec=now//10**9,nanosec=now%10**9),frame_id='Camera-1',
        image_width=640,image_height=480,prediction_age_sec=.02,
        predictions=[dict(label=color,confidence=.4,x=x,y=y,w=w,h=h)]))


def tick(g,now,color='red',progress=95.,speed=0.,box=None,route=ROUTE,waypoints=WAYPOINTS):
    g.observe_detector(sample(now,color,box),now)
    return g.decision(True,route,progress,speed,now,route_node_ids=waypoints)


def confirm(g,color='red',progress=95.,start=1_000_000_000,speed=0.,**kwargs):
    for i in range(3):r=tick(g,start+i*150_000_000,color,progress,speed,**kwargs)
    return r


def test_active_route_red_after_three_frames_stops_before_line():
    g=TrafficLightGate(config())
    assert not tick(g,1_000_000_000)['hold']
    assert not tick(g,1_150_000_000)['hold']
    r=tick(g,1_300_000_000)
    assert r['hold'] and r['speed_cap_kmh']==0.
    assert r['reason']=='RED_ACTIVE_STOPLINE' and r['stop_line_id']=='test_line_100'
    assert r['association_id']==MODE and r['fallback_used']
    assert not r['route_association_verified'] and r['signal_id'] is None
    assert r['distance_to_stopline_m']==5. and r['distance_to_stop_m']==0.


def test_early_red_envelope_is_positive_not_full_stop():
    r=confirm(TrafficLightGate(config()),progress=50.,speed=3.)
    assert not r['hold'] and 0<r['speed_cap_kmh']<50.


def test_far_red_never_zeroes_target():
    c=config();c['stop_policy']['approach_m']=50.
    r=confirm(TrafficLightGate(c),progress=0.)
    assert r['speed_cap_kmh'] is None and r['reason']=='BEFORE_VERIFIED_APPROACH'


@pytest.mark.parametrize('change',[dict(approach_link_ids=['other']),dict(reference_route_heading_rad=math.pi),dict(stop_line_xy=[[100.,20.],[100.,30.]])])
def test_unrelated_or_opposite_stopline_has_no_global_stop(change):
    c=config();c['stop_lines'][0].update(change)
    r=confirm(TrafficLightGate(c))
    assert r['speed_cap_kmh'] is None and not r['hold']
    assert r['reason']=='NO_FORWARD_ROUTE_STOPLINE_CROSSINGS'


def test_passed_line_never_reactivated_even_after_small_backwards_jitter():
    g=TrafficLightGate(config())
    assert tick(g,1_000_000_000,progress=101.)['speed_cap_kmh'] is None
    assert tick(g,1_150_000_000,progress=100.8)['speed_cap_kmh'] is None


def test_next_line_changes_with_progress_and_red_confirmation_resets():
    c=config();second=copy.deepcopy(c['stop_lines'][0]);second.update(stop_line_id='test_line_150',stop_line_xy=[[150.,-4.],[150.,4.]])
    c['stop_lines'].append(second);g=TrafficLightGate(c)
    assert confirm(g)['stop_line_id']=='test_line_100'
    r=tick(g,1_500_000_000,progress=100.,speed=25.)
    assert r['stop_line_id']=='test_line_150' and r['distance_to_stopline_m']==50.
    assert r['speed_cap_kmh'] is None and r['red_frames']==1


def test_stable_green_releases_same_stop_and_unknown_expires_hold():
    g=TrafficLightGate(config());assert confirm(g)['hold']
    assert tick(g,1_450_000_000,'green')['hold']
    assert tick(g,1_600_000_000,'green')['hold']
    r=tick(g,1_750_000_000,'green');assert not r['hold'] and r['reason']=='GREEN_RELEASE'
    g=TrafficLightGate(config());confirm(g)
    assert tick(g,1_450_000_000,'unknown')['hold']
    r=tick(g,2_150_000_000,'unknown')
    assert not r['hold'] and r['speed_cap_kmh'] is None


@pytest.mark.parametrize('color',['unknown','yellow'])
def test_unknown_yellow_never_start_new_stop(color):
    assert confirm(TrafficLightGate(config()),color)['speed_cap_kmh'] is None


def test_other_lane_roi_detection_ignored_and_tied_heads_are_ambiguous():
    g=TrafficLightGate(config())
    assert confirm(g,box=[100.,15.,20.,20.])['speed_cap_kmh'] is None
    for i in range(3):
        now=2_000_000_000+i*150_000_000;p=json.loads(sample(now,box=[310.,15.,8.,8.]))
        p['predictions'].append(dict(label='red',confidence=.4,x=330.,y=15.,w=8.,h=8.))
        g.observe_detector(json.dumps(p),now)
        r=g.decision(True,ROUTE,95.,0.,now,route_node_ids=WAYPOINTS)
    assert r['evidence_reason']=='AMBIGUOUS_CANDIDATE_SCORES' and r['speed_cap_kmh'] is None


def test_missing_association_and_wrong_waypoints_do_not_global_brake():
    g=TrafficLightGate();r=tick(g,1_000_000_000)
    assert not r['hold'] and r['reason']=='ASSOCIATION_NOT_VERIFIED'
    r=confirm(TrafficLightGate(config()),waypoints=['other'])
    assert not r['hold'] and r['reason']=='FALLBACK_WAYPOINT_MISMATCH'


def test_arc_recomputed_when_route_start_changes():
    c=config();route=copy.deepcopy(ROUTE);route['points'][0][0]=20.
    entries=associations_for_route(c,route,WAYPOINTS)
    assert entries[0]['stop_arc_m']==80.


def test_real_kcity_catalog_matches_actual_mgeo_route_and_all_crossings():
    root=Path(__file__).resolve().parents[4]
    mgeo=root/'local_data/kcity_mgeo'
    if not mgeo.exists():pytest.skip('K-City local map not installed')
    c=json.loads((Path(__file__).resolve().parents[1]/'config/traffic_light_associations.kcity.json').read_text())
    planner=DijkstraPlanner(mgeo)
    route=planner.plan_route(c['reference_route_link_ids'][0],c['route_node_ids'])
    assert route is not None
    entries=sorted(associations_for_route(c,route,c['route_node_ids']),key=lambda a:a['stop_arc_m'])
    assert len(entries)==11
    assert [e['stop_line_id'] for e in entries]==[s['stop_line_id'] for s in c['stop_lines']]
    assert entries[0]['stop_line_id']=='B219BS010007'
    assert entries[-1]['stop_line_id']=='B219BS010013'
    # Every real entry can independently request a RED stop at its 5m margin.
    for a in entries:
        g=TrafficLightGate(c);progress=a['stop_arc_m']-5.
        for i in range(3):
            now=1_000_000_000+i*150_000_000;g.observe_detector(sample(now),now)
            r=g.decision(True,route,progress,0.,now,route_node_ids=c['route_node_ids'])
        assert r['stop_line_id']==a['stop_line_id'] and r['hold']
    # Current recorded end-of-route progress cannot pick any line behind it.
    r=TrafficLightGate(c).decision(True,route,894.1225892136654,0.,1_000_000_000,route_node_ids=c['route_node_ids'])
    assert r['speed_cap_kmh'] is None and r['reason']=='ROUTE_STOPLINES_PASSED'
