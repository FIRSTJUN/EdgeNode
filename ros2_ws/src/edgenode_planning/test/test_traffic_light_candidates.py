"""Multi-lane candidate selection and distinct-frame signal evidence."""
import json
import math

import pytest

from edgenode_planning.traffic_light_candidates import TrafficLightCandidateSelector
from edgenode_planning.traffic_light_gate import TrafficLightGate
from test_route_stopline_fallback import config, sample, ROUTE, WAYPOINTS


def box(x, color='red', confidence=.6, y=30., width=8.):
    return dict(class_name=color,confidence=confidence,box_xyxy=[x-width/2,y-4,x+width/2,y+4])


def selector():
    return TrafficLightCandidateSelector(),config()['ego_roi']


def tick(g,now,rows,*,progress=95.,ego_yaw=0.):
    p=json.loads(sample(now))
    p['predictions']=[dict(label=d['class_name'],confidence=d['confidence'],
        x=(d['box_xyxy'][0]+d['box_xyxy'][2])/2,y=(d['box_xyxy'][1]+d['box_xyxy'][3])/2,
        w=d['box_xyxy'][2]-d['box_xyxy'][0],h=d['box_xyxy'][3]-d['box_xyxy'][1]) for d in rows]
    g.observe_detector(json.dumps(p),now)
    return g.decision(True,ROUTE,progress,0.,now,route_node_ids=WAYPOINTS,ego_yaw=ego_yaw)


def test_three_red_heads_select_forward_center_and_stop():
    g=TrafficLightGate(config());rows=[box(420),box(320),box(370)]
    for i in range(3):
        r=tick(g,1_000_000_000+i*150_000_000,rows)
        if i<2:assert not r['stop_requested']
    assert r['stop_requested'] and r['hold'] and r['reason']=='RED_ACTIVE_STOPLINE'
    assert r['detection_count']==3 and r['candidate_count']==3
    assert r['selected_candidate']['center']==[320.,30.]
    assert r['selected_score']>r['second_best_score']
    assert r['ambiguity_margin']>=r['required_ambiguity_margin'] and not r['ambiguous']


def test_extreme_left_right_heads_cannot_win_despite_confidence_and_size():
    s,a=selector();state,d=s.select([box(80,confidence=1.,width=60),box(320,confidence=.25),box(600,confidence=1.,width=60)],a,1_000_000_000)
    assert state=='RED' and d['selected_candidate']['center'][0]==320
    assert d['candidate_count']==1
    assert d['candidates'][0]['rejection']=='OUTSIDE_EGO_ROI'
    assert d['candidates'][2]['rejection']=='OUTSIDE_EGO_ROI'


def test_equally_plausible_disjoint_heads_are_ambiguous():
    g=TrafficLightGate(config())
    for i in range(3):r=tick(g,1_000_000_000+i*150_000_000,[box(310),box(330)])
    assert r['ambiguous'] and r['selected_candidate'] is None
    assert r['ambiguity_margin']==pytest.approx(0.)
    assert r['reason']=='AMBIGUOUS_CANDIDATE_SCORES' and not r['stop_requested']


def test_tracking_holds_same_head_when_confident_adjacent_green_appears():
    g=TrafficLightGate(config())
    for i in range(3):r=tick(g,1_000_000_000+i*150_000_000,[box(320)])
    track=r['selected_track_id'];assert r['stop_requested']
    for i in range(3):
        r=tick(g,1_450_000_000+i*150_000_000,[box(345,'green',1.),box(322,'red',.3)])
        assert r['selected_candidate']['center'][0]==322
        assert r['selected_track_id']==track and r['stop_requested']
    # A color transition at the tracked head must still release RED normally.
    for i in range(3):
        r=tick(g,1_900_000_000+i*150_000_000,[box(322,'green',.6),box(420,'red',1.)])
    assert not r['stop_requested'] and r['reason']=='GREEN_RELEASE'


def test_near_stop_retention_bonus_is_stronger_than_distant_bonus():
    far,a=selector();near,_=selector()
    for s in (far,near):s.select([box(350)],a,1_000_000_000)
    rows=[box(350,confidence=.25),box(320,'green',1.)]
    _,d1=far.select(rows,a,1_150_000_000,near_stop=False)
    _,d2=near.select(rows,a,1_150_000_000,near_stop=True)
    assert d1['selected_candidate']['center']==d2['selected_candidate']['center']==[350.,30.]
    assert d2['selected_candidate']['retention_bonus']>d1['selected_candidate']['retention_bonus']
    assert d2['ambiguity_margin']>d1['ambiguity_margin']


def test_sudden_jump_rejected_then_new_target_needs_own_three_frames():
    g=TrafficLightGate(config());r=tick(g,1_000_000_000,[box(320)])
    original=r['selected_track_id']
    for now in (1_150_000_000,1_300_000_000):
        r=tick(g,now,[box(420)])
        assert not r['stop_requested'] and r['reason']=='TARGET_JUMP_REJECTED'
    r=tick(g,1_600_000_000,[box(420)])
    assert r['selected_track_id']!=original and r['red_frames']==1 and not r['stop_requested']
    assert not tick(g,1_750_000_000,[box(420)])['stop_requested']
    assert tick(g,1_900_000_000,[box(420)])['stop_requested']


def test_repeated_frame_does_not_confirm_red_or_refresh_track():
    g=TrafficLightGate(config());tick(g,1_000_000_000,[box(320)])
    for i in range(1,5):
        r=g.decision(True,ROUTE,95.,0.,1_000_000_000+i*100_000_000,route_node_ids=WAYPOINTS,ego_yaw=0.)
        assert r['red_frames']==1 and not r['stop_requested']
    assert g.candidate_selector.stamp==1_000_000_000


def test_route_direction_hint_changes_expected_image_position():
    g=TrafficLightGate(config())
    r=tick(g,1_000_000_000,[box(320),box(400)],ego_yaw=math.atan(.25))
    assert r['expected_center_x']==pytest.approx(400.)
    assert r['selected_candidate']['center'][0]==400.


def test_single_head_and_roi_border_center_keep_existing_debounce():
    g=TrafficLightGate(config())
    for i in range(3):r=tick(g,1_000_000_000+i*150_000_000,[box(350)])
    assert r['stop_requested'] and r['red_frames']==3
    s,a=selector();state,d=s.select([box(290,width=20.)],a,1_000_000_000)
    assert state=='RED' and d['selected_candidate']['center'][0]==290.


def test_duplicate_same_head_not_a_second_competitor():
    s,a=selector();_,d=s.select([box(320),box(320,confidence=.9)],a,1_000_000_000)
    assert d['candidate_count']==1 and d['second_best_score'] is None and not d['ambiguous']


def test_below_threshold_detection_cannot_win():
    s,a=selector();_,d=s.select([box(320,confidence=.24),box(350,confidence=.25)],a,1_000_000_000)
    assert d['selected_candidate']['center'][0]==350.
    assert d['candidates'][0]['rejection']=='BELOW_CONFIDENCE_OR_UNKNOWN_CLASS'


def test_bad_detection_diagnostics_are_json_serializable():
    s,a=selector()
    _,d=s.select([dict(class_name='red',confidence=float('nan'),box_xyxy=[310,20,330,40]),None,box(320)],a,1_000_000_000)
    json.dumps(d,allow_nan=False)
    assert d['selected_candidate']['center'][0]==320.
    assert d['detection_count']==3


def test_new_stopline_does_not_inherit_previous_bbox_or_debounce():
    import copy
    c=config();second=copy.deepcopy(c['stop_lines'][0])
    second.update(stop_line_id='test_line_150',stop_line_xy=[[150.,-4.],[150.,4.]])
    c['stop_lines'].append(second);g=TrafficLightGate(c)
    for i in range(3):tick(g,1_000_000_000+i*150_000_000,[box(320)])
    # Bounded pose step at the route crossing; next line gets a new selector.
    now=1_500_000_000;g.observe_detector(sample(now,box=[420.,30.,8.,8.]),now)
    r=g.decision(True,ROUTE,100.,25.,now,route_node_ids=WAYPOINTS,ego_yaw=0.)
    assert r['stop_line_id']=='test_line_150' and r['selected_candidate']['center'][0]==420.
    assert r['red_frames']==1 and not r['stop_requested']
