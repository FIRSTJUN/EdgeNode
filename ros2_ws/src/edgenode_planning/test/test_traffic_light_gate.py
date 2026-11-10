import copy
import pytest
from edgenode_planning.traffic_light_gate import TrafficLightGate,route_digest

ROUTE=dict(points=[[0.,0.,0.],[200.,0.,0.]],link_ids=['link'])


def gate():
    return TrafficLightGate(dict(verified=True,stop_policy=dict(braking_verified=True,deceleration_mps2=1.5,reaction_sec=1.),route_sha256=route_digest(ROUTE['points']),associations=[dict(
        signal_id='synthetic_signal',controlled_link_id='link',movement='straight',stop_arc_m=100.,front_offset_m=2.,
        stop_line_xy=[[100.,-4.],[100.,4.]],stop_line_binding_verified=True,front_offset_verified=True,camera_binding_verified=True,
        buffer_m=2.,clearance_m=6.,approach_m=50.,max_heading_error_deg=10.,roi_xyxy_norm=[.1,.1,.8,.8],
        image_size=[640,480],camera_frame_id='Camera-1',camera_topic='/camera/image/compressed')]))


def message(now,color='red'):
    return dict(schema=1,status='OK',source_stamp_ns=now,source_topic='/camera/image/compressed',frame_id='Camera-1',image_size=[640,480],
        detections=[dict(class_name=color,confidence=.9,box_xyxy=[100.,100.,130.,140.])])


def tick(g,now,color='red',progress=96.,speed=0.):
    g.observe(message(now,color),now)
    return g.decision(True,ROUTE,progress,speed,now)


def green(g,progress=96.,start=1_000_000_000):
    for i in range(4):result=tick(g,start+i*200_000_000,'green',progress)
    return result


def confirmed(g,color='red',progress=96.,start=1_000_000_000,speed=0.):
    for i in range(3):r=tick(g,start+i*200_000_000,color,progress,speed)
    return r


def test_disabled_and_missing_association_never_global_brake():
    g=TrafficLightGate()
    assert not g.decision(False,None,None,None,0)['hold']
    r=g.decision(True,ROUTE,96.,0.,1_000_000_000)
    assert not r['hold'] and r['speed_cap_kmh'] is None
    assert r['binding_failure_reason']=='ASSOCIATION_NOT_VERIFIED'


@pytest.mark.parametrize('color',['yellow','unknown_arrow','green'])
def test_non_red_never_starts_a_stop(color):
    r=confirmed(gate(),color)
    assert not r['hold'] and r['speed_cap_kmh'] is None


def test_red_needs_three_distinct_frames_and_time():
    g=gate();now=1_000_000_000
    assert not tick(g,now)['hold']
    for i in range(10):assert not g.decision(True,ROUTE,96.,0.,now+i*10_000_000)['hold']
    assert not tick(g,now+200_000_000)['hold']
    assert tick(g,now+400_000_000)['hold']


def test_one_or_two_red_frames_do_not_interrupt_green():
    g=gate();confirmed(g,'green')
    assert not tick(g,1_600_000_000,'red')['hold']
    assert not tick(g,1_800_000_000,'red')['hold']
    assert not tick(g,2_000_000_000,'green')['hold']


def test_confirmed_green_releases_red_and_unknown_only_briefly_holds():
    g=gate();assert confirmed(g)['hold']
    assert tick(g,1_600_000_000,'unknown')['hold']
    assert not g.decision(True,ROUTE,96.,0.,2_300_000_000)['hold']
    g=gate();confirmed(g)
    assert tick(g,1_600_000_000,'green')['hold']
    assert tick(g,1_800_000_000,'green')['hold']
    assert not tick(g,2_000_000_000,'green')['hold']


def test_red_approach_envelope_and_distant_signal():
    r=confirmed(gate(),progress=85.,speed=2.)
    assert not r['hold'] and 0<r['speed_cap_kmh']<50
    r=confirmed(gate(),progress=0.,speed=14.)
    assert r['reason']=='BEFORE_VERIFIED_APPROACH' and r['speed_cap_kmh'] is None


@pytest.mark.parametrize('progress',[98.,101.,110.])
def test_passed_stopline_never_requests_retroactive_brake(progress):
    g=gate();confirmed(g)
    # new gate simulates startup after line; existing gate simulates crossing.
    for g in (g,gate()):
        r=tick(g,1_600_000_000,'red',progress,speed=20.)
        assert not r['hold'] and r['speed_cap_kmh'] is None


def test_cleared_line_does_not_reactivate_with_small_pose_jitter():
    g=gate();tick(g,1_000_000_000,'red',98.)
    r=tick(g,1_200_000_000,'red',97.8)
    assert r['reason']=='VERIFIED_SIGNALS_CLEARED'


@pytest.mark.parametrize('change',[dict(frame_id='other'),dict(image_size=[1280,720]),dict(source_stamp_ns=0)])
def test_mismatched_evidence_never_starts_red(change):
    g=gate()
    for i in range(3):
        now=1_000_000_000+i*200_000_000;p=message(now);p.update(change);g.observe(p,now)
        r=g.decision(True,ROUTE,96.,0.,now)
        assert not r['hold'] and r['state']=='UNKNOWN'


def test_roi_excludes_other_signal_and_confidence_matches_detector():
    g=gate()
    for i in range(3):
        now=1_000_000_000+i*200_000_000;p=message(now);p['detections'][0]['box_xyxy']=[580,100,600,130];g.observe(p,now)
        assert g.decision(True,ROUTE,96.,0.,now)['speed_cap_kmh'] is None
    g=gate()
    for i in range(3):
        now=1_000_000_000+i*200_000_000;p=message(now);p['detections'][0]['confidence']=.25;g.observe(p,now)
        r=g.decision(True,ROUTE,96.,0.,now)
    assert r['hold']


@pytest.mark.parametrize('change',[dict(movement='left'),dict(stop_arc_m=300.),dict(controlled_link_id='other'),dict(front_offset_verified=False)])
def test_bad_binding_is_visible_without_global_stop(change):
    g=gate();g.config['associations'][0].update(change)
    r=tick(g,1_000_000_000)
    assert r['binding_failure_reason'] and not r['hold']


def test_route_mismatch_visible_and_next_line_selected():
    g=gate();route=copy.deepcopy(ROUTE);route['points'][1][0]=201.
    assert g.decision(True,route,96.,0.,1_000_000_000)['reason']=='ROUTE_BINDING_MISMATCH'
    g=gate();a=copy.deepcopy(g.config['associations'][0]);a.update(signal_id='next',stop_line_id='next_line',stop_arc_m=150.,stop_line_xy=[[150.,-4.],[150.,4.]])
    g.config['associations'].append(a)
    r=tick(g,1_000_000_000,progress=110.)
    assert r['signal_id']=='next' and r['distance_to_stopline_m']==40.


def test_detector_adapter_preserves_stamp_camera_roi_and_state_callback():
    import json
    g=gate();g.observe_state('RED',1_000_000_000)
    assert g.observed_state=='RED'
    for i in range(3):
        now=1_000_000_000+i*200_000_000
        p=dict(state='RED',stamp=dict(sec=1,nanosec=i*200_000_000),frame_id='Camera-1',image_width=640,image_height=480,
            predictions=[dict(label='red',confidence=.4,x=115.,y=120.,w=30.,h=40.)])
        g.observe_detector(json.dumps(p),now)
        r=g.decision(True,ROUTE,96.,0.,now)
    assert r['hold'] and r['state']=='RED'
    g.observe_detector('{"state":"UNKNOWN","predictions":[]}',1_600_000_000)
    assert g.payload is None


def test_large_camera_gap_cannot_confirm_red():
    g=gate();tick(g,1_000_000_000)
    tick(g,2_000_000_000)
    assert not tick(g,2_200_000_000)['hold']
