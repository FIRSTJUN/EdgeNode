"""Located geometry, braking physics and conservative composition regressions."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.longitudinal_speed_planner import (
    CurveSpan, LongitudinalSpeedPlanner, RouteProfileBuilder, RouteSpeedProfile,
    SpeedPlanSettings, approach_speed, braking_distance,
)
from edgenode_planning.speed_planner import SpeedPlanner
from test_speed_planner import curvature_profile_path


def local(**settings):
    return SpeedPlanner(**(dict(cruise_speed_kmh=50.,curve_speed_kmh=25.,sharp_curve_speed_kmh=20.,
        curve_curvature_threshold=.025,sharp_curvature_threshold=.07) | settings))


def build(points, planner=None):
    builder = RouteProfileBuilder(points,planner or local())
    while builder.profile is None:
        builder.advance(16)
    return builder.profile


def span(entry=100.,exit=115.,level=2,k=.2,direction=1):
    return CurveSpan(entry,exit,level,direction,k,direction*k*(exit-entry),3)


@pytest.mark.parametrize('target,expected',[(25.,48.113425925925924),(20.,52.4537037037037)])
def test_fifty_kmh_distance_and_inverse(target,expected):
    settings = SpeedPlanSettings(deceleration_mps2=2.)
    assert braking_distance(50.,target,settings) == pytest.approx(expected)
    assert approach_speed(expected,target,settings) == pytest.approx(50.)
    assert approach_speed(expected-1.,target,settings) < 50.


@pytest.mark.parametrize('field,value',[
    ('deceleration_mps2',0.),('deceleration_mps2',math.nan),('deceleration_mps2',1e308),
    ('reaction_time_sec',-1.),('distance_margin_m',-1.),('lateral_accel_mps2',0.),
    ('speed_timeout_sec',0.),('preview_distance_m',0.),('preview_distance_m',True),
    ('acceleration_mps2',0.),('acceleration_mps2',math.nan),
    ('recovery_acceleration_mps2',0.),('recovery_acceleration_mps2',math.nan),
    ('recovery_clear_sec',0.),('recovery_clear_sec',math.inf),
])
def test_invalid_budget_is_rejected(field,value):
    settings = replace(SpeedPlanSettings(),**{field:value})
    assert not settings.valid()
    with pytest.raises(ValueError):braking_distance(50.,20.,settings)


@pytest.mark.parametrize('bad',[None,[],[[0,0,0]],[[0,0,0],[0,0,1]],
    [[0,0,0],[1,math.nan,0]],[[0,0,False],[1,0,0]],[[0,0,0],[100001,0,0]]])
def test_invalid_route_fails_closed(bad):
    with pytest.raises(ValueError):RouteProfileBuilder(bad,local())


def test_five_metre_radius_lateral_cap_is_below_twenty():
    settings = SpeedPlanSettings(lateral_accel_mps2=1.5)
    profile = RouteSpeedProfile(300.,(span(k=1/5.04),))
    planner = LongitudinalSpeedPlanner(profile,local(),settings)
    result = planner.plan(100.,9.,50.)
    assert result['target_speed_kmh'] == pytest.approx(3.6*math.sqrt(1.5*5.04))
    assert result['target_speed_kmh'] < 20.
    slower = LongitudinalSpeedPlanner(profile,local(sharp_curve_speed_kmh=8.),settings)
    assert slower.plan(100.,8.,30.)['target_speed_kmh'] == 8.


def test_gentle_bend_below_mode_threshold_still_has_lateral_limit():
    points = curvature_profile_path([(0.,80.,.5),(.015,40.,.5),(0.,100.,.5)])
    profile = build(points)
    assert profile.spans and any(s.level == 0 for s in profile.spans)
    result = LongitudinalSpeedPlanner(profile,local()).plan(90.,30.,50.)
    assert result['target_speed_kmh'] < 50.
    assert result['target_speed_kmh'] == pytest.approx(36.,abs=.2)


def test_gentle_approach_does_not_inherit_a_farther_bends_peak_curvature():
    points = curvature_profile_path([(0.,80.,.5),(.005,80.,.5),(.02,20.,.5),(0.,100.,.5)])
    profile = build(points)
    result = LongitudinalSpeedPlanner(profile,local()).plan(75.,50.,50.)
    assert result['target_speed_kmh'] == 50.
    assert LongitudinalSpeedPlanner(profile,local()).plan(160.,30.,50.)['target_speed_kmh'] < 40.


@pytest.mark.parametrize('speed',[30.,50.])
def test_straight_has_no_curve_braking_before_terminal_zone(speed):
    profile = build([[0.,0.,0.],[300.,0.,0.]])
    assert not profile.spans
    planner = LongitudinalSpeedPlanner(profile,local(cruise_speed_kmh=speed))
    for s in range(0,160,5):
        assert planner.plan(float(s),speed,speed)['target_speed_kmh'] == speed


@pytest.mark.parametrize('kind',['point','gaussian','zigzag'])
def test_isolated_artifact_does_not_become_a_global_curve(kind):
    points = [[i*.25,0.,0.] for i in range(721)]
    for p in points:
        x = p[0]
        if kind == 'point' and x == 80.:p[1] = 1.
        if kind == 'gaussian':p[1] = math.exp(-.5*((x-80.)/.5)**2)
        if kind == 'zigzag' and 78 < x < 82:p[1] = .1 if round(x/.25)%2 else -.1
    assert not build(points).spans


@pytest.mark.parametrize('shape',[
    [(0.,80.,.5),(.12,12.,.5),(0.,80.,.5)],
    [(0.,80.,.5),(.08,12.,.5),(-.08,12.,.5),(0.,80.,.5)],
    [(0.,80.,.5),(.05,2*math.pi/.05,.5),(0.,80.,.5)],
    [(0.,80.,.5),(.04,15.,.5),(-.04,15.,.5),(0.,80.,.5)],
])
def test_real_turns_s_curve_roundabout_merge_keep_support(shape):
    profile = build(curvature_profile_path(shape))
    assert profile.spans
    # The original classified turn remains located at the actual turn. The
    # gentle chord envelope can also anticipate steering before that point.
    classified = [span for span in profile.spans if span.level > 0]
    assert classified and abs(classified[0].entry_m-80.) < 2.
    if any(k < 0 for k,_,_ in shape):
        assert {s.direction for s in profile.spans} == {-1,1}
    result = LongitudinalSpeedPlanner(profile,local()).plan(40.,50.,50.)
    assert result['target_speed_kmh'] < 50.


def test_short_straight_does_not_accelerate_before_next_sharp_curve():
    profile = RouteSpeedProfile(400.,(span(100.,115.),span(130.,145.,direction=-1)))
    planner = LongitudinalSpeedPlanner(profile,local())
    previous = planner.plan(110.,10.,50.)['target_speed_kmh']
    result = planner.plan(116.,10.,50.)
    assert result['target_speed_kmh'] < 25.
    assert result['curve_id'] == 1
    assert result['target_speed_kmh'] <= approach_speed(14.,previous,planner.settings)+1e-9


def test_further_stricter_curve_can_control_before_nearest():
    profile = RouteSpeedProfile(300.,(span(80.,82.,1,.03),span(84.,100.,2,.2)))
    result = LongitudinalSpeedPlanner(profile,local()).plan(50.,20.,50.)
    assert result['curve_id'] == 1


def test_late_braking_latches_until_exit_and_resets_on_invalid_input():
    profile = RouteSpeedProfile(300.,(span(),))
    planner = LongitudinalSpeedPlanner(profile,local())
    first = planner.plan(80.,50.,50.)
    assert first['urgent'] and not first['braking_feasible']
    second = planner.plan(81.,8.,50.)
    assert second['urgent'] and second['target_speed_kmh'] == first['target_speed_kmh']
    assert planner.plan(82.,math.nan,50.) is None
    assert not planner._urgent
    assert not planner.plan(82.,8.,50.)['urgent']
    assert planner.plan(116.,8.,50.)['target_speed_kmh'] == 50.


def test_overspeed_expands_horizon_and_terminal_is_not_padded():
    profile = RouteSpeedProfile(300.,())
    normal = LongitudinalSpeedPlanner(profile,local()).plan(0.,50.,50.)
    fast = LongitudinalSpeedPlanner(profile,local()).plan(0.,80.,50.)
    assert fast['requested_preview_m'] > normal['requested_preview_m']
    planner = LongitudinalSpeedPlanner(profile,local())
    end = planner.plan(298.,10.,50.)
    assert end['available_preview_m'] == 2.
    assert 0. < end['target_speed_kmh'] < 10. and end['reason'] == 'TERMINAL_STOP'
    assert planner.plan(300.,0.,50.)['target_speed_kmh'] == 0.


def test_query_does_not_rebuild_and_incremental_build_matches_full_build():
    pts = curvature_profile_path([(0.,80.,.5),(.1,10.,.5),(0.,80.,.5)])
    before = deepcopy(pts)
    builder = RouteProfileBuilder(pts,local())
    assert builder.advance(1) is None and builder.next_window == 1
    while builder.profile is None:builder.advance(7)
    profile = builder.profile
    assert builder.advance() is profile and profile == build(pts)
    assert pts == before and not builder.cells


def test_arc_progress_distinguishes_loop_occurrences_and_rejects_backwards():
    profile = RouteSpeedProfile(500.,(span(100.,110.),span(300.,310.)))
    planner = LongitudinalSpeedPlanner(profile,local())
    assert planner.plan(120.,20.,50.)['target_speed_kmh'] == 50.
    assert planner.plan(290.,20.,50.)['curve_id'] == 1
    assert planner.plan(120.,20.,50.) is None


def test_recorded_638_support_and_original_local_limit_are_preserved():
    pts = json.loads((Path(__file__).parent/'fixtures/kcity_curve_638.json').read_text())['points']
    profile = build(pts)
    assert any(s.level == 2 and s.evidence & 2 for s in profile.spans)
    planner = LongitudinalSpeedPlanner(profile,local(sharp_curve_speed_kmh=8.))
    assert planner.plan(0.,8.,3.5)['target_speed_kmh'] <= 3.5


@pytest.mark.parametrize('bad',[math.nan,math.inf,-1.,True,1e308])
def test_invalid_speed_or_progress_cannot_produce_a_target(bad):
    planner = LongitudinalSpeedPlanner(RouteSpeedProfile(300.,()),local())
    assert planner.plan(0.,bad,50.) is None
    assert planner.plan(bad,50.,50.) is None


def test_timed_release_limits_only_increases_and_keeps_lower_constraints_immediate():
    planner = LongitudinalSpeedPlanner(RouteSpeedProfile(300.,(span(),)),local())
    first = planner.plan(110.,8.,8.,time_sec=1.)
    released = planner.plan(116.,8.,50.,time_sec=1.1)
    assert released['target_speed_kmh'] == pytest.approx(first['target_speed_kmh']+.36)
    assert released['release_limited'] and released['reason'] == 'RELEASE_LIMIT'
    assert released['uncapped_target_speed_kmh'] == 50.
    lower = planner.plan(117.,8.,3.5,time_sec=1.2)
    assert lower['target_speed_kmh'] == 3.5 and not lower['release_limited']
    assert lower['target_speed_kmh'] <= lower['uncapped_target_speed_kmh']


@pytest.mark.parametrize('stamp',[.9,math.nan,math.inf,-1.,True])
def test_invalid_timed_query_clears_release_and_urgent_state(stamp):
    planner = LongitudinalSpeedPlanner(RouteSpeedProfile(300.,(span(),)),local())
    assert planner.plan(80.,50.,50.,time_sec=1.)['urgent']
    assert planner.plan(81.,8.,50.,time_sec=stamp) is None
    assert planner._release_time is None and planner._release_target is None and not planner._urgent


def test_long_query_gap_caps_release_without_spurious_stop_for_fresh_speed():
    planner = LongitudinalSpeedPlanner(RouteSpeedProfile(300.,(span(),)),local())
    planner.plan(110.,8.,8.,time_sec=1.)
    result = planner.plan(116.,8.,50.,time_sec=10.)
    assert result is not None and result['target_speed_kmh'] == pytest.approx(8.72)
    assert result['release_limited']


def test_timed_s_curve_and_short_gap_keep_caps_without_reacceleration_jump():
    profile = RouteSpeedProfile(400.,(span(100.,115.),span(119.,130.,direction=-1)))
    planner = LongitudinalSpeedPlanner(profile,local())
    previous = None
    for index,s in enumerate(range(90,136)):
        result = planner.plan(float(s),10.,50.,time_sec=index*.1)
        assert result['target_speed_kmh'] <= result['uncapped_target_speed_kmh']
        if previous is not None:
            assert result['target_speed_kmh']-previous <= .36+1e-9
        if 115<=s<119:
            assert result['curve_id']==1 and result['target_speed_kmh']<25.
        previous = result['target_speed_kmh']


def test_reconfiguration_preserves_only_release_history():
    profile = RouteSpeedProfile(300.,(span(),))
    old = LongitudinalSpeedPlanner(profile,local())
    old.plan(80.,50.,6.,time_sec=1.)
    assert old._urgent
    new = LongitudinalSpeedPlanner(profile,local(sharp_curve_speed_kmh=30.))
    new.inherit_release_history(old)
    assert not new._urgent and new._progress is None
    result = new.plan(116.,8.,50.,time_sec=1.1)
    assert result['target_speed_kmh']==pytest.approx(6.36)


def test_confirmed_clear_road_recovers_faster_without_exceeding_caps():
    profile = RouteSpeedProfile(500.,())
    base = LongitudinalSpeedPlanner(profile,local(cruise_speed_kmh=30.),
        SpeedPlanSettings(recovery_acceleration_mps2=1.))
    candidate = LongitudinalSpeedPlanner(profile,local(cruise_speed_kmh=30.))
    for planner in [base,candidate]:
        planner.plan(0.,8.,8.,time_sec=0.,local_mode='SHARP_CURVE')
    first_base=first_candidate=None;previous=8.;active=False
    for i in range(1,81):
        t=i*.1
        b=base.plan(t*2,8.,30.,time_sec=t,local_mode='CRUISE')
        c=candidate.plan(t*2,8.,30.,time_sec=t,local_mode='CRUISE')
        if t<1.1:assert c['target_speed_kmh']==pytest.approx(b['target_speed_kmh'])
        assert b['target_speed_kmh']<=c['target_speed_kmh']<=c['uncapped_target_speed_kmh']
        assert c['target_speed_kmh']-previous<=.72+1e-9
        assert c['target_speed_kmh']-previous<=.5+1e-9
        if first_base is None and b['target_speed_kmh']>=29.5:first_base=t
        if first_candidate is None and c['target_speed_kmh']>=29.5:first_candidate=t
        active |= c['recovery_active'];previous=c['target_speed_kmh']
    assert active and first_candidate<first_base
    assert previous==30. # No asymptotic failure to reach exact cruise.


@pytest.mark.parametrize('mode',[None,'CURVE','SHARP_CURVE'])
def test_missing_or_noncruise_mode_cannot_authorize_faster_recovery(mode):
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(500.,()),local(cruise_speed_kmh=30.))
    planner.plan(0.,8.,8.,time_sec=0.,local_mode='SHARP_CURVE')
    previous=8.
    for i in range(1,25):
        r=planner.plan(float(i),8.,40.,time_sec=i*.1,local_mode=mode)
        assert not r['recovery_active'] and r['target_speed_kmh']-previous<=.36+1e-9
        previous=r['target_speed_kmh']


def test_binding_next_curve_disqualifies_fast_recovery_and_deceleration_is_immediate():
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(400.,(span(30.,50.),)),local())
    planner.plan(0.,8.,8.,time_sec=0.,local_mode='SHARP_CURVE')
    for i in range(1,25):
        r=planner.plan(float(i),8.,50.,time_sec=i*.1,local_mode='CRUISE')
        assert not r['recovery_active'] and r['target_speed_kmh']<=r['uncapped_target_speed_kmh']
    lower=planner.plan(25.,8.,3.5,time_sec=2.5,local_mode='SHARP_CURVE')
    assert lower['target_speed_kmh']==3.5 and not lower['release_limited']


def test_gap_and_reconfiguration_require_new_clear_road_observations():
    profile=RouteSpeedProfile(500.,())
    planner=LongitudinalSpeedPlanner(profile,local())
    planner.plan(0.,8.,8.,time_sec=0.,local_mode='SHARP_CURVE')
    for i in range(1,16):
        result=planner.plan(float(i),8.,50.,time_sec=i*.1,local_mode='CRUISE')
    assert result['recovery_active']
    old_target=result['target_speed_kmh']
    gap=planner.plan(16.,8.,50.,time_sec=3.,local_mode='CRUISE')
    assert not gap['recovery_active'] and gap['recovery_clear_sec']==0.
    assert gap['target_speed_kmh']==pytest.approx(old_target+.72)
    updated=LongitudinalSpeedPlanner(profile,local())
    updated.inherit_release_history(planner)
    r=updated.plan(17.,8.,50.,time_sec=3.1,local_mode='CRUISE')
    assert not r['recovery_active'] and r['target_speed_kmh']==pytest.approx(gap['target_speed_kmh']+.36)


def test_short_gentle_s_curve_is_not_discardable_as_weak_heading():
    points=curvature_profile_path([(0.,80.,.25),(.015,4.,.25),(-.015,4.,.25),(0.,100.,.25)])
    profile=build(points)
    assert profile.spans and {s.direction for s in profile.spans}=={-1,1}
    assert all(s.level==0 for s in profile.spans)
    r=LongitudinalSpeedPlanner(profile,local()).plan(82.,30.,50.)
    assert r['curve_speed_kmh']==pytest.approx(36.,abs=.2)
    assert r['target_speed_kmh']<50.


def test_unexpected_local_limit_still_applies_during_fast_recovery():
    planner=LongitudinalSpeedPlanner(RouteSpeedProfile(500.,()),local())
    targets=[planner.plan(0.,8.,8.,time_sec=0.,local_mode='SHARP_CURVE')['target_speed_kmh']]
    for i in range(1,51):
        r=planner.plan(i*.2,8.,50.,time_sec=i*.1,local_mode='CRUISE')
        assert r['target_speed_kmh']-targets[-1]<=.5+1e-9
        targets.append(r['target_speed_kmh'])
    assert r['recovery_active'] and targets[-1]>15.
    cut=planner.plan(10.2,8.,15.,time_sec=5.1,local_mode='CURVE')
    assert cut['target_speed_kmh']==15. and not cut['recovery_active']
    restored=planner.plan(10.4,8.,50.,time_sec=5.2,local_mode='CRUISE')
    assert restored['target_speed_kmh']==pytest.approx(15.36)
    assert not restored['recovery_active']
