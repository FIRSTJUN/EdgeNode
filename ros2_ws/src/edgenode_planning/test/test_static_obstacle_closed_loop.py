"""Actual production PP/steering adapters with causally generated vehicle poses."""
from functools import lru_cache
import json
import math
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from offline_static_avoidance import run
import offline_static_avoidance as simulation


@lru_cache(maxsize=16)
def outcome(speed,side,radius=math.inf,variant='nominal'):
    variants={'nominal':{},'small_errors':dict(lateral_error=.01,heading_error=.005),
        'one_mm':dict(lateral_error=.001),'outward_heading':dict(heading_error=-.01),
        'timing_and_delay':dict(periods=(1/8.,1/7.),delay_steps=1),
        'detection_loss':dict(disappear=True),
        'confirmation_interruption':dict(interrupt_confirmation=True),
        'decelerate':dict(brake_to_default=True)}
    # The current S4 launch explicitly uses the production 0.6 rad/s limiter.
    # Keep the no-limiter conservative rejections in the direct run() tests.
    result=run(speed,side,radius=radius,config_overrides=dict(steering_slew_limit_radps=.6),**variants[variant])
    output=os.environ.get('STATIC_AVOIDANCE_OUTPUT')
    if output:
        root=Path(output);root.mkdir(parents=True,exist_ok=True)
        (root/f'{speed:g}_{side}_{radius:g}_{variant}.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


@pytest.mark.parametrize('speed',[3.,10.])
@pytest.mark.parametrize('side',['left','right'])
def test_production_pp_full_lifecycle_with_early_known_obstacle_and_fifty_m_preview(speed,side):
    r=outcome(speed,side)
    assert r['success'],r['failure']
    assert r['phases']==['DETECT','AVOID_ENTRY','AVOID_HOLD','RETURN','NORMAL']
    assert r['entry_path_objects']==r['return_path_objects']==1
    assert r['entry_commits']==r['return_commits']==1
    assert r['candidate_lane_batches']==1  # No trajectory refit due to tracking error.
    assert r['max_planned_curvature_m_inv']<=simulation.AvoidanceConfig().curvature_limit
    assert r['clamp_count']==0 and r['max_steering']<=.70
    assert r['max_wheel_rate_radps']<=.6+1e-9
    assert r['min_body_clearance_m']>.25


@pytest.mark.parametrize('side',['left','right'])
def test_closed_loop_gentle_curve_crosses_link_and_returns(side):
    r=outcome(3.,side,300.)
    assert r['success'],r['failure']
    assert r['return_path_objects']==1 and r['clamp_count']==0
    assert r['max_wheel_rate_radps']<=.6+1e-9


@pytest.mark.parametrize('variant',['one_mm','small_errors','outward_heading','timing_and_delay','detection_loss','confirmation_interruption'])
def test_bounded_tracking_and_observation_disturbances(variant):
    r=outcome(3.,'left',variant=variant)
    assert r['success'],r['failure']
    assert r['entry_path_objects']==r['return_path_objects']==1
    assert r['min_body_clearance_m']>.25 and r['clamp_count']==0
    assert r['max_wheel_rate_radps']<=.6+1e-9
    if variant=='confirmation_interruption':assert r['confirmation_interrupted']


def test_initial_ten_kmh_decelerates_before_committing_default_three_kmh_entry():
    r=outcome(10.,'left',variant='decelerate')
    assert r['success'],r['failure']
    # An already observed obstacle is detected on the first planning call.
    assert r['phases']==['DETECT','AVOID_ENTRY','AVOID_HOLD','RETURN','NORMAL']
    assert all(row['speed_kmh']<=3.2 for row in r['rows'] if row['state']=='AVOID_ENTRY')


@pytest.mark.parametrize('speed',[3.,10.])
def test_current_preview_and_sensing_keep_low_speed_feasible_and_no_limiter_rejection(speed):
    import yaml
    config=yaml.safe_load((Path(__file__).parents[2]/'edgenode_perception/config/lidar_obstacle.yaml').read_text())['lidar_obstacle_node']['ros__parameters']
    planning=yaml.safe_load((Path(__file__).parents[1]/'config/planning.yaml').read_text())['planning_node']['ros__parameters']
    r=run(speed,'left',reference_horizon_m=planning['local_path_length_m'],
        observation_range_m=config['roi_max_x']+config['lidar_offset_x_m'])
    output=os.environ.get('STATIC_AVOIDANCE_OUTPUT')
    if output:
        root=Path(output);root.mkdir(parents=True,exist_ok=True)
        (root/f'{speed:g}_production_preview_and_sensing.json').write_text(json.dumps(r,indent=2)+'\n')
    assert planning['local_path_length_m']==80.
    if speed==3.:
        assert r['success'],r['failure']
        assert r['phases']==['NORMAL','DETECT','AVOID_ENTRY','AVOID_HOLD','RETURN','NORMAL']
        assert r['entry_commits']==r['return_commits']==1
    else:
        assert not r['success'] and r['failure']['reason']=='STEERING_RATE'
        assert r['phases']==['NORMAL','DETECT','AVOID_ENTRY','BLOCKED']
        assert r['entry_commits']==1 and r['return_commits']==0
    assert r['min_body_clearance_m']>.25


@pytest.mark.parametrize('speed',[3.,10.])
def test_actual_twenty_m_preview_with_early_known_obstacle_reports_current_rollout_limit(speed):
    r=run(speed,'left',reference_horizon_m=20.)
    output=os.environ.get('STATIC_AVOIDANCE_OUTPUT')
    if output:
        root=Path(output);root.mkdir(parents=True,exist_ok=True)
        (root/f'{speed:g}_production_preview_early_known.json').write_text(json.dumps(r,indent=2)+'\n')
    if speed==3.:
        assert r['success'],r['failure']
        assert r['phases']==['NORMAL','DETECT','AVOID_ENTRY','AVOID_HOLD','RETURN','NORMAL']
        assert r['entry_path_objects']==r['return_path_objects']==1
    else:
        # This is an explicit conservative rejection, not proof of an actual
        # actuator-rate violation. Pending steering commands are not observed
        # by Planning; every rollout initializes delay from measured turn.
        assert not r['success'] and r['failure']['reason']=='STEERING_RATE'
        assert r['phases']==['NORMAL','DETECT','AVOID_ENTRY','BLOCKED']
        assert r['max_wheel_rate_radps']<.6 and r['clamp_count']==0


@pytest.mark.parametrize('sign',[-1.,1.])
def test_exact_bicycle_quarter_circle_has_no_euler_or_pose_snap_error(sign):
    pose=simulation.bicycle_step((0.,0.,0.),sign/10.,5*math.pi)
    assert pose==pytest.approx((10.,sign*10.,sign*math.pi/2))


def test_harness_directly_uses_production_pp_and_command_adapter():
    compute=simulation.PurePursuit.compute
    with patch.object(simulation.PurePursuit,'compute',autospec=True,side_effect=compute) as pp, \
         patch.object(simulation,'to_morai_front_steer',wraps=simulation.to_morai_front_steer) as command:
        r=simulation.run(duration=.2)
    assert pp.call_count>=6
    assert command.call_count==len(r['rows'])==6
    assert r['distance_m']==pytest.approx(.2*3/3.6)
