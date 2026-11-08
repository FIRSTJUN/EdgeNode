"""Distance tests use synthetic visibility; never approve the real K-City map."""
import json
import math
from pathlib import Path

import pytest

from edgenode_planning.s4_distance_safety import (
    DistancePolicy, DistanceSafety, ObservationEvidence, load_evidence, sha256_file)
from edgenode_planning.static_obstacle_planner import AvoidanceConfig
from edgenode_planning.vehicle_geometry import sonata_geometry


ROOT = Path(__file__).resolve().parents[2]
LIDAR = ROOT / 'edgenode_perception/config/lidar_obstacle.yaml'
CONTROL = ROOT / 'edgenode_control/config/control.yaml'


def safety(range_m=21.19, policy=None):
    return DistanceSafety(policy or DistancePolicy(), AvoidanceConfig(), sonata_geometry(),
                          ObservationEvidence(range_m, 50., .5, ('O0', 'O1'), 'synthetic corridor'))


@pytest.mark.parametrize('speed,allowed', [(3., True), (10., True), (20., False), (50., False)])
@pytest.mark.parametrize('age', [0., .6])
def test_stopping_envelope_at_four_speeds(speed, allowed, age):
    s = safety(); result = s.evaluate(speed, age, 0., ('O0', 'O1'))
    v = speed / 3.6
    assert result['stopping_required_m'] == pytest.approx(5.04 + v * (age + 2.7333333333333334) + v*v/4.)
    assert result['ready'] is allowed
    if not allowed: assert result['reason'] == 'OBSERVATION_STOPPING_DISTANCE'


@pytest.mark.parametrize('speed', [3., 10., 20., 50.])
def test_all_latency_terms_are_separate_and_increase_required_distance(speed):
    result = safety().metrics(speed, .6)
    assert set(result['delays']) == {'source_age_s', 'perception_budget_s', 'planning_budget_s',
        'control_watchdog_s', 'control_period_s', 'command_delay_s', 'reaction_time_s'}
    assert result['stopping_required_m'] - safety().metrics(speed)['stopping_required_m'] == pytest.approx(speed/3.6*.6)


def test_no_obstacle_does_not_prove_range_or_admit_high_speed_before_first_detection():
    s = DistanceSafety(DistancePolicy(), AvoidanceConfig(), sonata_geometry())
    assert s.evaluate(0., 0., 0.)['reason'] == 'OBSERVATION_EVIDENCE_UNAVAILABLE'
    assert safety().metrics(0.)['checked_speed_kmh'] == 3.
    assert safety(policy=DistancePolicy(speed_cap_kmh=50.)).evaluate(0., 0., 0.)['reason'] == 'OBSERVATION_STOPPING_DISTANCE'
    # A 3 km/h target cannot hide an actual 50 km/h vehicle.
    assert safety().evaluate(50., 0., 0.)['reason'] == 'OBSERVATION_STOPPING_DISTANCE'


@pytest.mark.parametrize('speed', [3., 10., 20., 50.])
def test_entry_deadlines_include_delays_without_double_counting_command_preparation(speed):
    s = safety(); m = s.metrics(speed); v = speed/3.6
    lookahead = min(10., max(2.5, v*.45))
    for length in (12, 16, 20):
        bare = lookahead + v*.1 + length + 4.04
        assert m['entry_required_m'][str(length)] == pytest.approx(bare + m['entry_reserve_m'])
        assert m['entry_required_m'][str(length)] > 21.19
    assert s.metrics(3.)['entry_required_m']['16'] > 22.623


@pytest.mark.parametrize('speed,required', [(3., 6.433333), (10., 14.211111),
                                          (20., 25.322222), (50., 62.405556)])
def test_twenty_metre_control_window_is_not_eighty_metre_preview(speed, required):
    m = safety().metrics(speed)
    assert m['rollout_required_m'] == pytest.approx(required, abs=1e-5)
    assert m['preview_m'] == 80. and m['control_horizon_m'] == 20.
    assert (m['rollout_required_m'] < 20.) == (speed <= 10.)


@pytest.mark.parametrize('speed,age,curvature', [(-1., 0., 0.), (math.nan, 0., 0.),
    (math.inf, 0., 0.), (3., -1., 0.), (3., math.nan, 0.), (3., 0., math.nan), (3., 0., 1.)])
def test_invalid_distance_input_and_scope_cannot_approve(speed, age, curvature):
    assert not safety().evaluate(speed, age, curvature)['ready']


def test_route_scope_and_preview_are_hard_limits():
    assert safety().evaluate(3., 0., 0., ('UNREVIEWED',))['reason'] == 'OBSERVATION_ROUTE_SCOPE'
    assert safety(policy=DistancePolicy(preview_m=5.)).evaluate(3., 0., 0.)['reason'] == 'INSUFFICIENT_HAZARD_PREVIEW'


@pytest.fixture
def report(tmp_path):
    directory = tmp_path / 'synthetic_map'; directory.mkdir()
    (directory / 'link_set.json').write_text('[]')
    data = dict(schema='edgenode.s4.observation.v1', data_kind='measured_sensor_validation',
        source='synthetic schema fixture; NOT real sensor evidence', forward_range_m=21.19,
        max_speed_kmh=3., max_curvature=.01, route_link_ids=['O0'],
        perception_latency_upper_s=.2, planning_latency_upper_s=1.5,
        command_latency_upper_s=.1, minimum_deceleration_mps2=2.,
        stopping_corridor_verified=True, occlusion_policy_verified=True,
        full_obstacle_extent_verified=True, braking_verified=True, timing_verified=True,
        lidar_config_sha256=sha256_file(LIDAR), control_config_sha256=sha256_file(CONTROL),
        mgeo_links_sha256=sha256_file(directory / 'link_set.json'))
    file = tmp_path / 'SYNTHETIC_NOT_FOR_DRIVING.json'
    return file, directory, data


def load(report, pin=True):
    file, directory, data = report; file.write_text(json.dumps(data))
    return load_evidence(str(file), sha256_file(file) if pin else '', LIDAR, CONTROL,
                         directory, DistancePolicy(), AvoidanceConfig())


def test_pinned_complete_report_can_be_loaded_but_default_cannot(report):
    assert load(report)[0].forward_range_m == 21.19
    assert load(report, pin=False)[0] is None


@pytest.mark.parametrize('key,value', [('forward_range_m', 80.), ('data_kind', 'roi'),
    ('source', ''), ('timing_verified', False), ('braking_verified', False),
    ('occlusion_policy_verified', False), ('full_obstacle_extent_verified', False),
    ('stopping_corridor_verified', False), ('perception_latency_upper_s', .3),
    ('planning_latency_upper_s', 2.), ('command_latency_upper_s', .2),
    ('minimum_deceleration_mps2', 1.), ('forward_range_m', math.nan),
    ('route_link_ids', []), ('lidar_config_sha256', 'unreviewed'),
    ('control_config_sha256', 'unreviewed'), ('mgeo_links_sha256', 'unreviewed')])
def test_roi_only_or_incomplete_or_mismatched_report_is_rejected(report, key, value):
    report[2][key] = value
    assert load(report)[0] is None


def test_hash_and_controller_mismatch_are_rejected(report):
    file, directory, data = report
    assert load(report)[0]
    assert load_evidence(file, '0'*64, LIDAR, CONTROL, directory,
                         DistancePolicy(), AvoidanceConfig())[1] == 'OBSERVATION_EVIDENCE_HASH'
    assert load_evidence(file, sha256_file(file), LIDAR, CONTROL, directory,
        DistancePolicy(), AvoidanceConfig(wheelbase_m=3.))[1] == 'CONTROL_MODEL_MISMATCH'


@pytest.mark.parametrize('speed', [3., 10., 20., 50.])
def test_delayed_stop_model_stays_inside_envelope_at_each_speed(speed):
    # Conservative analytic stop integrated with small time steps; NOT physical
    # MORAI braking evidence. Includes the old command remaining live on stall.
    m = safety().metrics(speed, .6); v = speed / 3.6
    x = v * sum(m['delays'].values()); dt = .001
    while v > 0:
        end = max(0., v - 2.*dt); x += (v+end)*.5*dt; v=end
    assert x + m['front_m'] + .25 < m['stopping_required_m']
