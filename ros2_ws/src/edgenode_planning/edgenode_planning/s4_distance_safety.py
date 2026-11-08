"""S4-only admission model. ROI is a ceiling, never visibility evidence.

An operator-pinned, externally reviewed measurement report is required. Tests
use explicitly synthetic reports in temporary directories; none ships enabled.
Distances are from the rear axle along a validated stopping corridor, not a
claim that every empty point cloud proves free space. Physics remains subject
of the external validation report, not a consequence of passing this model.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import yaml

from edgenode_control.dynamic_lookahead import calculate_lookahead_distance


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (float, int)) and math.isfinite(value)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class DistancePolicy:
    speed_cap_kmh: float = 3.
    preview_m: float = 80.
    control_horizon_m: float = 20.
    perception_budget_s: float = .2
    planning_budget_s: float = 1.5
    control_watchdog_s: float = .6
    uncertainty_margin_m: float = 1.

    def __post_init__(self):
        if any(not finite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('positive finite S4 distance policy required')


@dataclass(frozen=True)
class ObservationEvidence:
    forward_range_m: float
    max_speed_kmh: float
    max_curvature: float
    route_link_ids: tuple
    source: str


def load_evidence(path, approved_sha256, lidar_file, control_file, map_dir, policy, config):
    """Reject absent/unpinned/mismatched/out-of-scope reports; never infer range.

    The pin records operator review, not authenticity or physical validation.
    Evidence must cover the configured delay/braking model and stopping corridor
    (including footprint, occlusion and ground/self-mask effects), not ROI alone.
    """
    if not path or not approved_sha256:
        return None, 'OBSERVATION_EVIDENCE_UNAVAILABLE'
    try:
        if sha256_file(path) != approved_sha256:
            return None, 'OBSERVATION_EVIDENCE_HASH'
        data = json.loads(Path(path).read_text())
        if (data['schema'] != 'edgenode.s4.observation.v1' or data['data_kind'] != 'measured_sensor_validation'
                or not isinstance(data['source'], str) or not data['source'].strip()):
            raise ValueError('report provenance')
        for key, file in (('lidar_config_sha256', lidar_file), ('control_config_sha256', control_file),
                          ('mgeo_links_sha256', Path(map_dir) / 'link_set.json')):
            if data[key] != sha256_file(file):
                return None, 'OBSERVATION_EVIDENCE_CONFIGURATION'
        lidar = yaml.safe_load(Path(lidar_file).read_text())['lidar_obstacle_node']['ros__parameters']
        control = yaml.safe_load(Path(control_file).read_text())['control_node']['ros__parameters']
        # Mirror the actual launch configuration, including watchdog and PP model.
        for key in ('wheelbase_m', 'max_wheel_angle_rad', 'max_front_steer_normalized',
                    'steering_sign', 'control_rate_hz', 'lookahead_min_m', 'lookahead_max_m',
                    'lookahead_preview_time_sec'):
            if getattr(config, key) != control[key]:
                return None, 'CONTROL_MODEL_MISMATCH'
        if policy.control_watchdog_s < max(control[k] for k in (
                'command_timeout_sec', 'local_path_timeout_sec', 'localization_timeout_sec', 'status_timeout_sec')):
            return None, 'CONTROL_WATCHDOG_BUDGET'
        ceiling = lidar['roi_max_x'] + lidar['lidar_offset_x_m']
        numbers = ('forward_range_m', 'max_speed_kmh', 'max_curvature',
                   'perception_latency_upper_s', 'planning_latency_upper_s',
                   'command_latency_upper_s', 'minimum_deceleration_mps2')
        if any(not finite(data[k]) or data[k] <= 0 for k in numbers):
            raise ValueError('report bounds')
        if data['forward_range_m'] > ceiling:
            return None, 'OBSERVATION_RANGE_EXCEEDS_ROI'
        if (data['perception_latency_upper_s'] > policy.perception_budget_s
                or data['planning_latency_upper_s'] > policy.planning_budget_s
                or data['command_latency_upper_s'] > config.command_delay_s
                or data['minimum_deceleration_mps2'] < config.deceleration_mps2):
            return None, 'OBSERVATION_EVIDENCE_MODEL_BOUNDS'
        if any(data[k] is not True for k in ('stopping_corridor_verified',
                'occlusion_policy_verified', 'full_obstacle_extent_verified', 'braking_verified',
                'timing_verified')):
            return None, 'OBSERVATION_EVIDENCE_INCOMPLETE'
        links = data['route_link_ids']
        if not isinstance(links, list) or not links or any(not isinstance(k, str) or not k for k in links):
            raise ValueError('explicit route scope')
        return ObservationEvidence(data['forward_range_m'], data['max_speed_kmh'], data['max_curvature'],
                                   tuple(links), data['source']), ''
    except (OSError, ValueError, TypeError, KeyError, OverflowError, yaml.YAMLError):
        return None, 'OBSERVATION_EVIDENCE_INVALID'


class DistanceSafety:
    def __init__(self, policy, config, body, evidence=None, evidence_reason='OBSERVATION_EVIDENCE_UNAVAILABLE'):
        self.policy, self.config, self.body = policy, config, body
        self.evidence, self.evidence_reason = evidence, evidence_reason

    def metrics(self, speed_kmh, source_age_s=0.):
        if not finite(speed_kmh) or speed_kmh < 0 or not finite(source_age_s) or source_age_s < 0:
            raise ValueError('invalid speed or source age')
        c, p = self.config, self.policy
        # Check prospective commanded speed BEFORE the first obstacle is seen.
        v = max(speed_kmh, p.speed_cap_kmh) / 3.6
        delays = dict(source_age_s=source_age_s, perception_budget_s=p.perception_budget_s,
                      planning_budget_s=p.planning_budget_s, control_watchdog_s=p.control_watchdog_s,
                      control_period_s=1. / c.control_rate_hz, command_delay_s=c.command_delay_s,
                      reaction_time_s=c.reaction_time_s)
        reaction = v * sum(delays.values())
        braking = v * v / (2. * c.deceleration_mps2)
        front = self.body.wheelbase_m + self.body.front_overhang_m
        required = front + c.obstacle_margin_m + p.uncertainty_margin_m + reaction + braking
        lookahead = calculate_lookahead_distance(v * 3.6, c.lookahead_min_m, c.lookahead_max_m,
                                               c.lookahead_preview_time_sec)
        # command_delay and lookahead already occur in Lattice's preparation arc.
        entry_reserve = p.uncertainty_margin_m + v * (sum(delays.values()) - c.command_delay_s)
        return dict(delays=delays, measured_speed_kmh=speed_kmh, checked_speed_kmh=v * 3.6,
                    front_m=front, reaction_distance_m=reaction, braking_distance_m=braking,
                    stopping_required_m=required, entry_reserve_m=entry_reserve,
                    entry_required_m={str(int(length)): lookahead + v * c.command_delay_s + length +
                                      front + c.obstacle_margin_m + entry_reserve
                                      for length in c.transition_lengths_m},
                    rollout_required_m=2. * v * c.rollout_seconds + lookahead + 2. * c.tracking_limit_m,
                    preview_m=p.preview_m, control_horizon_m=p.control_horizon_m,
                    verified_observation_m=self.evidence.forward_range_m if self.evidence else None)

    def evaluate(self, speed_kmh, source_age_s, curvature, route_link_ids=()):
        try:
            result = self.metrics(speed_kmh, source_age_s)
        except (ValueError, OverflowError):
            return dict(ready=False, reason='INVALID_DISTANCE_INPUT')
        reason = ''
        e = self.evidence
        if e is None:
            reason = self.evidence_reason
        elif not finite(curvature) or abs(curvature) > e.max_curvature or result['checked_speed_kmh'] > e.max_speed_kmh:
            reason = 'OBSERVATION_SCOPE'
        elif route_link_ids and any(k not in e.route_link_ids for k in route_link_ids):
            reason = 'OBSERVATION_ROUTE_SCOPE'
        elif result['stopping_required_m'] >= e.forward_range_m:
            reason = 'OBSERVATION_STOPPING_DISTANCE'
        elif result['stopping_required_m'] >= self.policy.preview_m:
            reason = 'INSUFFICIENT_HAZARD_PREVIEW'
        # Control window is checked against the exact extracted path in rollout.
        return dict(result, ready=not reason, reason=reason)
