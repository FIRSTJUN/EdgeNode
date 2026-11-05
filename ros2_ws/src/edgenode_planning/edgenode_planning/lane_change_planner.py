"""Fixed outbound/return paths, physical acquisition and latched centerlines."""
from dataclasses import replace
import math

from .lane_geometry import PolylineIndex, cross, sample
from .lane_change_path import (
    DEFAULT_CURVATURE_LIMIT, LaneChangePathError, _finite, generate_shift_path,
    prepare_target_centerline,
)
from .lane_change_types import (
    LaneAcquisitionConfig, LaneAcquisitionStatus,
    LaneChangeRequest, LaneChangeResult, LaneChangeState,
)


class LaneAcquisitionPolicy:
    """Geometry-only eligibility for either fixed transition; no state mutation."""
    def __init__(self, transition_path, target_path, config):
        self.config = config
        self._entry = PolylineIndex(transition_path.points)
        self._target = PolylineIndex(target_path.points)
        self._start = self._entry.lengths[transition_path.reference_s_m.index(transition_path.start_s_m)]
        self._end = self._entry.lengths[transition_path.reference_s_m.index(transition_path.end_s_m)]

    def evaluate(self, current_position, current_yaw):
        if (not isinstance(current_position, (list, tuple)) or len(current_position) != 2
                or not all(_finite(v) for v in current_position) or not _finite(current_yaw)):
            return LaneAcquisitionStatus(reason='INVALID_POSE')
        try:
            entry_s = self._entry.project(current_position)
            target_s = self._target.project(current_position)
        except OverflowError:
            return LaneAcquisitionStatus(reason='INVALID_PROJECTION')
        if not math.isfinite(entry_s) or not math.isfinite(target_s):
            return LaneAcquisitionStatus(reason='INVALID_PROJECTION')
        progress = entry_s - self._start
        fraction = progress / (self._end-self._start)
        remaining = self._end - entry_s
        status = LaneAcquisitionStatus(entry_progress_m=progress,
            entry_progress_fraction=fraction, entry_remaining_m=remaining)
        # PolylineIndex extrapolates endpoint station for other planner uses.
        # Acquisition must instead stay on the actual supplied path support.
        entry = sample(self._entry.points, entry_s, self._entry.lengths)
        target = sample(self._target.points, target_s, self._target.lengths)
        if entry is None or target is None:
            return replace(status, reason='ENTRY_REFERENCE_RANGE' if entry is None
                           else 'TARGET_REFERENCE_RANGE')
        center, tangent = target
        forward = (math.cos(current_yaw), math.sin(current_yaw))
        dot = sum(a*b for a, b in zip(tangent, forward))
        heading = math.atan2(cross(tangent, forward), dot)
        lateral = cross(tangent, (current_position[0]-center[0], current_position[1]-center[1]))
        aligned = (dot > 0 and sum(a*b for a, b in zip(entry[1], tangent)) > 0
                   and sum(a*b for a, b in zip(entry[1], forward)) > 0)
        status = replace(status, signed_lateral_distance_m=lateral,
                         heading_error_rad=heading, forward_aligned=aligned)
        if (remaining > self.config.entry_end_region_m
                or fraction < self.config.min_entry_progress_fraction):
            reason = 'ENTRY_END_REGION'
        elif not aligned:
            reason = 'DIRECTION_MISMATCH'
        elif abs(lateral) > self.config.max_lateral_distance_m:
            reason = 'TARGET_LATERAL_DISTANCE'
        elif abs(heading) > self.config.max_heading_error_rad:
            reason = 'TARGET_HEADING_ERROR'
        else:
            return replace(status, eligible=True, reason='ELIGIBLE')
        return replace(status, reason=reason)


class LaneChangePlanner:
    """Generate each transition once and confirm destination-lane acquisition.

    External behavior owns when/where to request a return. No scenario meaning
    or automatic return is inferred from request metadata or vehicle progress.
    """
    def __init__(self, *, curvature_limit_m_inv=DEFAULT_CURVATURE_LIMIT, acquisition_config=None):
        if acquisition_config is not None and not isinstance(acquisition_config, LaneAcquisitionConfig):
            raise TypeError('acquisition_config must be LaneAcquisitionConfig')
        self.curvature_limit_m_inv = curvature_limit_m_inv
        self._acquisition_config = acquisition_config or LaneAcquisitionConfig()
        self.reset()

    def reset(self):
        self._result = LaneChangeResult(LaneChangeState.IDLE)
        self._entry_path = self._target_path = self._policy = None
        self._recovery_path = None
        self._entry_request = self._return_request = None
        self._confirmation_count = 0
        self._acquisition = LaneAcquisitionStatus()

    @property
    def result(self):
        return self._result

    @property
    def entry_path(self):
        """The original ENTRY object remains available after centerline handoff."""
        return self._entry_path

    @property
    def acquisition_config(self):
        return self._acquisition_config

    @property
    def acquisition(self):
        return self._acquisition

    @property
    def recovery_path(self):
        return self._recovery_path

    def _generate_transition(self, request, reference_points, *, end_s_m, lateral_offset_m,
                             start_s_m, current_position, target_reference_points):
        path = generate_shift_path(reference_points, start_s_m=start_s_m,
            start_position=current_position, end_s_m=end_s_m, lateral_offset_m=lateral_offset_m,
            curvature_limit_m_inv=self.curvature_limit_m_inv)
        if path.lateral_offset_m * request.direction.sign <= 0:
            raise LaneChangePathError('DIRECTION_MISMATCH', 'offset sign disagrees with request')
        target = (prepare_target_centerline(target_reference_points,
                  curvature_limit_m_inv=self.curvature_limit_m_inv)
                  if target_reference_points is not None else None)
        return path, target

    def start(self, request, reference_points, *, end_s_m, lateral_offset_m,
              start_s_m=None, current_position=None, target_reference_points=None):
        """Repeated calls for the active request return its original result.

        Both reference geometries are consumed only in IDLE. A target reference
        is required for acquisition; omitting it preserves geometry-only usage.
        A different request requires explicit reset, including after BLOCKED.
        """
        if not isinstance(request, LaneChangeRequest):
            raise TypeError('request must be LaneChangeRequest')
        if self._result.state is not LaneChangeState.IDLE:
            if request != self._entry_request:
                raise RuntimeError('reset required before replacing a lane-change request')
            return self._result
        self._entry_request = request
        try:
            path, target = self._generate_transition(request, reference_points,
                start_s_m=start_s_m, current_position=current_position, end_s_m=end_s_m,
                lateral_offset_m=lateral_offset_m, target_reference_points=target_reference_points)
        except LaneChangePathError as error:
            self._result = LaneChangeResult(LaneChangeState.BLOCKED, request=request,
                blocked_reason=error.reason, detail=str(error))
        else:
            self._entry_path, self._target_path = path, target
            if target is not None:
                self._policy = LaneAcquisitionPolicy(path, target, self._acquisition_config)
            self._result = LaneChangeResult(LaneChangeState.ENTRY, path=path, request=request)
        return self._result

    def request_return(self, request, reference_points, *, end_s_m, lateral_offset_m,
                       target_reference_points, start_s_m=None, current_position=None):
        """Explicit external return from HOLD; one attempt until reset.

        Reference is the held lane, target reference the requested destination.
        Repeated equal requests ignore new geometry even in COMPLETE/BLOCKED.
        """
        if not isinstance(request, LaneChangeRequest):
            raise TypeError('request must be LaneChangeRequest')
        if self._return_request is not None:
            if request != self._return_request:
                raise RuntimeError('reset required before replacing a return request')
            return self._result
        if self._result.state is not LaneChangeState.TARGET_LANE_HOLD:
            raise RuntimeError('return request requires TARGET_LANE_HOLD')
        self._return_request = request
        try:
            if target_reference_points is None:
                raise LaneChangePathError('INVALID_TARGET_REFERENCE', 'return target reference required')
            path, target = self._generate_transition(request, reference_points,
                start_s_m=start_s_m, current_position=current_position, end_s_m=end_s_m,
                lateral_offset_m=lateral_offset_m, target_reference_points=target_reference_points)
        except LaneChangePathError as error:
            self._result = LaneChangeResult(LaneChangeState.BLOCKED, request=request,
                blocked_reason=error.reason, detail=str(error))
        else:
            self._recovery_path, self._target_path = path, target
            self._policy = LaneAcquisitionPolicy(path, target, self._acquisition_config)
            self._confirmation_count = 0
            self._acquisition = LaneAcquisitionStatus()
            self._result = LaneChangeResult(LaneChangeState.RECOVERING, path=path, request=request)
        return self._result

    def update(self, current_position=None, current_yaw=None):
        """Evaluate active transition acquisition; never regenerate or slice.

        A missing/invalid pose or any failed condition breaks confirmation.
        HOLD and COMPLETE return their exact latched result until explicit action.
        """
        if self._result.state not in (LaneChangeState.ENTRY, LaneChangeState.RECOVERING):
            return self._result
        status = (self._policy.evaluate(current_position, current_yaw) if self._policy is not None
                  else LaneAcquisitionStatus(reason='TARGET_REFERENCE_REQUIRED'))
        self._confirmation_count = self._confirmation_count+1 if status.eligible else 0
        confirmed = self._confirmation_count >= self._acquisition_config.confirmation_updates
        self._acquisition = replace(status, confirmation_count=self._confirmation_count,
                                   reason='ACQUIRED' if confirmed else status.reason)
        if confirmed:
            state = (LaneChangeState.COMPLETE if self._result.state is LaneChangeState.RECOVERING
                     else LaneChangeState.TARGET_LANE_HOLD)
            self._result = LaneChangeResult(state,
                path=self._target_path, request=self._result.request)
        return self._result
