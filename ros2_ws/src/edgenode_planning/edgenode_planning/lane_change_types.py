"""ROS-independent lane-change intent, immutable path and lifecycle vocabulary."""
from dataclasses import dataclass
from enum import Enum
import math


class LaneChangeDirection(str, Enum):
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'

    @property
    def sign(self):
        """Positive lateral offset is left of the forward reference tangent."""
        return 1.0 if self is LaneChangeDirection.LEFT else -1.0


class LaneChangeState(str, Enum):
    IDLE = 'IDLE'
    ENTRY = 'ENTRY'
    TARGET_LANE_HOLD = 'TARGET_LANE_HOLD'
    RECOVERING = 'RECOVERING'
    RECOVERY = 'RECOVERING'  # Compatibility with the first-stage vocabulary.
    COMPLETE = 'COMPLETE'
    BLOCKED = 'BLOCKED'


@dataclass(frozen=True)
class LaneChangeRequest:
    """Opaque target link and caller-owned purpose; no scenario-specific gates."""
    target_link_id: str
    direction: LaneChangeDirection
    return_required: bool = False
    reason: str = ''

    def __post_init__(self):
        if not isinstance(self.target_link_id, str) or not self.target_link_id.strip():
            raise ValueError('target_link_id must be a nonempty string')
        if not isinstance(self.direction, LaneChangeDirection):
            raise ValueError('direction must be LaneChangeDirection')
        if not isinstance(self.return_required, bool) or not isinstance(self.reason, str):
            raise ValueError('return_required must be bool and reason must be str')


@dataclass(frozen=True)
class LaneChangePath:
    """Generated XY geometry and its original-reference station/shift samples.

    Tuple storage belongs to the generator; input arrays are never retained.
    The derivative bound is a continuous curvature bound on a straight reference
    only. Curved/polyline references additionally receive a sampled XY check.
    """
    points: tuple[tuple[float, float], ...]
    reference_s_m: tuple[float, ...]
    lateral_offsets_m: tuple[float, ...]
    start_s_m: float
    end_s_m: float
    lateral_offset_m: float
    maximum_curvature_m_inv: float
    shift_second_derivative_bound_m_inv: float
    curvature_limit_m_inv: float


@dataclass(frozen=True)
class LaneCenterlinePath:
    """Unshifted target centerline snapshot used after acquisition."""
    points: tuple[tuple[float, float], ...]
    maximum_curvature_m_inv: float
    curvature_limit_m_inv: float


@dataclass(frozen=True)
class LaneAcquisitionConfig:
    """Shared transition acquisition tolerances; no scenario gates.

    End-region distance and minimum fraction both apply to the active fixed
    transition's XY arc, excluding its unshifted prefix and constant-offset tail.
    Confirmation counts consecutive complete, eligible update calls.
    """
    max_lateral_distance_m: float = 0.15
    max_heading_error_rad: float = math.radians(8.0)
    entry_end_region_m: float = 2.0
    min_entry_progress_fraction: float = 0.80
    confirmation_updates: int = 3

    def __post_init__(self):
        values = (self.max_lateral_distance_m, self.max_heading_error_rad,
                  self.entry_end_region_m, self.min_entry_progress_fraction)
        try:
            valid = all(not isinstance(v, bool) and isinstance(v, (int, float))
                        and math.isfinite(v) for v in values)
        except OverflowError:
            valid = False
        if (not valid or self.max_lateral_distance_m <= 0 or self.entry_end_region_m <= 0
                or not 0 < self.max_heading_error_rad < math.pi/2
                or not 0 < self.min_entry_progress_fraction < 1):
            raise ValueError('invalid acquisition distance, heading or progress thresholds')
        if (isinstance(self.confirmation_updates, bool)
                or not isinstance(self.confirmation_updates, int)
                or self.confirmation_updates < 2):
            raise ValueError('confirmation_updates must be an integer >= 2')


@dataclass(frozen=True)
class LaneAcquisitionStatus:
    """Last active-transition evaluation; entry_* names retain API compatibility."""
    eligible: bool = False
    reason: str = 'NOT_EVALUATED'
    entry_progress_m: float | None = None
    entry_progress_fraction: float | None = None
    entry_remaining_m: float | None = None
    signed_lateral_distance_m: float | None = None
    heading_error_rad: float | None = None
    forward_aligned: bool = False
    confirmation_count: int = 0

    @property
    def transition_progress_m(self):
        return self.entry_progress_m

    @property
    def transition_remaining_m(self):
        return self.entry_remaining_m


@dataclass(frozen=True)
class LaneChangeResult:
    state: LaneChangeState
    path: LaneChangePath | LaneCenterlinePath | None = None
    request: LaneChangeRequest | None = None
    blocked_reason: str = ''
    detail: str = ''

    @property
    def target_link_id(self):
        return self.request.target_link_id if self.request is not None else None
