"""Located global-route constraints and distance-based speed limits (no ROS).

No geometry sent to Pure Pursuit is changed. Build once per route/configuration
in bounded batches; queries use immutable spans and monotonic route progress.
Public speed arguments/results are km/h; internal physics uses SI units.
"""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, replace
import math

from .speed_planner import _finite_number, curve_support


@dataclass(frozen=True)
class SpeedPlanSettings:
    preview_distance_m: float = 80.0
    deceleration_mps2: float = 1.5
    reaction_time_sec: float = 0.5
    distance_margin_m: float = 5.0
    lateral_accel_mps2: float = 1.5
    speed_timeout_sec: float = 0.2
    acceleration_mps2: float = 1.0  # Target rise rate, not measured vehicle acceleration.
    recovery_acceleration_mps2: float = 2.0
    recovery_clear_sec: float = 1.0

    def valid(self):
        bounds = ((self.preview_distance_m,.01,100000.),
                  (self.deceleration_mps2,.01,20.),(self.lateral_accel_mps2,.01,20.),
                  (self.speed_timeout_sec,.01,5.),(self.reaction_time_sec,0.,10.),
                  (self.distance_margin_m,0.,1000.),(self.acceleration_mps2,.01,20.),
                  (self.recovery_acceleration_mps2,.01,20.),(self.recovery_clear_sec,.1,10.))
        return all(_finite_number(x) and lo <= x <= hi for x,lo,hi in bounds)


def braking_distance(speed_kmh, target_kmh, settings, include_margin=True):
    """Distance budget; configured deceleration is an assumption."""
    if (not settings.valid() or not all(_finite_number(x) and 0 <= x <= 1e6
                                       for x in (speed_kmh,target_kmh))):
        raise ValueError('invalid distance budget or speed')
    v, u = speed_kmh/3.6, target_kmh/3.6
    if v <= u:
        return 0.0
    return (v*settings.reaction_time_sec+(v*v-u*u)/(2*settings.deceleration_mps2)
            +(settings.distance_margin_m if include_margin else 0.0))


def approach_speed(distance_m, curve_speed_kmh, settings):
    if (not settings.valid() or not _finite_number(distance_m) or distance_m < 0
            or not _finite_number(curve_speed_kmh) or not 0 <= curve_speed_kmh <= 1e6):
        raise ValueError('invalid approach constraint')
    u, a, tau = curve_speed_kmh/3.6,settings.deceleration_mps2,settings.reaction_time_sec
    room = max(0.0,distance_m-settings.distance_margin_m)
    return max(curve_speed_kmh,3.6*(-a*tau+math.sqrt((a*tau)**2+u*u+2*a*room)))


@dataclass(frozen=True)
class CurveSpan:
    entry_m: float
    exit_m: float
    level: int
    direction: int
    curvature: float
    heading_change_rad: float
    evidence: int  # 1: median/persistence; 2: heading; 3: both


@dataclass(frozen=True)
class RouteSpeedProfile:
    length_m: float
    spans: tuple


class RouteProfileBuilder:
    """Incrementally extract located Phase 3 support on an arc grid.

    Retain original vertices; subdivide only long segments to <=2m. A 20m
    context owns its central 1m; boundary windows own the remaining tail.
    """
    STEP_M = 0.25

    def __init__(self, points, local_planner):
        if not isinstance(points,(list,tuple)) or not 2 <= len(points) <= 100000:
            raise ValueError('route requires 2..100000 finite XYZ points')
        self.points, self.arcs = [], []
        length = 0.0
        for p in points:
            if (not isinstance(p,(list,tuple)) or len(p) != 3
                    or not all(_finite_number(x) for x in p)):
                raise ValueError('invalid route point')
            q = tuple(float(x) for x in p)
            if self.points:
                ds = math.dist(q[:2],self.points[-1][:2])
                if ds == 0:
                    continue
                length += ds
            if not math.isfinite(length) or length > 100000.0:
                raise ValueError('route exceeds finite 100km analysis limit')
            self.points.append(q)
            self.arcs.append(length)
        if length <= 1e-6:
            raise ValueError('route has no usable length')
        self.length = length
        self.local_planner = local_planner
        self.centers = ([length/2] if length < 20 else
                        [10.0+i for i in range(int(length-20)+1)])
        if length >= 20 and self.centers[-1] < length-10-1e-9:
            self.centers.append(length-10)
        self.boundaries = [0.0]+[(a+b)/2 for a,b in zip(self.centers,self.centers[1:])]+[length]
        self.cells = [None]*math.ceil(length/self.STEP_M)
        self.next_window = 0
        self.profile = None

    def _point(self, s):
        i = min(len(self.points)-2,max(0,bisect_right(self.arcs,s)-1))
        f = (s-self.arcs[i])/(self.arcs[i+1]-self.arcs[i])
        return tuple((1-f)*a+f*b for a,b in zip(self.points[i],self.points[i+1]))

    def _slice(self, start, end):
        grid = [start]+self.arcs[bisect_right(self.arcs,start):bisect_left(self.arcs,end)]+[end]
        result = [self._point(start)]
        for a,b in zip(grid,grid[1:]):
            n = max(1,math.ceil((b-a)/2.0-1e-9))
            result.extend(self._point(a+(b-a)*i/n) for i in range(1,n+1))
        return result

    def advance(self, max_windows=8):
        if self.profile is not None:
            return self.profile
        if isinstance(max_windows,bool) or not isinstance(max_windows,int) or max_windows <= 0:
            raise ValueError('positive window budget required')
        stop = min(len(self.centers),self.next_window+max_windows)
        for i in range(self.next_window,stop):
            start = max(0.0,min(self.centers[i]-10,self.length-20))
            pts = self._slice(start,min(self.length,start+20))
            for lo,hi,level,k,direction,evidence in curve_support(pts,self.local_planner):
                a,b = max(start+lo,self.boundaries[i]),min(start+hi,self.boundaries[i+1])
                first = max(0,math.ceil(a/self.STEP_M-.5-1e-9))
                last = min(len(self.cells),math.ceil(b/self.STEP_M-.5-1e-9))
                for index in range(first,last):
                    old = self.cells[index]
                    if old is None or level > old[0] or (level == old[0] and k > old[1]):
                        self.cells[index] = (level,k,direction,evidence | (old[3] if old else 0))
                    elif level == old[0]:
                        self.cells[index] = (*old[:3],old[3] | evidence)
        self.next_window = stop
        if stop == len(self.centers):
            self.profile = RouteSpeedProfile(self.length,tuple(self._spans()))
            self.cells = []
        return self.profile

    def _heading_change(self, lo, hi):
        pts = self._slice(max(0,lo-.25),min(self.length,hi+.25))
        angles = [math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(pts,pts[1:])]
        return math.fsum(math.remainder(b-a,2*math.pi) for a,b in zip(angles,angles[1:]))

    def _spans(self):
        first = None
        low_k = high_k = 0.0
        for i in range(len(self.cells)+1):
            c = self.cells[i] if i < len(self.cells) else None
            old = self.cells[first] if first is not None else None
            if old and (c is None or (c[0],c[2]) != (old[0],old[2])
                        or max(high_k,c[1]) > 1.1*min(low_k,c[1])):
                lo,hi = first*self.STEP_M,min(i*self.STEP_M,self.length)
                flags = 0
                for v in self.cells[first:i]:
                    flags |= v[3]
                yield CurveSpan(lo,hi,old[0],old[2],max(v[1] for v in self.cells[first:i]),
                                self._heading_change(lo,hi),flags)
                first = None
            if c is not None and first is None:
                first = i
                low_k = high_k = c[1]
            elif c is not None:
                low_k,high_k = min(low_k,c[1]),max(high_k,c[1])


class LongitudinalSpeedPlanner:
    """Never raise the local limit; evaluate every relevant ahead constraint.

    Gentle sustained curvature also constrains lateral acceleration. An urgent
    cap latches only when the margin-free braking budget is already exceeded.
    """
    RECOVERY_STEP_KMH = 0.5  # Bound extra recovery per tick; preserve baseline release.
    def __init__(self, profile, local_planner, settings=SpeedPlanSettings()):
        self.profile, self.local_planner, self.settings = profile,local_planner,settings
        # The known route endpoint is the stop location itself. A curve's
        # advance margin must not move the mission endpoint five metres back.
        self._terminal_settings = replace(settings,distance_margin_m=0.0)
        self._entries = tuple(s.entry_m for s in profile.spans)
        self._exits = tuple(s.exit_m for s in profile.spans)
        self.reset()

    def reset(self):
        self._progress = None
        self._urgent = set()
        self._release_time = None
        self._release_target = None
        self._recovery_clear_since = None

    def inherit_release_history(self, previous):
        """Keep upward continuity across a committed non-geometric update.

        Do not inherit urgent/progress state, whose meaning depends on the old
        speed budget. Invalid input/route replacement still clears all history.
        """
        self._release_time = previous._release_time
        self._release_target = previous._release_target

    def _curve_speed(self, span):
        p = self.local_planner
        mode = (p.cruise_speed_kmh,p.curve_speed_kmh,p.sharp_curve_speed_kmh)[span.level]
        lateral = 3.6*math.sqrt(self.settings.lateral_accel_mps2/span.curvature)
        return min(p.cruise_speed_kmh,mode,lateral)

    def plan(self, progress_m, actual_speed_kmh, local_speed_kmh, *, time_sec=None, local_mode=None):
        p,cfg = self.profile,self.settings
        values = (progress_m,actual_speed_kmh,local_speed_kmh,
                  self.local_planner.cruise_speed_kmh,self.local_planner.curve_speed_kmh,
                  self.local_planner.sharp_curve_speed_kmh)
        if (not cfg.valid() or local_mode not in (None,'CRUISE','CURVE','SHARP_CURVE')
                or not all(_finite_number(x) and 0 <= x <= 1e6 for x in values)
                or not 0 <= progress_m <= p.length_m+1e-8
                or (self._progress is not None and progress_m < self._progress-1e-8)):
            self.reset()
            return None
        if time_sec is not None and (not _finite_number(time_sec) or time_sec < 0
                or (self._release_time is not None
                    and time_sec < self._release_time)):
            self.reset()
            return None
        self._progress = progress_m
        cruise = self.local_planner.cruise_speed_kmh
        requested = max(cfg.preview_distance_m,
                        braking_distance(max(actual_speed_kmh,cruise),0.0,cfg)+12.0)
        coverage = min(requested,max(0.0,p.length_m-progress_m))
        speed = min(cruise,local_speed_kmh)
        result = dict(target_speed_kmh=speed,reason='LOCAL_OR_CRUISE',mode='CRUISE',
                      requested_preview_m=requested,available_preview_m=coverage,
                      curve_id=None,distance_m=None,curve_speed_kmh=None,curvature=None,
                      required_distance_m=0.0,braking_margin_m=None,braking_feasible=True,
                      urgent=False,direction=0,heading_change_rad=0.0,evidence=0)
        self._urgent = {i for i in self._urgent if p.spans[i].exit_m > progress_m}
        first = bisect_right(self._exits,progress_m)
        last = bisect_right(self._entries,progress_m+requested)
        indices = sorted(set(range(first,last)) | self._urgent)
        constraints = [(i,p.spans[i],self._curve_speed(p.spans[i])) for i in indices]
        # The known finite route end is a zero-speed boundary, never padded road.
        if p.length_m-progress_m <= requested:
            constraints.append((-1,CurveSpan(p.length_m,p.length_m,0,0,0.0,0.0,0),0.0))
        for i,span,cap in constraints:
            distance = max(0.0,span.entry_m-progress_m)
            budget = self._terminal_settings if i == -1 else cfg
            required = braking_distance(actual_speed_kmh,cap,budget)
            physical = braking_distance(actual_speed_kmh,cap,budget,include_margin=False)
            if i >= 0 and actual_speed_kmh > cap and distance+1e-6 < physical:
                self._urgent.add(i)
            urgent = i in self._urgent
            limit = cap if urgent else approach_speed(distance,cap,budget)
            if limit <= speed and (limit < speed or result['curve_id'] is None):
                speed = min(speed,limit)
                result.update(target_speed_kmh=speed,
                    reason=('TERMINAL_STOP' if i == -1 else 'URGENT_DECEL' if urgent
                            else 'CURVE' if span.entry_m <= progress_m else 'APPROACH'),
                    mode=('STOP' if i == -1 else ('CRUISE','CURVE','SHARP_CURVE')[span.level]),
                    curve_id=i,distance_m=distance,curve_speed_kmh=cap,curvature=span.curvature,
                    required_distance_m=required,braking_margin_m=distance-required,
                    braking_feasible=distance+1e-6 >= required,urgent=urgent,
                    direction=span.direction,heading_change_rad=span.heading_change_rad,evidence=span.evidence)
        # Reductions remain immediate. A released curve/local-mode dip must not
        # produce an instantaneous acceleration target on the following cycle.
        result.update(uncapped_target_speed_kmh=speed,release_limited=False,
                      geometric_reason=result['reason'],release_rise_limit_kmh=None,
                      recovery_active=False,recovery_clear_sec=0.,
                      recovery_acceleration_mps2=cfg.acceleration_mps2)
        if time_sec is None:
            # Untimed calls are geometry/physics queries retained for callers
            # without a clock. ROS and timed replay always supply ROS time.
            self._release_time = self._release_target = None
            self._recovery_clear_since = None
        else:
            # Faster release requires observed Local CRUISE and no binding
            # future constraint. Never infer this from a missing Mode, a low
            # measured speed, or an unobserved scheduler gap.
            clear = (local_mode == 'CRUISE' and local_speed_kmh >= cruise-1e-6
                     and speed >= cruise-1e-6 and result['curve_id'] is None)
            if (not clear or (self._release_time is not None
                              and time_sec-self._release_time > cfg.speed_timeout_sec)):
                self._recovery_clear_since = None
            if clear and self._recovery_clear_since is None:
                self._recovery_clear_since = time_sec
            clear_sec = time_sec-self._recovery_clear_since if clear else 0.
            result['recovery_clear_sec'] = clear_sec
            if self._release_time is not None:
                # A scheduler/service gap must neither authorize a large rise
                # nor cause a new stop when the measured speed is still fresh.
                elapsed = min(time_sec-self._release_time,cfg.speed_timeout_sec)
                acceleration = cfg.acceleration_mps2
                if clear and clear_sec >= cfg.recovery_clear_sec:
                    faster = max(acceleration,cfg.recovery_acceleration_mps2)
                    gap = max(0.,speed-self._release_target)
                    # Taper only the extra rate near cruise; baseline release
                    # still reaches the exact cap without an asymptotic tail.
                    acceleration += (faster-acceleration)*min(
                        1.,gap/(3.6*faster*cfg.recovery_clear_sec))
                baseline_rise = 3.6*cfg.acceleration_mps2*elapsed
                # At the nominal 20Hz, 2m/s² permits only .36km/h per tick.
                # If the loop is slower, do not turn that extra recovery into
                # a larger jump. The existing baseline rise is never reduced.
                rise = min(3.6*acceleration*elapsed,max(baseline_rise,self.RECOVERY_STEP_KMH))
                effective = rise/(3.6*elapsed) if elapsed > 0 else cfg.acceleration_mps2
                result['recovery_acceleration_mps2'] = effective
                result['recovery_active'] = effective > cfg.acceleration_mps2+1e-9
                result['release_rise_limit_kmh'] = rise
                limited = min(speed,self._release_target+rise)
                if limited < speed:
                    result.update(target_speed_kmh=limited,release_limited=True,reason='RELEASE_LIMIT')
            self._release_time,self._release_target = time_sec,result['target_speed_kmh']
        return result
