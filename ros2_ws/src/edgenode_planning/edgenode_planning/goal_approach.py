"""Finite-route braking and measured arrival, independent of ROS and map matching.

The endpoint is a rear-axle target. Progress must come from the ordered route,
so a loop passing near its own endpoint cannot finish the mission early.
"""
import math


class GoalApproach:
    def __init__(self, deceleration_mps2=1.5, response_time_s=.8):
        if not all(math.isfinite(v) and v > 0 for v in (deceleration_mps2, response_time_s)):
            raise ValueError('positive finite goal braking bounds required')
        self.deceleration = deceleration_mps2
        self.response_time = response_time_s
        self.reset()

    def reset(self):
        self.reached = False
        self._settled_since = None
        self._last_time = None

    def update(self, points, remaining_m, pose, speed_kmh, now_s, *, armed):
        if (len(points) < 2 or not all(math.isfinite(v) for v in
                (remaining_m, *pose, speed_kmh, now_s)) or remaining_m < 0 or speed_kmh < 0
                or self._last_time is not None and now_s < self._last_time):
            raise ValueError('invalid terminal approach input')
        self._last_time = now_s
        end = points[-1]
        previous = next((p for p in reversed(points[:-1]) if math.dist(p[:2], end[:2]) > 1e-6),None)
        if previous is None:raise ValueError('nonzero terminal tangent required')
        yaw = math.atan2(end[1]-previous[1], end[0]-previous[0])
        error = math.dist(pose[:2], end[:2])
        heading_error = abs(math.atan2(math.sin(pose[2]-yaw), math.cos(pose[2]-yaw)))
        # Begin braking well before the point where the measured speed needs it.
        # The quadratic envelope reserves response delay, then finite braking.
        available = max(0., remaining_m-.35)
        delay_velocity = self.deceleration*self.response_time
        cap = 3.6*(math.sqrt(delay_velocity**2+2*self.deceleration*available)-delay_velocity)
        near = remaining_m <= max(80., speed_kmh/3.6*self.response_time +
            (speed_kmh/3.6)**2/(2*self.deceleration)+5.)
        at_target = remaining_m <= .8 and error <= .8 and heading_error <= math.radians(8.)
        stopped = speed_kmh <= .15
        if armed and at_target and stopped:
            self._settled_since = now_s if self._settled_since is None else self._settled_since
            if now_s-self._settled_since >= 2.:
                self.reached = True
        else:
            self._settled_since = None
        if self.reached or near and at_target and remaining_m <= .35:
            cap = 0.
        return dict(phase='GOAL_REACHED' if self.reached else 'SETTLING' if self._settled_since is not None
                    else 'APPROACH' if near else 'CRUISE', reached=self.reached,
                    remaining_m=remaining_m, position_error_m=error, heading_error_rad=heading_error,
                    measured_speed_kmh=speed_kmh, speed_cap_kmh=cap if near or self.reached else None,
                    hold_s=0. if self._settled_since is None else now_s-self._settled_since,
                    stopping_required_m=speed_kmh/3.6*self.response_time +
                        (speed_kmh/3.6)**2/(2*self.deceleration))
