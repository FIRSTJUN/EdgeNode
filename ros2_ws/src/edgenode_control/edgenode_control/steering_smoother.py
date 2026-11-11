"""Time-based shaping of normalized steering commands, independent of ROS."""

import math


class SteeringSmoother:
    """Filter and slew-limit steering without accumulating hidden filter lag.

    The previous *output* is the only state. Safety stops bypass shaping and
    reset this state to zero. Zero settings retain the unfiltered legacy path.
    A scheduler gap never buys more than 100 ms of steering travel.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.command = 0.0

    def update(self, requested, dt, time_constant_sec=0.0, max_rate_per_sec=0.0,
               max_command=1.0):
        values = (requested, dt, time_constant_sec, max_rate_per_sec, max_command)
        try:
            valid = all(not isinstance(v, bool) and isinstance(v, (int, float))
                        and math.isfinite(v) for v in values)
        except OverflowError:
            valid = False
        if not valid:
            return None
        if (abs(requested) > 1.0 or time_constant_sec < 0 or max_rate_per_sec < 0
                or not 0.0 < max_command <= 1.0):
            return None
        requested = max(-max_command, min(max_command, requested))
        self.command = max(-max_command, min(max_command, self.command))
        if time_constant_sec == max_rate_per_sec == 0.0:
            self.command = float(requested)
            return self.command
        elapsed = max(0.0, min(dt, 0.1))
        alpha = -math.expm1(-elapsed / time_constant_sec) if time_constant_sec else 1.0
        change = alpha * (requested - self.command)
        if max_rate_per_sec:
            travel = max_rate_per_sec * elapsed
            change = max(-travel, min(travel, change))
        self.command += change
        return self.command
