"""ROS-independent conversion from left-positive steering to MORAI commands."""

import math


def to_morai_front_steer(steering_normalized, steering_sign=-1.0,
                        max_front_steer_normalized=0.70):
    """Apply sign and command limit, or return None for invalid parameters."""
    values = (steering_normalized, steering_sign, max_front_steer_normalized)
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        try:
            if not math.isfinite(value):
                return None
        except OverflowError:
            return None
    if not 0 < max_front_steer_normalized <= 1.0:
        return None
    command = float(steering_sign) * float(steering_normalized)
    if not math.isfinite(command):
        return None
    return max(-max_front_steer_normalized, min(max_front_steer_normalized, command))


def limit_steering_change(requested,previous,rate,dt):
    """Bound wheel-angle change; zero rate preserves the existing command."""
    if (any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
            for v in (requested,previous,rate,dt)) or rate<0 or dt<=0):
        return None
    if rate==0:return requested
    step=rate*dt
    return previous+max(-step,min(step,requested-previous))
