"""Spend the proved approach over the remaining bounded computation time."""
import math


def approach_cap(remaining_m,reaction_s,deceleration_mps2,previous_search_ms,
                 remaining_episode_s=None):
    if not all(math.isfinite(v) for v in (remaining_m,reaction_s,deceleration_mps2,previous_search_ms)):
        raise ValueError('finite search approach state required')
    if remaining_m<0 or reaction_s<0 or deceleration_mps2<=0 or previous_search_ms<0:
        raise ValueError('invalid search approach state')
    horizon=max(2.,previous_search_ms/1000.+1.)
    if remaining_episode_s is not None:
        if not math.isfinite(remaining_episode_s):raise ValueError('finite episode remainder required')
        horizon=max(horizon,max(0.,remaining_episode_s))
    stopping=math.sqrt((deceleration_mps2*reaction_s)**2+
        2*deceleration_mps2*remaining_m)-deceleration_mps2*reaction_s
    return 3.6*min(stopping,remaining_m/horizon),horizon
