"""MORAI admission using separately measured speed/range limits, never ROI as evidence."""
import math


def stopping_speed_cap(distance_m, deceleration_mps2=2., delay_s=1.6, margin_m=2.):
    values=(distance_m,deceleration_mps2,delay_s,margin_m)
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):return 0.
    if distance_m<=margin_m or deceleration_mps2<=0 or min(delay_s,margin_m)<0:return 0.
    a=deceleration_mps2
    return 3.6*(math.sqrt((a*delay_s)**2+2*a*(distance_m-margin_m))-a*delay_s)


def required_deceleration_distance(speed_kmh, target_kmh, *, deceleration_mps2=2.,delay_s=1.6,margin_m=2.):
    if not all(math.isfinite(v) and v>=0 for v in (speed_kmh,target_kmh,delay_s,margin_m)) or deceleration_mps2<=0:
        raise ValueError('finite positive braking policy required')
    v,u=speed_kmh/3.6,target_kmh/3.6
    return v*delay_s+max(0.,v*v-u*u)/(2*deceleration_mps2)+margin_m
