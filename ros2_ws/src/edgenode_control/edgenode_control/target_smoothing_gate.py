"""Keep spatial target rounding out of steering reversals."""
import math


class TargetSmoothingGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.sign = 0
        self.remaining_m = 0.0

    def update(self, raw_steering, travelled_m, hold_distance_m=6.0):
        values=(raw_steering,travelled_m,hold_distance_m)
        try:
            valid=all(not isinstance(v,bool) and isinstance(v,(int,float))
                      and math.isfinite(v) for v in values)
        except OverflowError:
            valid=False
        if not valid or travelled_m<0 or not 0<=hold_distance_m<=20:
            return None
        sign=1 if raw_steering>.05 else -1 if raw_steering<-.05 else 0
        if sign and self.sign and sign!=self.sign:
            self.remaining_m=hold_distance_m
        else:
            self.remaining_m=max(0.,self.remaining_m-travelled_m)
        if sign:self.sign=sign
        return self.remaining_m <= 1e-9
