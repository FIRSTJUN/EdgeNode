"""Confirm relocation from distinct GPS samples; a single outlier never resets EKF."""
import math


class PoseResetDetector:
    def __init__(self, jump_m=10., max_speed_mps=20., confirmations=3, consistency_m=1.):
        if jump_m<=0 or max_speed_mps<=0 or confirmations<2 or consistency_m<=0:
            raise ValueError('positive relocation limits and at least two confirmations required')
        self.jump_m=jump_m;self.max_speed_mps=max_speed_mps
        self.confirmations=confirmations;self.consistency_m=consistency_m
        self.anchor=None;self.candidate=None;self.count=0;self.expected=None

    def expect(self, x, y):
        if not all(math.isfinite(v) for v in (x,y)):raise ValueError('finite reset position required')
        self.expected=(x,y);self.candidate=None;self.count=0

    def observe(self, x, y, stamp_ns):
        """Return NORMAL, PENDING, CANCELLED or CONFIRMED without changing an EKF."""
        if not all(math.isfinite(v) for v in (x,y)) or stamp_ns<=0:return 'INVALID'
        row=(x,y,stamp_ns)
        if self.candidate is not None and stamp_ns<=self.candidate[2]:return 'INVALID'
        if self.anchor is not None and stamp_ns<=self.anchor[2]:return 'INVALID'
        if self.expected is not None:
            jumped=math.dist((x,y),self.expected)<=self.consistency_m
            if not jumped:return 'PENDING'
        elif self.anchor is None:
            self.anchor=row;return 'NORMAL'
        else:
            dt=((self.candidate[2] if self.candidate is not None else stamp_ns)-self.anchor[2])/1e9
            jumped=math.dist((x,y),self.anchor[:2])>self.jump_m+self.max_speed_mps*dt
        if not jumped:
            cancelled=self.candidate is not None
            self.anchor=row;self.candidate=None;self.count=0
            return 'CANCELLED' if cancelled else 'NORMAL'
        if self.candidate is not None:
            dt=(stamp_ns-self.candidate[2])/1e9
            consistent=math.dist((x,y),self.candidate[:2])<=self.consistency_m+self.max_speed_mps*dt
            self.count=self.count+1 if consistent else 1
        else:self.count=1
        self.candidate=row
        return 'CONFIRMED' if self.count>=self.confirmations else 'PENDING'

    def accept(self, x, y, stamp_ns):
        self.anchor=(x,y,stamp_ns);self.candidate=None;self.count=0;self.expected=None
