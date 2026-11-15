"""Bounded source-time pose interpolation; never substitute a newer pose."""
from bisect import bisect_left
from collections import deque
import math


class PoseHistory:
    def __init__(self,max_gap_sec=.2,capacity=200):
        if not math.isfinite(max_gap_sec) or max_gap_sec<=0:raise ValueError('positive interpolation gap required')
        self.max_gap_ns=round(max_gap_sec*1e9);self.rows=deque(maxlen=capacity)

    def clear(self):self.rows.clear()

    def add(self,stamp_ns,pose):
        if (type(stamp_ns) is not int or stamp_ns<=0 or len(pose)!=3
                or not all(math.isfinite(v) for v in pose)):return False
        if self.rows and stamp_ns<=self.rows[-1][0]:return False
        if self.rows:
            previous=self.rows[-1];dt=(stamp_ns-previous[0])/1e9
            # Do not interpolate across a relocation. This only invalidates
            # evidence; it does not change localization or vehicle commands.
            if math.dist(previous[1][:2],pose[:2])>5.+80.*dt:self.clear()
        self.rows.append((stamp_ns,tuple(pose)));return True

    def sample(self,stamp_ns):
        rows=list(self.rows);stamps=[row[0] for row in rows];i=bisect_left(stamps,stamp_ns)
        if i<len(rows) and rows[i][0]==stamp_ns:
            return dict(pose=rows[i][1],span_ms=0.,nearest_delta_ms=0.)
        if i==0 or i==len(rows):return None
        a,p=rows[i-1];b,q=rows[i]
        if b-a>self.max_gap_ns:return None
        t=(stamp_ns-a)/(b-a);yaw=p[2]+t*math.atan2(math.sin(q[2]-p[2]),math.cos(q[2]-p[2]))
        return dict(pose=((1-t)*p[0]+t*q[0],(1-t)*p[1]+t*q[1],yaw),
                    span_ms=(b-a)/1e6,nearest_delta_ms=min(stamp_ns-a,b-stamp_ns)/1e6)
