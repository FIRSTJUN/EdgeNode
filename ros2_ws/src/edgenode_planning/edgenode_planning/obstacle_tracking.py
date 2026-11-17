"""Source-time world observations to persistent kinematics; no simulator API."""
from dataclasses import dataclass,field
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class Observation:
    center_xy: tuple
    footprints: tuple


@dataclass
class Track:
    identifier: int
    state: np.ndarray
    covariance: np.ndarray
    first_ns: int
    observed_ns: int
    frames: int
    observation: Observation
    history: list = field(default_factory=list)
    motion_velocity: np.ndarray = field(default_factory=lambda:np.zeros(2))
    motion_sigma: float = math.inf

    @property
    def speed_mps(self):return float(np.linalg.norm(self.state[2:]))

    @property
    def velocity_sigma(self):return math.sqrt(float(np.linalg.eigvalsh(self.covariance[2:,2:]).max()))

    @property
    def motion(self):
        if self.frames<5 or self.observed_ns-self.first_ns<400_000_000:return 'UNKNOWN'
        # Past displacement confidence and future acceleration uncertainty are
        # different quantities. KF prediction covariance must not classify a
        # steadily moving slow car as static forever.
        fitted=float(np.linalg.norm(self.motion_velocity))
        if fitted-2*self.motion_sigma>.3 and self.speed_mps>.3:return 'MOVING'
        if fitted+2*self.motion_sigma<.3:return 'STATIC'
        return 'UNKNOWN'

    def fit_motion(self,measurement_variance):
        rows=self.history[-20:]
        if len(rows)<5:return
        time=np.array([(s-rows[0][0])/1e9 for s,p in rows]);time-=time.mean()
        denominator=float(time@time)
        if denominator<=0:return
        positions=np.array([p for s,p in rows]);positions-=positions.mean(axis=0)
        self.motion_velocity=(time[:,None]*positions).sum(axis=0)/denominator
        residual=positions-time[:,None]*self.motion_velocity
        variance=max(measurement_variance,float((residual*residual).sum(axis=0).max())/max(1,len(rows)-2))
        self.motion_sigma=math.sqrt(variance/denominator)


class ObstacleTracking:
    def __init__(self,measurement_std_m=.2,acceleration_std_mps2=1.,max_missed_s=.8):
        if min(measurement_std_m,acceleration_std_mps2,max_missed_s)<=0:raise ValueError('positive tracking limits required')
        self.measurement_var=measurement_std_m**2;self.acceleration_var=acceleration_std_mps2**2
        self.max_missed_ns=round(max_missed_s*1e9);self.reset()

    def reset(self):self.tracks={};self.source_ns=None;self._next_id=1

    def update(self,source_ns,observations):
        if type(source_ns) is not int or source_ns<=0 or (self.source_ns is not None and source_ns<=self.source_ns):
            raise ValueError('strictly ordered source observations required')
        if any(len(o.center_xy)!=2 or not all(math.isfinite(v) for v in o.center_xy) for o in observations):
            raise ValueError('finite map-frame object centers required')
        dt=(source_ns-self.source_ns)/1e9 if self.source_ns is not None else 0.
        self.source_ns=source_ns
        F=np.eye(4);F[0,2]=F[1,3]=dt
        G=np.array(((dt*dt/2,0.),(0.,dt*dt/2),(dt,0.),(0.,dt)))
        self.tracks={key:t for key,t in self.tracks.items() if source_ns-t.observed_ns<=self.max_missed_ns}
        tracks=list(self.tracks.values())
        for t in tracks:
            t.state=F@t.state;t.covariance=F@t.covariance@F.T+self.acceleration_var*(G@G.T)
        cost=np.full((len(tracks),len(observations)),1e9)
        R=np.eye(2)*self.measurement_var
        for i,t in enumerate(tracks):
            S=t.covariance[:2,:2]+R;inverse=np.linalg.inv(S)
            for j,o in enumerate(observations):
                delta=np.asarray(o.center_xy)-t.state[:2];distance=float(delta@inverse@delta)
                if distance<=9.21:cost[i,j]=distance  # 99% 2-D measurement gate.
        matched=set()
        if cost.size:
            for i,j in zip(*linear_sum_assignment(cost)):
                if cost[i,j]>=1e9:continue
                t=tracks[i];o=observations[j]
                K=t.covariance[:,:2]@np.linalg.inv(t.covariance[:2,:2]+R)
                t.state=t.state+K@(np.asarray(o.center_xy)-t.state[:2])
                A=np.eye(4);A[:,:2]-=K
                t.covariance=A@t.covariance@A.T+K@R@K.T
                t.observed_ns=source_ns;t.frames+=1;t.observation=o;matched.add(j)
                t.history.append((source_ns,o.center_xy));t.history=t.history[-20:]
                t.fit_motion(self.measurement_var)
        for j,o in enumerate(observations):
            if j in matched:continue
            key=self._next_id;self._next_id+=1
            self.tracks[key]=Track(key,np.array((*o.center_xy,0.,0.),dtype=float),
                np.diag((self.measurement_var,self.measurement_var,9.,9.)),source_ns,source_ns,1,o,
                history=[(source_ns,o.center_xy)])
        return tuple(self.tracks.values())


def observations_from_frame(envelope):
    """Reassemble components from their retained footprint pieces, not tiles.

    Component indices are local to one source frame. Persistent IDs and
    velocities come only from source-time association, never simulator actors.
    """
    points=envelope['points'];objects=envelope['objects'];observations=[];used=set()
    for obj in objects:
        ids=obj['footprint_indices'];center=tuple(obj['center_xy'])
        if (len(center)!=2 or not all(math.isfinite(v) for v in center) or not ids or len(set(ids))!=len(ids)
                or any(type(i) is not int or i<0 or (i+1)*8>len(points) or i in used for i in ids)):
            raise ValueError('invalid component footprint membership')
        used.update(ids);pieces=tuple(tuple(tuple(p) for p in points[i*8:i*8+8]) for i in ids)
        # Long roadside components remain collision solids. They are not
        # interpreted as moving traffic from changing visibility centroids.
        vertices=[p for piece in pieces for p in piece]
        span=max(max(p[k] for p in vertices)-min(p[k] for p in vertices) for k in (0,1))
        if span<=12.:observations.append(Observation(center,pieces))
    if used!=set(range(len(points)//8)):raise ValueError('incomplete component membership')
    return observations


def closing_ttc(gap_m,ego_speed_mps,other_speed_mps):
    if not all(math.isfinite(v) for v in (gap_m,ego_speed_mps,other_speed_mps)):raise ValueError('finite TTC input required')
    if gap_m<=0:return 0.
    relative=ego_speed_mps-other_speed_mps
    return gap_m/relative if relative>0 else math.inf
