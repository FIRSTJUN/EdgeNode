"""Measured raw LiDAR ray support; an empty object list is not free space."""
import math
import numpy as np

class RaySupport:
    def __init__(self,xyz,source_ns):
        xyz=np.asarray(xyz,dtype=float)
        if xyz.ndim!=2 or xyz.shape[1]!=3 or type(source_ns)is not int or source_ns<=0:raise ValueError('finite XYZ source frame required')
        xyz=xyz[np.isfinite(xyz).all(axis=1)];self.source_ns=source_ns
        order=np.argsort(np.arctan2(xyz[:,1],xyz[:,0]));self.xyz=xyz[order];self.ranges=np.linalg.norm(self.xyz[:,:2],axis=1);self.azimuth=np.arctan2(self.xyz[:,1],self.xyz[:,0])
    def support(self,target_xy,*,now_ns,angular_tolerance_rad=.006,minimum_belt_z=-.9,maximum_belt_z=-.4):
        if not -.05<=(now_ns-self.source_ns)/1e9<=.6:return dict(proven=False,reason='RAW_RAY_SOURCE_STALE')
        distance=float(np.linalg.norm(target_xy))
        if distance<=0:return dict(proven=False,reason='INVALID_RAY_TARGET')
        azimuth=math.atan2(target_xy[1],target_xy[0]);indices=[]
        for turn in (-2*math.pi,0.,2*math.pi):
            lo,hi=np.searchsorted(self.azimuth,(azimuth-angular_tolerance_rad+turn,azimuth+angular_tolerance_rad+turn));indices.extend(range(lo,hi))
        index=np.asarray(indices,dtype=int);index=index[self.ranges[index]>=distance+1.]
        z=self.xyz[index,2]*distance/self.ranges[index]
        good=index[(z>=minimum_belt_z)&(z<=maximum_belt_z)]
        return dict(proven=len(good)>=2,reason='MEASURED_BODY_BELT_RAY_SUPPORT' if len(good)>=2 else 'BODY_BELT_RAY_SUPPORT_UNPROVEN',ray_count=len(good),target_sensor_xy=list(map(float,target_xy)),minimum_endpoint_range_m=float(self.ranges[good].min()) if len(good) else None)
    def rear_strip(self,rear_m,lateral_m,*,now_ns,width_m=2.12):
        rows=[]
        for x in np.arange(-rear_m,-1.99,2.):
            for y in (lateral_m-width_m/2,lateral_m,lateral_m+width_m/2):rows.append(self.support((x-1.19,y),now_ns=now_ns))
        return dict(proven=bool(rows) and all(r['proven'] for r in rows),rear_m=rear_m,lateral_m=lateral_m,targets=len(rows),supported_targets=sum(r['proven'] for r in rows),missing_targets=[r for r in rows if not r['proven']])
