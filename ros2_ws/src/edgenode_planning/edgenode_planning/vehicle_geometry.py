"""Explicit future full-body geometry contract; no guessed Sonata dimensions."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VerifiedVehicleGeometry:
    """All dimensions require verified units/axes and a named measurement source.

    rear_axle_from_pose is expressed in vehicle forward/left coordinates. The
    current planner does not enable full-body gates merely by importing this type.
    MORAI's ambiguous `overhang` field must be mapped explicitly after verification.
    """
    width_m: float
    length_m: float
    wheelbase_m: float
    front_overhang_m: float
    rear_overhang_m: float
    rear_axle_from_pose_forward_m: float
    rear_axle_from_pose_left_m: float
    pose_reference: str
    source: str

    def __post_init__(self):
        dimensions=(self.width_m,self.length_m,self.wheelbase_m,self.front_overhang_m,
                    self.rear_overhang_m,self.rear_axle_from_pose_forward_m,self.rear_axle_from_pose_left_m)
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in dimensions):
            raise ValueError('all geometry must be verified finite numeric metres')
        if min(self.width_m,self.length_m,self.wheelbase_m)<=0 or min(self.front_overhang_m,self.rear_overhang_m)<0:
            raise ValueError('invalid body dimensions')
        if not math.isclose(self.length_m,self.wheelbase_m+self.front_overhang_m+self.rear_overhang_m,abs_tol=1e-6):
            raise ValueError('length must equal verified wheelbase + front/rear overhangs')
        if any(not isinstance(v,str) or not v.strip() for v in (self.pose_reference,self.source)):
            raise ValueError('pose reference and geometry provenance are required')

    def polygon(self,x,y,yaw):
        """Oriented solid rectangle vertices for future swept-body validation."""
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            raise ValueError('pose must be finite')
        c,s=math.cos(yaw),math.sin(yaw)
        rear=self.rear_axle_from_pose_forward_m-self.rear_overhang_m
        front=self.rear_axle_from_pose_forward_m+self.wheelbase_m+self.front_overhang_m
        half=self.width_m/2
        vertices=[]
        for forward,left in ((rear,-half),(front,-half),(front,half),(rear,half)):
            left+=self.rear_axle_from_pose_left_m
            vertices.append((x+c*forward-s*left,y+s*forward+c*left))
        return vertices
