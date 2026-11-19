"""Native exact distances on the existing swept solids and axle envelopes.

No buffers or replacement vehicle geometry are introduced. The union is an
exact acceleration of testing every original path segment separately.
"""
from functools import lru_cache
import math
from shapely.geometry import Polygon,LineString,Point,box
from shapely.strtree import STRtree
from .avoidance_geometry import inflate


@lru_cache(maxsize=4096)
def shape(points):
    if len(points)==1:return Point(points[0])
    if len(points)==2:return LineString(points)
    return Polygon(points)


def distance(first,second):
    return shape(tuple(map(tuple,first))).distance(shape(tuple(map(tuple,second))))


def normal_hazards(segments,obstacles,body_margin,reference_margin):
    if not segments:return (),math.inf
    rows=[(shape(g[0]),shape(inflate((tuple(a[:2]),tuple(b[:2])),g[3]))) for a,b,g in segments]
    tree=STRtree([r[0] for r in rows]);by_id={}
    for body,axle in rows:by_id.setdefault(id(body),[]).append((body,axle))
    reach=max(body_margin,reference_margin)
    found=[];minimum=math.inf
    for obstacle in obstacles:
        solid=shape(tuple(map(tuple,obstacle)));x0,y0,x1,y1=solid.bounds
        candidates=tree.query(box(x0-reach,y0-reach,x1+reach,y1+reach))
        # Every omitted body AND its enclosed axle have bounding-box
        # separation greater than reach. This is a conservative clearance
        # lower bound, not a substituted free-space certificate.
        minimum=min(minimum,reach);hit=False
        for candidate in candidates:
            options=by_id[id(candidate)] if hasattr(candidate,'geom_type') else (rows[int(candidate)],)
            for body,axle in options:
                clearance=body.distance(solid);minimum=min(minimum,clearance)
                if clearance<=body_margin or axle.distance(solid)<reference_margin:
                    found.append(obstacle);hit=True;minimum=0.;break
            if hit:break
    return tuple(found),minimum
