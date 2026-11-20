"""D lane-internal clearance candidates; all original map/body gates still apply."""
import math
from shapely.geometry import Polygon
from edgenode_planning.lane_geometry import LanePair,sample,arc
from edgenode_planning.avoidance_geometry import swept_body
from .traffic import projected_piece

def clearance_pair(pair,offset_m):
    if not 0.<=offset_m<=.5:raise ValueError('D lane clearance candidate outside configured bound')
    if offset_m==0.:return pair
    lengths=arc(pair.adjacent);points=[];sign=1. if pair.side=='left' else -1.
    for point,s in zip(pair.adjacent,lengths):
        _,tangent=sample(pair.adjacent,s,lengths)
        points.append((point[0]-sign*offset_m*tangent[1],point[1]+sign*offset_m*tangent[0],*point[2:]))
    shifted=LanePair(pair.original,points,pair.ids,pair.side,pair.section_ends)
    for name in ('bank_crossings','bank_continuation','bank_lane_id'):
        if hasattr(pair,name):setattr(shifted,name,getattr(pair,name))
    shifted.clearance_offset_m=offset_m;shifted.native_adjacent=pair.adjacent
    return shifted

def parallel_clearance(pair,track,body):
    # This is an extra geometry reservation, not motion permission. It keeps
    # the unchanged 2 s / 2-sigma prediction and the full .6 s source-age bound.
    age=.6;t=2.;future=tuple(a+(t+age)*v for a,v in zip(track['center_xy'],track['velocity_xy_mps']))
    station=pair.adjacent_index.project(future);station=max(0.,min(pair.adjacent_index.lengths[-1],station))
    point,tangent=sample(pair.adjacent,station);pose=(*point[:2],math.atan2(tangent[1],tangent[0]))
    ego=Polygon(swept_body(body,pose,pose));other=Polygon(projected_piece(track,t,age,1.5))
    gap=ego.distance(other)
    return dict(proven=gap>=.4,minimum_gap_m=gap,required_gap_m=.4,source_age_bound_s=age,prediction_s=t,offset_m=getattr(pair,'clearance_offset_m',0.))
