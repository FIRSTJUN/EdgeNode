"""Separate source-owned vehicle observations from persistent static memory.

Every latest raw return remains an immediate collision solid. Static history
uses the complete paired LiDAR frame, excluding only pieces positively owned
by a sensor track in that exact source frame. Missing vehicle tracks remain
uncertain solids until behind Ego; expiration is never a free-space claim.
"""
import math
from shapely.geometry import Polygon,Point,GeometryCollection
from shapely.strtree import STRtree
from shapely.ops import unary_union
from edgenode_normal_experiment.observation_memory import NormalObservationMemory
from edgenode_planning.avoidance_geometry import inflate
from .traffic import projected_piece

def collision_parts(shape):
    """Exact union of overlapping collection pieces before GEOS predicates."""
    if shape.is_empty:return []
    if shape.geom_type=='GeometryCollection':
        normalized=unary_union(list(shape.geoms))
        if normalized.geom_type=='GeometryCollection':
            return [piece for child in normalized.geoms for piece in collision_parts(child)]
        return collision_parts(normalized)
    if shape.geom_type.startswith('Multi'):
        return [piece for child in shape.geoms for piece in collision_parts(child)]
    return [shape]

class SceneMemory:
    def __init__(self,provider):
        self.provider=provider;self.static=NormalObservationMemory();self.vehicles={};self.geometry=GeometryCollection();self.tree=STRtree([]);self.debug={}
    @property
    def tracks(self):return self.static.tracks
    @property
    def cells(self):return self.static.cells+list(self.vehicles.values())
    def update(self,observed,index,ego,rear,padding,*,pose,stamp,now):
        value=self.provider();scene=value.get('observation_scene',{});source=scene.get('source_ns')
        if source is not None and source==value.get('source_stamp_ns'):
            def key(piece):return tuple(sorted(tuple(p) for p in piece))
            owned={key(piece) for t in value.get('tracks',[]) if t['observed_ns']==source for piece in t.get('source_footprints_raw',[])}
            points=scene['points'];unowned=[]
            for i in range(0,len(points),8):
                piece=points[i:i+8]
                if len(piece)==8 and key(piece) not in owned:unowned.append(piece)
            self.static.update(unowned,index,ego,rear,padding,pose=pose,stamp=source,now=now)
        session=value.get('tracker_session_id')
        for track in value.get('tracks',[]):
            if not track.get('source_footprints_raw'):continue
            name=(session,track['id'])
            if track['missed']:
                shape=Polygon(projected_piece(track,0.,max(0.,now-track['state_source_ns']/1e9),1.5))
            else:shape=unary_union([Polygon(inflate(p,padding)) for p in track['source_footprints_raw']])
            for previous,record in list(self.vehicles.items()):
                if previous[0]!=session and record['shape'].intersects(shape) and record['shape'].centroid.distance(shape.centroid)<.75:
                    # A reset can rename a continuously observed vehicle.
                    # Positive overlapping source geometry replaces that
                    # vehicle hypothesis; unowned static history is untouched.
                    del self.vehicles[previous]
            self.vehicles[name]=dict(shape=shape,observed_ns=track['observed_ns'],motion=track.get('motion','UNKNOWN'))
        for name,record in list(self.vehicles.items()):
            shape=record['shape'];x0,y0,x1,y1=shape.bounds
            corners=((x0,y0),(x0,y1),(x1,y0),(x1,y1))
            if all(math.cos(pose[2])*(x-pose[0])+math.sin(pose[2])*(y-pose[1])<-rear-.5 for x,y in corners) and all(index.project(p)<ego-rear-.5 for p in corners):del self.vehicles[name]
        shapes=[t['shape'] for t in self.static.tracks]+[v['shape'] for v in self.vehicles.values()]+[Polygon(inflate(p,padding)) for p in observed]
        shapes=[part for shape in shapes for part in collision_parts(shape)];self.tree=STRtree(shapes);self.geometry=GeometryCollection(shapes)
        self.debug=dict(static=self.static.debug,tracked_vehicle_solids=len(self.vehicles),current_raw_solids=len(observed),source_owned_partition=True,missing_vehicle_is_not_free_space=True)
        return self.geometry
