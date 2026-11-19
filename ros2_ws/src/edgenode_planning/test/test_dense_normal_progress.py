"""A metric-length curve must remain trackable after reference densification."""
import math

from edgenode_planning.local_path_planner import LocalPathPlanner


def test_dense_curved_reference_catches_up_without_removing_progress_guards():
    points=[(50.*math.sin(s/50.),50.*(1.-math.cos(s/50.)),0.) for s in (i*.25 for i in range(401))]
    planner=LocalPathPlanner();prepared=planner.prepare(points)
    errors={}
    for window in (10,200):
        previous=None;at=None;distance=0.
        for station in range(0,61,3):
            x,y,_=points[station*4]
            result=planner.extract(prepared,x,y,horizon_m=20.,previous_index=previous,
                search_back=3,search_forward=window,previous_arc_m=at)
            assert result is not None
            previous=result['nearest_index'];at=result['progress_arc_m'];distance=result['nearest_distance_m']
        errors[window]=distance
    assert errors[10]>3.5
    assert errors[200]<.25
