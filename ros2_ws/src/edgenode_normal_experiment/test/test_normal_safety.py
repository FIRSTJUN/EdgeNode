from types import SimpleNamespace
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
from edgenode_normal_experiment.normal_safety import ExactObservedTiles,NormalSafetyPlanner
from edgenode_planning.static_obstacle_planner import AvoidanceConfig
from edgenode_planning.vehicle_geometry import morai_sonata_collision_envelope
from edgenode_planning.lane_geometry import PolylineIndex

PATH=tuple((float(i),0.,0.) for i in range(81))
def box(x,y,width=1.,length=2.):return ((x,y),(x+length,y),(x+length,y+width),(x,y+width))

def planner():
    class Road:
        def covers(self,polygon):return True
    class Boundaries:
        cells={};forbidden=[]
        def swept_reason(self,polygon):return ''
    return NormalSafetyPlanner(SimpleNamespace(road_corridor=Road(),boundaries=Boundaries()),config=AvoidanceConfig(native_obstacle_geometry=True,continuous_lane_follow=True,deceleration_mps2=4.,observation_padding_m=.1),weights=None,body=morai_sonata_collision_envelope(),route_provider=lambda:{})

def test_exact_tiles_never_fill_empty_inside_of_observed_corner():
    memory=ExactObservedTiles();idx=PolylineIndex(PATH)
    result=memory.update([box(20,3,1,4),box(23,3,4,1)],idx,0.,1.12,.1)
    assert not result.covers(Point(21,6))
    assert result.covers(Polygon(box(20,3,1,4)))

def test_missing_observation_preserves_actual_occupied_space():
    memory=ExactObservedTiles();idx=PolylineIndex(PATH);shape=memory.update([box(40,-1,2,4)],idx,0.,1.12,.1)
    retained=memory.update([],idx,0.,1.12,.1)
    assert retained.equals(shape)

def test_normal_clear_road_keeps_original_path_and_never_activates_lateral_mode():
    p=planner();result=p.plan(PATH,[box(30,8)],(0.,0.,0.),speed_kmh=60.,now_sec=0.)
    assert not result['blocked'] and not result['active'] and result['points']==PATH
    assert result['entry_commits']==result['return_commits']==result['generated_transition_count']==0
    assert result['static_avoidance_method']=='NONE_PHASE_A'

def test_real_blocker_creates_safe_speed_cap_without_lateral_motion():
    p=planner();result=p.plan(PATH,[box(55,-1,2,4)],(0.,0.,0.),speed_kmh=30.,now_sec=0.)
    assert not result['blocked'] and not result['active'] and result['points']==PATH
    assert result['speed_cap_kmh'] is not None and result['entry_commits']==0

def test_imminent_real_blocker_stops_and_never_searches():
    p=planner();result=p.plan(PATH,[box(8,-1,2,4)],(0.,0.,0.),speed_kmh=60.,now_sec=0.)
    assert result['blocked'] and result['points']==[] and result['speed_cap_kmh']==0.
    assert result['reason']=='NORMAL_BRAKING_DISTANCE'

def test_invalid_source_stops_instead_of_being_treated_as_free_space():
    result=planner().plan(PATH,[],(0.,0.,0.),speed_kmh=20.,observation_valid=False)
    assert result['blocked']


def test_confirmed_real_object_survives_missing_views():
    from edgenode_normal_experiment.observation_memory import NormalObservationMemory
    memory=NormalObservationMemory();idx=PolylineIndex(PATH);obj=box(40,-1,2,4)
    memory.update([obj],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=1,now=0.)
    old=memory.update([obj],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=2,now=.1)
    for i in range(3,20):
        result=memory.update([],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=i,now=i*.1)
    assert result.equals(old) and memory.debug['confirmed']==1


def test_provisional_return_is_immediate_then_requires_fresh_unoccluded_views_to_retire():
    from edgenode_normal_experiment.observation_memory import NormalObservationMemory
    memory=NormalObservationMemory();idx=PolylineIndex(PATH);obj=box(40,-1,2,4)
    original=memory.update([obj],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=1,now=0.)
    assert not original.is_empty
    for i in range(10):
        assert memory.update([],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=1,now=2.).equals(original)
    memory.update([],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=2,now=.2)
    assert not memory.update([],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=3,now=.4).is_empty
    assert memory.update([],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=4,now=.7).is_empty


def test_current_occluder_prevents_retiring_an_unconfirmed_real_object():
    from edgenode_normal_experiment.observation_memory import NormalObservationMemory
    memory=NormalObservationMemory();idx=PolylineIndex(PATH);obj=box(40,-1,2,4);closer=box(20,-1,2,3)
    memory.update([obj],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=1,now=0.)
    for i in range(2,15):
        result=memory.update([closer],idx,0.,1.12,.1,pose=(0.,0.,0.),stamp=i,now=i*.1)
    assert result.covers(Polygon(obj))
