from types import SimpleNamespace
from edgenode_planning.avoidance_visualization import display_candidates


def test_many_rejected_geometries_do_not_delay_control_for_rviz_rendering():
    candidates=[SimpleNamespace(side='left',rejection='ROAD_AREA_UNPROVEN',transition_length_m=12.,
                                points=((float(i),0.,0.),)) for i in range(500)]
    safe=SimpleNamespace(side='right',rejection='',transition_length_m=16.,points=((1.,2.,0.),))
    candidates.append(safe)
    shown=display_candidates(candidates)
    assert len(shown)<=16 and safe in shown
    assert len(candidates)==501 and all(c.points for c in candidates)
