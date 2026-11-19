from dataclasses import replace
from test_static_obstacle_planner import setup


def test_live_lateral_limit_is_not_relaxed_and_rejection_retains_the_predicted_state():
 p,point,_=setup();p.config=replace(p.config,avoid_speed_kmh=10.,max_lateral_acceleration_mps2=.2)
 path=tuple(point(i*.5,0.) for i in range(101))
 assert p._rollout(path,(0.,0.,0.),10.,.1,())=='LATERAL_ACCELERATION'
 rejected=p._block('LATERAL_ACCELERATION')
 evidence=rejected['failure_context']
 assert evidence['lateral_acceleration_mps2']>evidence['limit_mps2']==.2
 assert evidence['prediction_t_s']==0. and evidence['speed_kmh']==10.
 assert evidence['initial_curvature_m_inv']==.1
