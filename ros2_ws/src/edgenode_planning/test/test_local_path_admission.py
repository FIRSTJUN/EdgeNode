from unittest.mock import Mock
import pytest
from test_static_obstacle_node import node,output


@pytest.mark.parametrize('state,allowed',[('NORMAL',False),('AVOID_ENTRY',True),('AVOID_HOLD',True),('RETURN',True)])
def test_global_lateral_distance_gate_applies_only_to_normal_following(node,state,allowed):
    node.latest_pose=(0.,3.6,0.)
    node.avoidance_planner.state=state
    path=[(float(i),3.6,0.) for i in range(31)]
    node.avoidance_planner.plan=Mock(return_value=dict(state=state,stage=state,points=path,
        blocked=False,active=state!='NORMAL',reason=''))
    node.avoidance_planner.needs_validation_stop=Mock(return_value=False)
    node.plan()
    assert node.avoidance_planner.plan.called==allowed
    if not allowed:assert output(node,'target_speed').data==0.
    else:assert output(node,'local_path').poses
