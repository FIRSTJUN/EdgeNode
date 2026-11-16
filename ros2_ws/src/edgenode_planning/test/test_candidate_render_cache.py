"""Large diagnostic geometry must not delay live safety callbacks on HOLD."""
from unittest.mock import patch

from visualization_msgs.msg import Marker
from edgenode_planning.avoidance_visualization import candidate_markers
from test_static_obstacle_node import node
from test_static_obstacle_planner import setup, enter


def test_hold_text_updates_do_not_rebuild_or_republish_fixed_candidates(node):
    planner, point, obstacle = setup()
    result = enter(planner, point, obstacle)
    with patch('edgenode_planning.planning_node.time.monotonic', return_value=1.) as clock, \
            patch('edgenode_planning.planning_node.candidate_markers', wraps=candidate_markers) as render:
        node.publish_avoidance(result)
        assert render.call_count == 1
        bulk = node.candidate_pub.publish.call_args.args[0]
        assert any(m.points for m in bulk.markers)
        clock.return_value = 1.1
        hold = dict(result, state='SENSOR_HOLD', stage='AVOID_ENTRY',
                    reason='S4_SENSOR_SENSOR_INPUT_TIMEOUT', blocked=True, points=())
        node.publish_avoidance(hold)
        state = node.candidate_pub.publish.call_args.args[0]
        assert render.call_count == 1 and [m.id for m in state.markers] == [0]
        assert 'S4_SENSOR_SENSOR_INPUT_TIMEOUT' in state.markers[0].text
        clock.return_value = 1.6
        node.publish_avoidance(hold)
        assert render.call_count == 1
        assert any(m.points for m in node.candidate_pub.publish.call_args.args[0].markers)
        # New candidates are published immediately, including old-ID deletion.
        clock.return_value = 1.61
        node.publish_avoidance(dict(result, candidates=()))
        assert render.call_count == 2
        assert any(m.action == Marker.DELETE for m in node.candidate_pub.publish.call_args.args[0].markers)
