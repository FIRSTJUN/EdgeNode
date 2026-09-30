"""Check ROS message handling and zero commands without starting ROS nodes."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav_msgs.msg import Odometry

from edgenode_planning.planning_node import PlanningNode


class PlanningNodeTest(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(latest_pose=None, map_matcher=Mock())
        for name in (
            'target_error', 'target_speed', 'state', 'current_link',
            'map_match_status', 'map_match_distance', 'map_match_heading_diff',
        ):
            setattr(self.node, name + '_pub', Mock())

    def output(self, name):
        return getattr(self.node, name + '_pub').publish.call_args.args[0].data

    def assert_stopped(self):
        self.assertEqual(self.output('target_speed'), 0.0)
        self.assertEqual(self.output('target_error'), 0.0)

    def test_wait_for_localization(self):
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.assert_not_called()
        self.assertEqual(self.output('state'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('map_match_status'), 'WAIT_FOR_LOCALIZATION')
        self.assertEqual(self.output('current_link'), '')
        self.assertTrue(math.isnan(self.output('map_match_distance')))
        self.assertTrue(math.isnan(self.output('map_match_heading_diff')))
        self.assert_stopped()

    def test_callback_stores_only_latest_pose_with_enu_yaw(self):
        for degrees in [0.0, 90.0, -90.0, 179.0, -179.0]:
            with self.subTest(degrees=degrees):
                msg = Odometry()
                msg.pose.pose.position.x = 12.0
                msg.pose.pose.position.y = -3.0
                yaw = math.radians(degrees)
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                PlanningNode.localization_callback(self.node, msg)
                self.assertEqual(self.node.latest_pose[:2], (12.0, -3.0))
                self.assertAlmostEqual(self.node.latest_pose[2], yaw)
        self.node.map_matcher.match.assert_not_called()
        self.node.target_speed_pub.publish.assert_not_called()

    def test_quaternion_with_roll_and_pitch(self):
        # Standard ROS quaternion (x, y, z, w) for roll=30, pitch=-20, yaw=70 deg.
        msg = Odometry()
        q = msg.pose.pose.orientation
        q.x = 0.30499790703769875
        q.y = 0.00879977431167487
        q.z = 0.5824308212572792
        q.w = 0.7534408929201135
        PlanningNode.localization_callback(self.node, msg)
        self.assertAlmostEqual(math.degrees(self.node.latest_pose[2]), 70.0)

    def test_match_success_publishes_actual_diagnostics_and_degrees(self):
        self.node.latest_pose = (12.0, -3.0, math.pi / 2.0)
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.assert_called_once_with(12.0, -3.0, 90.0)
        self.assertEqual(self.output('state'), 'MAP_MATCHED')
        self.assertEqual(self.output('map_match_status'), 'MATCHED')
        self.assertEqual(self.output('current_link'), 'link_42')
        self.assertEqual(self.output('map_match_distance'), 1.25)
        self.assertEqual(self.output('map_match_heading_diff'), 12.5)
        self.assert_stopped()

    def test_match_loss_clears_link_and_diagnostics_after_success(self):
        self.node.latest_pose = (12.0, -3.0, 0.0)
        self.node.map_matcher.match.return_value = {
            'link_id': 'link_42', 'distance': 1.25, 'heading_diff': 12.5,
        }
        PlanningNode.plan(self.node)
        self.node.map_matcher.match.return_value = None
        PlanningNode.plan(self.node)
        self.assertEqual(self.output('state'), 'MAP_MATCH_LOST')
        self.assertEqual(self.output('map_match_status'), 'NO_MATCH')
        self.assertEqual(self.output('current_link'), '')
        self.assertTrue(math.isnan(self.output('map_match_distance')))
        self.assertTrue(math.isnan(self.output('map_match_heading_diff')))
        self.assert_stopped()


if __name__ == '__main__':
    unittest.main()
