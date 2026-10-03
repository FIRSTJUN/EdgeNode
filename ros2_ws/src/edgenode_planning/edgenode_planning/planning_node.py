"""C-track map pursuit with Hansung camera confidence and obstacle supervision."""
import math
import time
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Float32MultiArray, String
from morai_ros2_msgs.msg import EgoVehicleStatus, CollisionData
from edgenode_planning.fsm import PlanningConfig, PlanningFSM, LaneInput
from edgenode_planning.input_adapter import PerceptionInputs, obstacle_input
from edgenode_planning.map_tracker import MapTracker


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        defaults = asdict(PlanningConfig())
        defaults.update({
            'cruise_speed': 6.0, 'avoid_distance': 3.5,
            'map_file': '/workspace/local_data/c_track_mgeo/link_set.json',
            'turn_preference': 'straight', 'enable_drive': False,
            'drive_duration_sec': 60.0,
            'wheelbase_m': 1.04, 'max_wheel_angle_rad': .49,
            'max_cross_track_m': 1.5, 'min_lookahead_m': 2.5,
            'lane_confidence_enter': .55, 'lane_confidence_exit': .40,
            'lane_lost_grace_sec': .4, 'lane_degraded_speed': 2.0,
            'curve_speed_kmh': 3.5, 'status_timeout_sec': .6,
            'target_error_topic': '/planning/target_error',
            'target_speed_topic': '/planning/target_speed',
            'state_topic': '/planning/state',
            'camera_steering_weight': .10,
        })
        for k, value in defaults.items():
            self.declare_parameter(k, value)
        self.inputs = PerceptionInputs()
        self.fsm = PlanningFSM(PlanningConfig(**{
            k: self.get_parameter(k).value for k in asdict(PlanningConfig())}))
        self.tracker = MapTracker(self.p('map_file'), self.p('wheelbase_m'),
                                  self.p('max_wheel_angle_rad'), self.p('turn_preference'))
        self.ego = None
        self.ego_stamp = 0.
        self.collision_stamp = 0.
        self.collision_latched = False
        self.lane_confident = False
        self.drive_started = None
        self.last_state = None
        self.last_report = 0.
        self.create_subscription(Float32, '/perception/lane_error', self.error_cb, 10)
        self.create_subscription(Float32, '/perception/lane_confidence', self.confidence_cb, 10)
        self.create_subscription(Float32MultiArray, '/perception/obstacle', self.obstacle_cb, 10)
        self.create_subscription(EgoVehicleStatus, '/ego_vehicle_status', self.ego_cb, qos_profile_sensor_data)
        self.create_subscription(CollisionData, '/collision_data', self.collision_cb, qos_profile_sensor_data)
        self.error_pub = self.create_publisher(Float32, self.p('target_error_topic'), 10)
        self.speed_pub = self.create_publisher(Float32, self.p('target_speed_topic'), 10)
        self.steer_pub = self.create_publisher(Float32, '/planning/target_steering', 10)
        self.curve_pub = self.create_publisher(Float32, '/planning/curvature', 10)
        self.state_pub = self.create_publisher(String, self.p('state_topic'), 10)
        self.diag_pub = self.create_publisher(String, '/planning/diagnostics', 10)
        self.create_timer(.05, self.plan)
        self.get_logger().info('C-track planner uses MORAI EgoVehicleStatus local ENU pose; camera + LiDAR must be fresh')

    def p(self, name):
        return self.get_parameter(name).value

    def error_cb(self, m):
        self.inputs.error = (float(m.data), time.monotonic())

    def confidence_cb(self, m):
        c = float(m.data)
        if not math.isfinite(c) or not 0 <= c <= 1:
            self.lane_confident = False
        elif c >= self.p('lane_confidence_enter'):
            self.lane_confident = True
        elif c <= self.p('lane_confidence_exit'):
            self.lane_confident = False
        self.inputs.confidence = (c, time.monotonic())

    def obstacle_cb(self, m):
        self.inputs.obstacle = obstacle_input(m.data, time.monotonic())

    def ego_cb(self, m):
        self.ego = m
        self.ego_stamp = time.monotonic()

    def collision_cb(self, m):
        self.collision_stamp = time.monotonic()
        if m.collision_object:
            self.collision_latched = True

    def publish(self, state, speed=0., steer=0., error=0., curvature=0., detail=''):
        for pub, value in [(self.error_pub, error), (self.speed_pub, speed),
                           (self.steer_pub, steer), (self.curve_pub, curvature)]:
            pub.publish(Float32(data=float(value)))
        self.state_pub.publish(String(data=state))
        self.diag_pub.publish(String(data=detail))
        if state != self.last_state:
            self.get_logger().info(f'{state}: {detail}')
            self.last_state = state

    def plan(self):
        now = time.monotonic()
        # Every drive activation has a finite lease. The controller watchdog
        # still stops independently if this planner itself stalls or exits.
        if not self.p('enable_drive'):
            self.drive_started = None
        else:
            if self.drive_started is None:
                self.drive_started = now
            duration = float(self.p('drive_duration_sec'))
            if not math.isfinite(duration) or not 0 < duration <= 300:
                return self.publish('INVALID_DRIVE_DURATION')
            if now-self.drive_started >= duration:
                return self.publish('DRIVE_SESSION_COMPLETE', detail=f'{duration:.0f}s drive session ended; stopped')
        if self.collision_latched:
            return self.publish('COLLISION_STOP', detail='Collision latched; inspect scene and restart planner')
        if self.ego is None or now-self.ego_stamp >= self.p('status_timeout_sec'):
            return self.publish('WAIT_FOR_EGO')
        lane = self.inputs.lane()
        obstacle = self.inputs.obstacle
        if lane is None or not 0 <= now-lane.received_at < self.p('lane_timeout_sec'):
            self.lane_confident = False
            return self.publish('CAMERA_TIMEOUT')
        if not (math.isfinite(lane.error) and -1 <= lane.error <= 1 and
                math.isfinite(lane.confidence) and 0 <= lane.confidence <= 1):
            return self.publish('INVALID_CAMERA')
        if obstacle is None or not 0 <= now-obstacle.received_at < self.p('obstacle_timeout_sec'):
            return self.publish('LIDAR_TIMEOUT')
        e = self.ego
        v = math.sqrt(e.velocity.x**2+e.velocity.y**2+e.velocity.z**2)
        track = self.tracker.track(e.position.x, e.position.y, math.radians(e.heading),
                                   v, self.p('min_lookahead_m'), self.p('max_cross_track_m'))
        if track is None and not self.p('enable_drive'):
            # A stopped simulator may be reset/repositioned between sessions.
            # Reacquire only while disabled; active driving keeps route lock.
            self.tracker.current = None
            self.tracker.progress = 0.0
            self.tracker.next_links = []
            self.tracker._previous_link = None
            track = self.tracker.track(e.position.x, e.position.y, math.radians(e.heading),
                                      v, self.p('min_lookahead_m'), self.p('max_cross_track_m'))
        if track is None:
            return self.publish('MAP_OR_POSE_INVALID', detail='No connected forward route within pose limits')
        # Reference FSM supplies obstacle STOP priority. Map geometry is the
        # lateral observation through unmarked intersections, never old pixels.
        output = self.fsm.step(LaneInput(float(max(-1, min(1, track.cross_track))), 1., now),
                               obstacle, now=now)
        distance = obstacle.front_distance
        stop_distance = max(self.p('stop_distance'), 1.0+v*.6+v*v/(2*1.8))
        if output.state.value == 'STOP' or (distance is not None and distance <= stop_distance):
            return self.publish('OBSTACLE_STOP', curvature=track.curvature,
                                detail=f'front={distance}, required={stop_distance:.2f}m')
        speed = float(self.p('cruise_speed'))
        steer = track.steering
        # Only a strong camera pair can add a small bounded correction on a
        # straight. A single stripe must not steer across a connected map lane.
        if self.lane_confident and lane.confidence >= .65 and abs(track.steering) < .25 and abs(track.cross_track) < .5:
            correction = max(-.06, min(.06, -lane.error*self.p('camera_steering_weight')))
            steer = max(-.85, min(.85, steer+correction))
        if abs(steer) > .22 or abs(track.heading_error) > .15:
            speed = min(speed, self.p('curve_speed_kmh'))
        if abs(steer) > .55:
            speed = min(speed, 2.5)
        if abs(track.cross_track) > .5 or abs(track.heading_error) > .4:
            speed = min(speed, 1.0)
        if not self.lane_confident:
            speed = min(speed, self.p('lane_degraded_speed'))
        if distance is not None and distance < 10.:
            speed = min(speed, max(0., (distance-stop_distance)*1.0))
        # If a directed route terminates, brake before its final point.
        speed = min(speed, math.sqrt(max(0., 2*.8*(track.remaining-1.0)))*3.6)
        detail = (f'link={track.link_id} cte={track.cross_track:+.3f}m '
                  f'heading={math.degrees(track.heading_error):+.1f}deg '
                  f'lane={lane.error:+.3f}/{lane.confidence:.2f} '
                  f'front={distance} steer={steer:+.3f} speed={v*3.6:.2f}/{speed:.2f}')
        state = 'MAP_LANE_FOLLOW' if self.lane_confident else 'MAP_FOLLOW_CAMERA_DEGRADED'
        if not self.p('enable_drive'):
            state, speed = 'PREVIEW_STOP', 0.
        self.publish(state, speed, steer, lane.error, track.curvature, detail)
        if now-self.last_report >= 2.:
            self.get_logger().info(detail)
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
