"""Single operational entry point; APF proposes paths, existing Control actuates."""
import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def preflight(localization, apf, occupancy_topic='/perception/lane_occupancy', require_lane_inputs=True):
    """Refuse an occupied command chain before launching operational nodes."""
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node as ROSNode
    ctx = Context()
    probe = None
    try:
        rclpy.init(args=[], context=ctx)
        probe = ROSNode('autonomy_bringup_preflight', context=ctx)
        # DDS discovery updates the graph independently of executor callbacks.
        # This probe has no subscriptions/timers: do not instantiate rclpy's
        # global executor inside the launch process (Humble compatibility).
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            if not ctx.ok():
                raise RuntimeError('ROS context stopped during graph discovery')
            time.sleep(.05)
        topics = ['/planning/local_path', '/planning/target_speed',
                  '/planning/global_path', '/planning/apf_proposal', '/ctrl_cmd']
        reserved = {'planning_node', 'control_node', 'apf_planning_provider', 'crosswalk_detector'}
        if localization:
            topics.append('/localization/odometry')
            reserved.add('localization_node')
        if apf:
            topics.append(occupancy_topic)
            reserved.add('lane_occupancy_node')
        if apf and require_lane_inputs:
            topics += ['/perception/person_detections',
                       '/perception/pedestrian_fusion_diagnostics', '/perception/lane_geometry']
            reserved.update(['person_detector_node', 'pedestrian_fusion_shadow_node', 'lane_detection_node', 'lane_occupancy_node'])
        errors = [t + ': publisher already running' for t in topics if probe.count_publishers(t)]
        names = {name for name, namespace in probe.get_node_names_and_namespaces() if namespace == '/'}
        errors += [name + ': node already running' for name in sorted(names & reserved)]
        if not localization and probe.count_publishers('/localization/odometry') != 1:
            errors.append('retained Localization must have exactly one odometry publisher')
        if errors:
            raise RuntimeError('Bringup blocked: ' + '; '.join(errors))
    except Exception as exc:
        raise RuntimeError('Bringup preflight failed; no nodes started: ' + str(exc)) from exc
    finally:
        try:
            if probe is not None:
                probe.destroy_node()
        finally:
            ctx.try_shutdown()


def configure(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    mode = arg('planning_mode')
    if mode not in ('normal', 'apf', 'apf_simple', 'apf_lane'):
        raise RuntimeError('planning_mode must be normal, apf, apf_simple or apf_lane')
    value = arg('start_localization').lower()
    if value not in ('true', 'false'):
        raise RuntimeError('start_localization must be true or false')
    localization, apf = value == 'true', mode != 'normal'
    simple = mode in ('apf','apf_simple')
    preflight(localization, apf, arg('occupancy_topic'), not simple)
    def cfg(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)
    def node(package, executable, parameters, **kwargs):
        return Node(package=package, executable=executable, name=executable,
                    output='screen', parameters=parameters, **kwargs)
    nodes = [node('edgenode_perception', 'crosswalk_detector',
                  [cfg('edgenode_perception', 'crosswalk.yaml')])]
    if localization:
        nodes.append(node('edgenode_localization', 'localization_node',
                          [cfg('edgenode_localization', 'localization.yaml')],
                          additional_env={key: '1' for key in ['OPENBLAS_NUM_THREADS',
                                          'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS']}))
    planning = dict(enable_drive=False, apf_integration_enabled=apf,
                    apf_shadow_only=False, apf_provider_name='/apf_planning_provider')
    planning['apf_policy_mode']='apf_simple' if simple else 'apf'
    if arg('map_dir'):
        planning['mgeo_dir'] = arg('map_dir')
    nodes += [node('edgenode_planning', 'planning_node',
                   [cfg('edgenode_planning', 'planning.yaml'), planning]),
              node('edgenode_control', 'control_node', [cfg('edgenode_control', 'control.yaml')])]
    if apf:
        nodes += [
            node('edgenode_perception', 'lane_occupancy_node',
                 [dict(map_dir=arg('map_dir'), output_topic=arg('occupancy_topic'),
                       simple_evidence_path=arg('simple_evidence_path') if simple else '')]),
        ]
        if not simple:
            nodes += [
            node('edgenode_perception', 'person_detector_node',
                 [cfg('edgenode_perception', 'person_detector.yaml'), {'model_path': arg('model_path')}]),
            node('edgenode_perception', 'pedestrian_fusion_shadow_node',
                 [arg('fusion_config'),
                  {'camera_calibration_path': arg('camera_calibration_path'),
                   'validation_evidence_path': arg('fusion_evidence_path')}],
                 remappings=[('/planning/local_path', '/planning/reference_local_path')]),
            node('edgenode_perception', 'lane_detection_node',
                 [cfg('edgenode_perception', 'lane_detection.kcity.yaml'), {'enabled': True}]),
            ]
        nodes += [node('edgenode_perception', 'apf_planning_provider',
                 [dict(enabled=True, map_dir=arg('map_dir'), evidence_path=arg('evidence_path'),
                       camera_calibration_path=arg('camera_calibration_path'),
                       policy_mode='apf_simple' if simple else 'apf',
                       simple_evidence_path=arg('simple_evidence_path') if simple else '')],
                 remappings=[('/planning/local_path', '/planning/reference_local_path'),
                             ('/shadow/lane_maneuver/occupancy', arg('occupancy_topic'))]),
        ]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n,
                on_exit=[EmitEvent(event=Shutdown(reason='Autonomy member exited'))])) for n in nodes]
    return handlers + nodes


def generate_launch_description():
    args = dict(planning_mode='normal', start_localization='true',
                map_dir='/workspace/local_data/kcity_mgeo', model_path='/workspace/yolo26n.pt',
                camera_calibration_path='', fusion_evidence_path='', evidence_path='',
                occupancy_topic='/perception/lane_occupancy',
                simple_evidence_path='/workspace/local_data/apf_simple/reviewed_evidence.json',
                fusion_config=os.path.join(get_package_share_directory('edgenode_perception'),
                                           'config', 'pedestrian_fusion.yaml'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
                             + [OpaqueFunction(function=configure)])
