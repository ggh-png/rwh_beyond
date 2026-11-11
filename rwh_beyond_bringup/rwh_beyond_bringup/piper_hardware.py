"""PiPER hardware composition using the MuJoCo MoveIt planning baseline.

The physical CAN driver and trajectory command transport remain hardware-specific.
Camera geometry is opt-in because the simulation mount is not a calibration.
"""
import ast
import math
import os
import tempfile

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


DEFAULT_TCP = '[0.0,0.0,0.1425,0.0,0.0,0.0]'


def moveit_parameters(tcp_offset, effector_type='agx_gripper', camera_model=False,
                      point_cloud_topic='/camera/depth/color/points', octomap=True):
    """Match simulation planning parameters, using physical time and feedback."""
    offset = ast.literal_eval(tcp_offset)
    if (not isinstance(offset, (list, tuple)) or len(offset) != 6 or
            not all(isinstance(x, (int, float)) and math.isfinite(x) for x in offset)):
        raise ValueError('tcp_offset must contain six finite XYZ/RPY values')
    if effector_type not in ('agx_gripper', 'none'):
        raise ValueError('The PiPER baseline supports agx_gripper or none')
    sim_share = Path(get_package_share_directory('moma_bringup'))
    mapping = dict(arm_type='piper', effector_type=effector_type, revo2_type='left',
                   tcp_offset_xyz=' '.join(str(x) for x in offset[:3]),
                   tcp_offset_rpy=' '.join(str(x) for x in offset[3:]))
    urdf = (str(sim_share / 'config/agx_arm_with_camera.urdf.xacro') if camera_model
            else 'config/agx_arm.urdf.xacro')
    srdf = (str(sim_share / 'config/agx_arm_with_camera.srdf.xacro') if camera_model
            else 'config/agx_arm.srdf.xacro')
    config = (MoveItConfigsBuilder('agx_arm', package_name='agx_arm_moveit')
              .robot_description(file_path=urdf, mappings=mapping)
              .robot_description_semantic(file_path=srdf, mappings={
                  'arm_type': 'piper', 'effector_type': effector_type, 'revo2_type': 'left'})
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .sensors_3d(file_path=str(sim_share / 'config/sensors_3d.yaml'))
              .trajectory_execution(file_path='config/moveit_controllers_' +
                                    ('gripper' if effector_type == 'agx_gripper' else 'none') + '.yaml')
              .to_moveit_configs())
    params = config.to_dict()
    params.update({
        'use_sim_time': False, 'moveit_manage_controllers': False,
        'allow_trajectory_execution': True, 'publish_robot_description': True,
        'publish_robot_description_semantic': True, 'publish_planning_scene': True,
        'publish_geometry_updates': True, 'publish_state_updates': True,
        'publish_transforms_updates': True, 'monitor_dynamics': False,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'trajectory_execution': {'allowed_execution_duration_scaling': 3.0,
                                 'allowed_goal_duration_margin': 2.0},
        'octomap_frame': 'world', 'octomap_resolution': 0.01})
    params['depth_cloud']['point_cloud_topic'] = point_cloud_topic
    if not octomap:
        params.pop('sensors', None)
    return params


def controller_parameters(namespace, effector_type):
    """Keep the official mock trajectory-to-command transport for the CAN driver."""
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    controllers = {'arm_controller': [f'joint{i}' for i in range(1, 7)]}
    if effector_type == 'agx_gripper':
        controllers['gripper_controller'] = ['gripper']
    result = {prefix + '/controller_manager': {'ros__parameters': {
        'update_rate': 200,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        **{name: {'type': 'joint_trajectory_controller/JointTrajectoryController'}
           for name in controllers}}}}
    for name, joints in controllers.items():
        result[prefix + '/' + name] = {'ros__parameters': {
            'joints': joints, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'allow_nonzero_velocity_at_trajectory_end': False}}
    return result, list(controllers)


def hardware_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    namespace = value('namespace')
    effector = value('effector_type')
    params = moveit_parameters(
        value('tcp_offset'), effector, value('camera_model') == 'true',
        value('point_cloud_topic'), value('octomap') == 'true')
    description = ParameterValue(params['robot_description'], value_type=str)
    params['robot_description'] = description
    params['robot_description_semantic'] = ParameterValue(
        params['robot_description_semantic'], value_type=str)
    ctrl_params, controllers = controller_parameters(namespace, effector)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                     prefix='piper_controllers_', delete=False) as config_file:
        yaml.safe_dump(ctrl_params, config_file)
        controllers_file = config_file.name
    driver_args = {name: LaunchConfiguration(name) for name in (
        'namespace', 'can_port', 'effector_type', 'auto_enable', 'speed_percent',
        'log_level', 'fw_version', 'tcp_offset')}
    driver_args.update(arm_type='piper', control_enabled=(
        'false' if value('auto_control_gate') == 'true' else 'true'))
    driver = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
        get_package_share_directory('agx_arm_ctrl')) / 'launch/start_single_agx_arm.launch.py')),
        launch_arguments=driver_args.items())
    feedback = value('feedback_topic')
    library = Path(get_package_prefix('moveit_simple_controller_manager')) / 'lib/libmoveit_simple_controller_manager.so'
    preload = str(library) + (':' + os.environ['LD_PRELOAD'] if os.environ.get('LD_PRELOAD') else '')
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': False, 'robot_description': description}],
             remappings=[('joint_states', feedback)], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[params],
             remappings=[('joint_states', feedback)], output='screen',
             additional_env={'LD_PRELOAD': preload}),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description, 'use_sim_time': False}, controllers_file],
             remappings=[('joint_states', 'control/joint_states')], output='screen'),
    ]
    manager = ('/' + namespace.strip('/') if namespace.strip('/') else '') + '/controller_manager'
    actions.append(Node(package='controller_manager', executable='spawner',
                        arguments=['joint_state_broadcaster', *controllers,
                                   '--controller-manager', manager], output='screen'))
    if value('auto_control_gate') == 'true':
        # Preserve the official arm execution gate; do not open it for gripper-only
        # trajectories while the transport's arm state could still be at its defaults.
        actions.append(Node(package='agx_arm_moveit', executable='agx_arm_control_gate',
                            parameters=[{'status_topics': [
                                'arm_controller/follow_joint_trajectory/_action/status'],
                                'gate_service_name': 'control_enable'}], output='screen'))
    if value('use_rviz') == 'true':
        rviz_params = {k: params[k] for k in (
            'use_sim_time', 'robot_description', 'robot_description_semantic',
            'robot_description_kinematics', 'robot_description_planning')}
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[rviz_params],
                            arguments=['-d', value('rviz_config')],
                            remappings=[('joint_states', feedback)], output='screen'))
    return [driver, GroupAction(actions=[PushRosNamespace(namespace), *actions])]
