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
from .camera_mount import mount_mappings, validate_mount_parent


DEFAULT_TCP = '[0.0,0.0,0.1425,0.0,0.0,0.0]'


def moveit_parameters(tcp_offset, effector_type='agx_gripper', camera_model=False,
                      point_cloud_topic='/camera/depth/color/points', octomap=True,
                      nominal_camera_tf=True, camera_type='d405',
                      d435_mount_parent='', d435_mount_xyz='', d435_mount_rpy='',
                      d405_mount_parent='', d405_mount_xyz='', d405_mount_rpy=''):
    """Match simulation planning parameters, using physical time and feedback."""
    offset = ast.literal_eval(tcp_offset)
    if (not isinstance(offset, (list, tuple)) or len(offset) != 6 or
            not all(isinstance(x, (int, float)) and math.isfinite(x) for x in offset)):
        raise ValueError('tcp_offset must contain six finite XYZ/RPY values')
    if effector_type not in ('agx_gripper', 'none'):
        raise ValueError('The PiPER baseline supports agx_gripper or none')
    if camera_type not in ('d405', 'd435'):
        raise ValueError('camera_type must be d405 or d435')
    sim_share = Path(get_package_share_directory('moma_bringup'))
    mapping = dict(arm_type='piper', effector_type=effector_type, revo2_type='left',
                   tcp_offset_xyz=' '.join(str(x) for x in offset[:3]),
                   tcp_offset_rpy=' '.join(str(x) for x in offset[3:]))
    mapping['camera_nominal_tf'] = str(nominal_camera_tf).lower()
    urdf = (str(sim_share / 'config/agx_arm_with_camera.urdf.xacro') if camera_model
            else 'config/agx_arm.urdf.xacro')
    srdf = (str(sim_share / 'config/agx_arm_with_camera.srdf.xacro') if camera_model
            else 'config/agx_arm.srdf.xacro')
    mount = {}
    if camera_model and camera_type == 'd435':
        mount = mount_mappings(d435_mount_parent, d435_mount_xyz, d435_mount_rpy)
        mapping.update(mount)
        camera_share = Path(get_package_share_directory('rwh_beyond_bringup'))
        urdf = str(camera_share / 'config/agx_arm_with_d435.urdf.xacro')
        srdf = str(camera_share / 'config/agx_arm_with_d435.srdf.xacro')
    if camera_model and camera_type == 'd405' and any((d405_mount_parent, d405_mount_xyz, d405_mount_rpy)):
        mount = mount_mappings(d405_mount_parent, d405_mount_xyz, d405_mount_rpy, camera_type='d405')
        mapping.update(mount)
        camera_share = Path(get_package_share_directory('rwh_beyond_bringup'))
        urdf = str(camera_share / 'config/agx_arm_with_d405.urdf.xacro')
        srdf = str(camera_share / 'config/agx_arm_with_d405.srdf.xacro')
    config = (MoveItConfigsBuilder('agx_arm', package_name='agx_arm_moveit')
              .robot_description(file_path=urdf, mappings=mapping)
              .robot_description_semantic(file_path=srdf, mappings={
                  'arm_type': 'piper', 'effector_type': effector_type, 'revo2_type': 'left', **mount})
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .sensors_3d(file_path=str(sim_share / 'config/sensors_3d.yaml'))
              .trajectory_execution(file_path='config/moveit_controllers_' +
                                    ('gripper' if effector_type == 'agx_gripper' else 'none') + '.yaml')
              .to_moveit_configs())
    if mount:
        validate_mount_parent(config.to_dict()['robot_description'], mount, camera_type)
    tolerance_path = Path(get_package_share_directory('rwh_beyond_bringup')) / 'config/bt_demo.yaml'
    shared_settings = yaml.safe_load(tolerance_path.read_text())
    bt_settings = shared_settings['bt_navigator']['ros__parameters']['bt_motion']
    octomap_enabled = shared_settings.get('/**', {}).get('ros__parameters', {}).get(
        'octomap_enabled', True)
    if type(octomap_enabled) is not bool:
        raise ValueError('octomap_enabled must be a YAML boolean')
    execution = shared_settings.get('move_group', {}).get('ros__parameters', {}).get(
        'trajectory_execution', {})
    execution_scaling = execution.get('allowed_execution_duration_scaling', 3.0)
    execution_margin = execution.get('allowed_goal_duration_margin', 2.0)
    for name, setting, minimum in (
            ('allowed_execution_duration_scaling', execution_scaling, 1.0),
            ('allowed_goal_duration_margin', execution_margin, 0.0)):
        if type(setting) not in (int, float) or not math.isfinite(setting) or setting < minimum:
            raise ValueError(name + ' must be finite and >= ' + str(minimum))
    start_tolerance = bt_settings['planning_start_bounds_tolerance_deg']
    if not math.isfinite(start_tolerance) or not 0 < start_tolerance <= 180.0:
        raise ValueError('planning_start_bounds_tolerance_deg must be in (0, 180] degrees')
    start_tolerance = math.radians(start_tolerance)
    params = config.to_dict()
    ompl_settings = shared_settings.get('move_group', {}).get('ros__parameters', {}).get('ompl', {})
    planner_overrides = ompl_settings.get('planner_configs', {})
    if not isinstance(planner_overrides, dict):
        raise ValueError('move_group.ompl.planner_configs must be a mapping')
    for name, override in planner_overrides.items():
        if name not in params['ompl']['planner_configs'] or not isinstance(override, dict):
            raise ValueError('move_group.ompl.planner_configs must override registered planners')
        if set(override) - {'range'}:
            raise ValueError('Only planner range can be overridden in move_group.ompl.planner_configs')
        value = override.get('range', 0.0)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError('OMPL planner range must be finite and >= 0 (0 means automatic)')
        params['ompl']['planner_configs'][name].update(range=float(value))
    arm_ompl = ompl_settings.get('arm', {})
    if arm_ompl:
        planners = arm_ompl.get('planner_configs', [])
        if not isinstance(planners, list) or not planners or any(
                name not in params['ompl']['planner_configs'] for name in planners):
            raise ValueError('move_group.ompl.arm.planner_configs must list registered planners')
        default_planner = arm_ompl.get('default_planner_config')
        if default_planner is not None and default_planner not in planners:
            raise ValueError('default_planner_config must be listed in arm.planner_configs')
        if type(arm_ompl.get('enforce_joint_model_state_space')) is not bool:
            raise ValueError('enforce_joint_model_state_space must be a YAML boolean')
        fraction = arm_ompl.get('longest_valid_segment_fraction')
        if type(fraction) not in (int, float) or not math.isfinite(fraction) or not 0 < fraction <= 1:
            raise ValueError('longest_valid_segment_fraction must be in (0, 1]')
        params['ompl'].setdefault('arm', {}).update(arm_ompl)
    params.update({
        'use_sim_time': False, 'moveit_manage_controllers': False,
        'allow_trajectory_execution': True, 'publish_robot_description': True,
        'publish_robot_description_semantic': True, 'publish_planning_scene': True,
        'publish_geometry_updates': True, 'publish_state_updates': True,
        'publish_transforms_updates': True, 'monitor_dynamics': False,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'trajectory_execution': {'allowed_execution_duration_scaling': float(execution_scaling),
                                 'allowed_goal_duration_margin': float(execution_margin),
                                 'allowed_start_tolerance': start_tolerance},
        'octomap_frame': 'world', 'octomap_resolution': 0.01})
    params['depth_cloud']['point_cloud_topic'] = point_cloud_topic
    if not octomap or not octomap_enabled:
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
    transport = value('execution_transport')
    if transport not in ('official', 'feedback'):
        raise ValueError('execution_transport must be official or feedback')
    namespace = value('namespace')
    effector = value('effector_type')
    params = moveit_parameters(
        value('tcp_offset'), effector, value('camera_model') == 'true',
        value('point_cloud_topic'), value('octomap') == 'true', nominal_camera_tf=False,
        camera_type=value('camera_type'),
        d405_mount_parent=value('d405_mount_parent'),
        d405_mount_xyz=value('d405_mount_xyz'), d405_mount_rpy=value('d405_mount_rpy'),
        d435_mount_parent=value('d435_mount_parent'),
        d435_mount_xyz=value('d435_mount_xyz'), d435_mount_rpy=value('d435_mount_rpy'))
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
        'false' if transport == 'feedback' or value('auto_control_gate') == 'true' else 'true'))
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
    ]
    if value('octomap') == 'true':
        actions.append(Node(package='rwh_beyond_grasp_pipeline', executable='target_cloud_filter',
                            parameters=[{'use_sim_time': False}], output='screen'))
    if transport == 'feedback':
        settings = yaml.safe_load((Path(get_package_share_directory('rwh_beyond_bringup')) /
            'config/bt_demo.yaml').read_text())['bt_navigator']['ros__parameters']['bt_motion']
        actions.append(Node(package='rwh_beyond_grasp_pipeline', executable='physical_trajectory',
                            parameters=[{'use_sim_time': False, 'initial_position_tolerance':
                                params['trajectory_execution']['allowed_start_tolerance'],
                                'gripper_close_width': settings['gripper_close_width'],
                                'gripper_close_duration': settings['gripper_close_duration']}], output='screen'))
    else:
        actions.append(Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description, 'use_sim_time': False}, controllers_file],
             remappings=[('joint_states', 'control/joint_states')], output='screen'))
        manager = ('/' + namespace.strip('/') if namespace.strip('/') else '') + '/controller_manager'
        actions.append(Node(package='controller_manager', executable='spawner',
                            arguments=['joint_state_broadcaster', *controllers,
                                       '--controller-manager', manager], output='screen'))
    if transport == 'official' and value('auto_control_gate') == 'true':
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
    result = [driver, GroupAction(actions=[PushRosNamespace(namespace), *actions])]
    if value('start_sensors') == 'true':
        result.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory(
                'rwh_beyond_bringup')) / 'launch/sensor.launch.py')),
            launch_arguments={
                'mode': 'real', 'namespace': namespace,
                'params_file': value('sensor_params_file'), 'use_sim_time': 'false',
                'enable_lidar': value('enable_lidar'),
                'enable_d405': value('enable_d405') if value('camera_type') == 'd405' else 'false',
                'd405_mount_parent': value('d405_mount_parent'),
                'd405_mount_xyz': value('d405_mount_xyz'), 'd405_mount_rpy': value('d405_mount_rpy'),
                'd405_serial': value('d405_serial'), 'd405_profile': value('d405_profile'),
                'publish_d405_mount_tf': str(value('camera_model') != 'true').lower(),
            }.items())]))
    if (value('start_sensors') == 'true' and value('camera_type') == 'd435'
            and value('enable_d435') == 'true'):
        result.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
            get_package_share_directory('rwh_beyond_bringup')) / 'launch/d435.launch.py')),
            launch_arguments={
                'serial_no': value('d435_serial'), 'depth_profile': value('d435_depth_profile'),
                'color_profile': value('d435_color_profile'), 'pipeline_topics': 'true',
                'rviz': 'false',
            }.items())]))
    return result
