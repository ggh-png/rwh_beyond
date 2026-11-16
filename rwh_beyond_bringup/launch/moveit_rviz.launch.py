"""Launch only MoveIt/grasp RViz for an existing PiPER backend."""

import ast
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from rwh_beyond_bringup.camera_mount import mount_mappings, validate_mount_parent


def create_rviz(context):
    offset = ast.literal_eval(LaunchConfiguration('tcp_offset').perform(context))
    if (
        not isinstance(offset, (list, tuple)) or len(offset) != 6 or
        not all(isinstance(value, (int, float)) and math.isfinite(value) for value in offset)
    ):
        raise ValueError('tcp_offset must contain six finite XYZ/RPY values in meters/radians')

    share = Path(get_package_share_directory('moma_bringup'))
    mapping = {
        'arm_type': 'piper',
        'effector_type': 'agx_gripper',
        'revo2_type': 'left',
        'camera_nominal_tf': LaunchConfiguration('use_sim_time', default='true').perform(context).lower(),
        'tcp_offset_xyz': ' '.join(str(value) for value in offset[:3]),
        'tcp_offset_rpy': ' '.join(str(value) for value in offset[3:]),
    }
    camera_type = LaunchConfiguration('camera_type', default='d405').perform(context)
    mount = {}
    if camera_type == 'd435':
        mount = mount_mappings(*[LaunchConfiguration(name, default='').perform(context)
                                for name in ('d435_mount_parent', 'd435_mount_xyz', 'd435_mount_rpy')])
        mapping.update(mount)
    camera_share = Path(get_package_share_directory('rwh_beyond_bringup'))
    urdf = (camera_share / 'config/agx_arm_with_d435.urdf.xacro' if camera_type == 'd435'
            else share / 'config/agx_arm_with_camera.urdf.xacro')
    srdf = (camera_share / 'config/agx_arm_with_d435.srdf.xacro' if camera_type == 'd435'
            else share / 'config/agx_arm_with_camera.srdf.xacro')
    d405_args = [LaunchConfiguration(name, default='').perform(context)
                 for name in ('d405_mount_parent', 'd405_mount_xyz', 'd405_mount_rpy')]
    if camera_type == 'd405' and any(d405_args):
        mount = mount_mappings(*d405_args, camera_type='d405')
        mapping.update(mount)
        urdf = camera_share / 'config/agx_arm_with_d405.urdf.xacro'
        srdf = camera_share / 'config/agx_arm_with_d405.srdf.xacro'
    config = (
        MoveItConfigsBuilder('agx_arm', package_name='agx_arm_moveit')
        .robot_description(file_path=str(urdf),
                           mappings=mapping)
        .robot_description_semantic(
            file_path=str(srdf),
            mappings={'arm_type': 'piper', 'effector_type': 'agx_gripper', 'revo2_type': 'left', **mount})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )
    all_parameters = config.to_dict()
    if mount:
        validate_mount_parent(all_parameters['robot_description'], mount, camera_type)
    parameters = {key: all_parameters[key] for key in (
        'robot_description', 'robot_description_semantic',
        'robot_description_kinematics', 'robot_description_planning')}
    parameters['use_sim_time'] = ParameterValue(
        LaunchConfiguration('use_sim_time', default='true'), value_type=bool)
    for key in ('robot_description', 'robot_description_semantic'):
        parameters[key] = ParameterValue(parameters[key], value_type=str)
    return [Node(
        package='rviz2', executable='rviz2', name='moma_rviz', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('d405_mount_parent', default_value=''),
        DeclareLaunchArgument('d405_mount_xyz', default_value='', description='D405 mount XYZ [m]'),
        DeclareLaunchArgument('d405_mount_rpy', default_value='', description='D405 mount RPY [rad]'),
        DeclareLaunchArgument('d435_mount_parent', default_value=''),
        DeclareLaunchArgument('d435_mount_xyz', default_value='', description='D435 mount XYZ [m]'),
        DeclareLaunchArgument('d435_mount_rpy', default_value='', description='D435 mount RPY [rad]'),
        DeclareLaunchArgument('camera_type', default_value='d405', choices=['d405', 'd435']),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use false for a physical robot'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('moma_bringup'), 'config', 'depth_planning.rviz']),
            description='MoveIt and grasp debug RViz configuration'),
        DeclareLaunchArgument(
            'tcp_offset', default_value='[0.0,0.0,0.1425,0.0,0.0,0.0]',
            description='Flange-to-TCP XYZ/RPY; match the running simulator'),
        OpaqueFunction(function=create_rviz),
    ])
