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
    config = (
        MoveItConfigsBuilder('agx_arm', package_name='agx_arm_moveit')
        .robot_description(file_path=str(share / 'config/agx_arm_with_camera.urdf.xacro'),
                           mappings=mapping)
        .robot_description_semantic(
            file_path=str(share / 'config/agx_arm_with_camera.srdf.xacro'),
            mappings={'arm_type': 'piper', 'effector_type': 'agx_gripper', 'revo2_type': 'left'})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )
    all_parameters = config.to_dict()
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
