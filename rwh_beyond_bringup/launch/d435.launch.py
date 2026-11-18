"""Standalone D435 RGB-D inspection; no robot, MoveIt or calibrated mount TF."""
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rwh_beyond_bringup.camera_parameters import camera_parameters


def camera_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    for key in ('depth_profile', 'color_profile'):
        if not re.fullmatch(r'[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*', value(key)):
            raise ValueError(key + ' must be WIDTHxHEIGHTxFPS')
    # Shared pipeline aliases preserve the existing BT, cloud filter and RViz inputs.
    topics = {
        'color/image_raw': 'color/image_raw', 'color/camera_info': 'color/camera_info',
        'depth/image_rect_raw': 'depth/image_raw', 'depth/camera_info': 'depth/raw_camera_info',
        'aligned_depth_to_color/image_raw': 'depth/image_rect_raw',
        'aligned_depth_to_color/camera_info': 'depth/camera_info',
        'depth/color/points': 'depth/color/points',
    }
    remaps = [('~/' + source, '/camera/' + target) for source, target in topics.items()] if value('pipeline_topics') == 'true' else []
    params = camera_parameters(value('camera_params_file'), '/camera/d435')
    params.update({
        'use_sim_time': False, 'serial_no': ParameterValue(value('serial_no'), value_type=str),
        'depth_module.depth_profile': value('depth_profile'),
        'rgb_camera.color_profile': value('color_profile'),
    })
    nodes = [Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='d435', output='screen',
        parameters=[params], remappings=remaps, respawn=True, respawn_delay=3.0)]
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='d435_rviz',
                          arguments=['-d', value('rviz_config')],
                          parameters=[{'use_sim_time': False}], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pipeline_topics', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('serial_no', default_value='',
                              description='D435 serial; empty selects a matching device'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15'),
        DeclareLaunchArgument('color_profile', default_value='640x480x15'),
        DeclareLaunchArgument('camera_params_file', default_value=str(
            Path(get_package_share_directory('rwh_beyond_bringup')) / 'config/bt_demo.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(
            Path(get_package_share_directory('rwh_beyond_bringup')) / 'rviz/d435.rviz')),
        OpaqueFunction(function=camera_nodes),
    ])
