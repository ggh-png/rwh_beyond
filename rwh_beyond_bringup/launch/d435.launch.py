"""Standalone D435 RGB-D inspection; no robot, MoveIt or calibrated mount TF."""
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    nodes = [Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='d435', output='screen',
        parameters=[{
            'use_sim_time': False, 'camera_name': 'd435', 'device_type': 'd435',
            'serial_no': value('serial_no'),
            'enable_depth': True, 'enable_color': True,
            'enable_infra1': False, 'enable_infra2': False, 'enable_sync': True,
            'depth_module.depth_profile': value('depth_profile'),
            'rgb_camera.color_profile': value('color_profile'),
            'align_depth.enable': True, 'pointcloud.enable': True,
            'pointcloud.stream_filter': 2, 'pointcloud.stream_index_filter': 0,
            'publish_tf': True, 'base_frame_id': 'link',
            'depth_qos': 'SENSOR_DATA', 'color_qos': 'SENSOR_DATA',
        }], remappings=remaps, respawn=True, respawn_delay=3.0)]
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
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(
            Path(get_package_share_directory('rwh_beyond_bringup')) / 'rviz/d435.rviz')),
        OpaqueFunction(function=camera_nodes),
    ])
