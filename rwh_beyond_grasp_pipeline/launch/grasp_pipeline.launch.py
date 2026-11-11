"""Compose independent SAM, GraspGenX and robot orchestration ROS nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config(package, filename):
    return PathJoinSubstitution([FindPackageShare(package), 'config', filename])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config(
            'rwh_beyond_grasp_pipeline', 'grasp_pipeline.yaml')),
        DeclareLaunchArgument('sam3_params_file', default_value=config(
            'rwh_beyond_sam3_bridge', 'bridge.yaml')),
        DeclareLaunchArgument('graspgenx_params_file', default_value=config(
            'rwh_beyond_graspgenx_bridge', 'bridge.yaml')),
        DeclareLaunchArgument('segmentation_endpoint', default_value='tcp://127.0.0.1:5567'),
        DeclareLaunchArgument('endpoint', default_value='tcp://127.0.0.1:5566'),
        DeclareLaunchArgument('segmentation_backend', default_value='sam3'),
        DeclareLaunchArgument('target_text', default_value='red box'),
        Node(package='rwh_beyond_sam3_bridge', executable='segmentation_action_server',
             output='screen', parameters=[LaunchConfiguration('sam3_params_file'),
                 {'endpoint': LaunchConfiguration('segmentation_endpoint')}]),
        Node(package='rwh_beyond_graspgenx_bridge', executable='inference_action_server',
             output='screen', parameters=[LaunchConfiguration('graspgenx_params_file'),
                 {'endpoint': LaunchConfiguration('endpoint')}]),
        Node(package='rwh_beyond_grasp_pipeline', executable='grasp_coordinator',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                 {'segmentation_backend': LaunchConfiguration('segmentation_backend'),
                  'target_text': LaunchConfiguration('target_text')}]),
    ])
