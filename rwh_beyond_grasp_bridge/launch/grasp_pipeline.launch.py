from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=PathJoinSubstitution([FindPackageShare('rwh_beyond_grasp_bridge'),'config','grasp_pipeline.yaml'])),
        DeclareLaunchArgument('segmentation_endpoint', default_value='tcp://127.0.0.1:5567'),
        DeclareLaunchArgument('segmentation_backend', default_value='sam3'),
        DeclareLaunchArgument('target_text', default_value='red box'),
        DeclareLaunchArgument('endpoint', default_value='tcp://127.0.0.1:5566'),
        Node(package='rwh_beyond_grasp_bridge',executable='inference_action_server',
             output='screen',parameters=[LaunchConfiguration('params_file'),{'endpoint':LaunchConfiguration('endpoint'), 'segmentation_endpoint':LaunchConfiguration('segmentation_endpoint')}]),
        Node(package='rwh_beyond_grasp_bridge',executable='grasp_coordinator',
             output='screen',parameters=[LaunchConfiguration('params_file'),{'endpoint':LaunchConfiguration('endpoint'), 'segmentation_endpoint':LaunchConfiguration('segmentation_endpoint'), 'segmentation_backend':LaunchConfiguration('segmentation_backend'), 'target_text':LaunchConfiguration('target_text')}]),
    ])
