"""Start only the rwh_beyond_sam3_bridge ROS interface; never starts a GPU model."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rwh_beyond_sam3_bridge'), 'config', 'bridge.yaml'])),
        DeclareLaunchArgument('endpoint', default_value='tcp://127.0.0.1:5567'),
        Node(package='rwh_beyond_sam3_bridge', executable='segmentation_action_server', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'endpoint': LaunchConfiguration('endpoint')}]),
    ])
