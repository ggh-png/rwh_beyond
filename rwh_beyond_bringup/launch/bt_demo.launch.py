"""Launch the arm behavior-tree demo navigator and lifecycle manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rwh_beyond_bringup'),
                'params',
                'bt_demo_params.yaml',
            ]),
            description='BT navigator and lifecycle manager parameter YAML',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='bt_lifecycle_manager',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
