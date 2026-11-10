"""Launch MuJoCo, MoveIt, and the arm behavior-tree demo navigator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gripper_contact_force_threshold', default_value='1.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('moma_bringup'),
                'launch',
                'simulation.launch.py',
            )),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'moveit': 'true',
                'gripper_contact_force_threshold': LaunchConfiguration(
                    'gripper_contact_force_threshold'),
            }.items(),
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'global_frame': 'base_link',
                'robot_base_frame': 'base_link',
                'odom_topic': '/odom',
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                # BT nodes connect to MoveIt and MuJoCo during activation.
                # Allow their action servers to finish starting (milliseconds).
                'wait_for_service_timeout': 30000,
                'action_server_result_timeout': 900.0,
                'plugin_lib_names': ['rwh_beyond_bt_nodes'],
                'navigators': ['demo'],
                'demo': {
                    'plugin': 'rwh_beyond_navigator::DemoNavigator',
                },
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='bt_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['bt_navigator'],
            }],
        ),
    ])
