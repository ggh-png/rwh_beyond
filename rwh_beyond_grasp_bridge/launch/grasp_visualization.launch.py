"""Open only the grasp observation viewer against an existing ROS graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('rwh_beyond_grasp_bridge'), 'rviz', 'grasp_perception.rviz'])),
        Node(
            package='rviz2', executable='rviz2', name='grasp_perception_rviz',
            output='screen', arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
