from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rwh_beyond_grasp_bridge'), 'config', 'bridge.yaml'])),
        DeclareLaunchArgument('endpoint', default_value='tcp://127.0.0.1:5556'),
        DeclareLaunchArgument('segmentation_endpoint', default_value='tcp://127.0.0.1:5567'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='Start synthetic loopback ZMQ server for transport testing only'),
        Node(package='rwh_beyond_grasp_bridge', executable='mock_zmq_server',
             condition=IfCondition(LaunchConfiguration('mock')), output='screen',
             arguments=['--endpoint', LaunchConfiguration('endpoint')]),
        Node(package='rwh_beyond_grasp_bridge', executable='inference_action_server',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                                         {'endpoint': LaunchConfiguration('endpoint'), 'segmentation_endpoint': LaunchConfiguration('segmentation_endpoint')}])])
