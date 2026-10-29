"""RWH lifecycle/component service server, independently of Nav2 and hardware."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    share = get_package_share_directory('rwh_beyond_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0724'),
        DeclareLaunchArgument('autostart', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('server_params_file', default_value=os.path.join(share, 'params', 'rwh_beyond.yaml')),
        Node(package='rwh_beyond', executable='rwh_beyond_server', name='beyond_server',
             namespace=namespace, output='screen', parameters=[RewrittenYaml(
                 source_file=LaunchConfiguration('server_params_file'), root_key=namespace,
                 param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')}, convert_types=True)]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_beyond', namespace=namespace, output='screen',
             parameters=[{'use_sim_time': time,
                          'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                          'node_names': ['beyond_server']}]),
    ])
