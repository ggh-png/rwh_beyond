"""Start the SAM and GraspGenX bridges used directly by the BT."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def config(package, filename):
    return PathJoinSubstitution([FindPackageShare(package), 'config', filename])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sam3_params_file', default_value=config(
            'rwh_beyond_sam3_bridge', 'bridge.yaml')),
        DeclareLaunchArgument('graspgenx_params_file', default_value=config(
            'rwh_beyond_graspgenx_bridge', 'bridge.yaml')),
        DeclareLaunchArgument('segmentation_endpoint', default_value=''),
        DeclareLaunchArgument('endpoint', default_value=''),
        OpaqueFunction(function=pipeline_nodes),
    ])


def pipeline_nodes(context):
    def overrides(parameter, argument):
        value = LaunchConfiguration(argument).perform(context)
        return {parameter: value} if value else {}
    return [
        Node(package='rwh_beyond_sam3_bridge', executable='segmentation_action_server',
             name='sam3_bridge', namespace='',
             output='screen', parameters=[LaunchConfiguration('sam3_params_file'),
                 {**overrides('endpoint', 'segmentation_endpoint'),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='rwh_beyond_graspgenx_bridge', executable='inference_action_server',
             name='graspgenx_bridge', namespace='',
             output='screen', parameters=[LaunchConfiguration('graspgenx_params_file'),
                 {**overrides('endpoint', 'endpoint'),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ]
