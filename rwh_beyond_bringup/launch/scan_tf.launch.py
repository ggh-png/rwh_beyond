"""Publish only the configured scan transform; no Nav2, sensor driver, or motion."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rwh_beyond_bringup.scan_tf import make_scan_tf_node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    with open(value('sensor_tf_file'), encoding='utf-8') as stream:
        robot = yaml.safe_load(stream)
    # Invoking this standalone launch explicitly requests the transform.
    robot.setdefault('scan_static_tf', {})['enabled'] = True
    if value('scan_parent_frame'):
        robot['scan_static_tf']['parent_frame'] = value('scan_parent_frame')
    return make_scan_tf_node(robot, value('namespace').strip('/'), value('use_sim_time') == 'true')


def generate_launch_description():
    share = get_package_share_directory('rwh_beyond_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0724'),
        DeclareLaunchArgument('sensor_tf_file', default_value=os.path.join(share, 'params', 'sensor_tf.yaml')),
        DeclareLaunchArgument('scan_parent_frame', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
