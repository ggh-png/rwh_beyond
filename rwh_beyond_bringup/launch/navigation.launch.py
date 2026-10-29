"""Use repository-owned full parameters with the official nav2_bringup launch."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rwh_beyond_bringup.parameters import render_parameters
from rwh_beyond_bringup.scan_tf import make_scan_tf_node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    namespace = value('namespace').strip('/')
    map_path = os.path.abspath(os.path.expanduser(value('map')))
    if not os.path.isfile(map_path):
        raise ValueError('Map YAML does not exist: ' + map_path)
    with open(value('params_file'), encoding='utf-8') as stream:
        source = yaml.safe_load(stream)
    parameters = render_parameters(source, namespace, value('use_sim_time') == 'true', value('cmd_vel_type'))
    with open(value('sensor_tf_file'), encoding='utf-8') as stream:
        sensor = yaml.safe_load(stream)
    if value('publish_scan_tf'):
        sensor.setdefault('scan_static_tf', {})['enabled'] = value('publish_scan_tf') == 'true'
    transforms = make_scan_tf_node(sensor, namespace, value('use_sim_time') == 'true')
    with tempfile.NamedTemporaryFile(mode='w', prefix='rwh-nav2-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(parameters, stream, sort_keys=False)
        path = stream.name

    def cleanup(_context):
        if os.path.exists(path):
            os.unlink(path)
        return []

    share = get_package_share_directory('rwh_beyond_bringup')
    return [*transforms,
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace, 'use_namespace': 'True' if namespace else 'False',
                'map': map_path, 'params_file': path, 'slam': 'False', 'use_localization': 'True',
                'use_sim_time': value('use_sim_time'), 'autostart': value('autostart'),
                'use_composition': 'False', 'use_respawn': 'False',
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'server.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_server')),
            launch_arguments={
                'namespace': namespace, 'autostart': value('autostart'),
                'use_sim_time': value('use_sim_time'),
                'server_params_file': value('server_params_file'),
            }.items())]


def generate_launch_description():
    share = get_package_share_directory('rwh_beyond_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Existing map YAML on the external PC'),
        DeclareLaunchArgument('namespace', default_value='a200_0724'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('sensor_tf_file', default_value=os.path.join(share, 'params', 'sensor_tf.yaml')),
        DeclareLaunchArgument('server_params_file', default_value=os.path.join(share, 'params', 'rwh_beyond.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_server', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('cmd_vel_type', default_value='', choices=['', 'twist', 'twist_stamped'],
                             description='Empty: use the checked-in full configuration unchanged'),
        DeclareLaunchArgument('publish_scan_tf', default_value='', choices=['', 'true', 'false']),
        OpaqueFunction(function=setup),
    ])
