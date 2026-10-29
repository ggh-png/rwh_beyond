#!/usr/bin/env python3
"""SICK sensor driver and static TF, independent of navigation and localization."""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, OpaqueFunction,
    SetEnvironmentVariable, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rwh_beyond_bringup.scan_tf import transform_arguments


def prepare_sensor_configuration(context):
    """Read YAML after launch arguments resolve; keep node definitions below."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    with open(value('params_file'), encoding='utf-8') as stream:
        sensor = yaml.safe_load(stream)['sensor_config']['ros__parameters']
    frame = sensor['scan_frame']
    if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
        raise ValueError('scan_frame must be a nonempty frame ID without leading slash or whitespace')

    settings = {'_sensor_namespace': value('namespace').strip('/'), '_scan_frame': frame,
                '_sensor_driver_file': '',
                '_sensor_hostname': value('sensor_hostname') or sensor['hostname'],
                '_sensor_min_ang': value('min_ang') or str(sensor['min_ang']),
                '_sensor_max_ang': value('max_ang') or str(sensor['max_ang'])}
    tf_setting = value('use_tf_publisher').lower()
    if not tf_setting:
        enabled = sensor.get('scan_static_tf', {}).get('enabled', False)
        if not isinstance(enabled, bool):
            raise ValueError('scan_static_tf.enabled must be a YAML boolean')
        tf_setting = str(enabled).lower()
    settings['_sensor_use_tf_publisher'] = tf_setting
    if tf_setting == 'true':
        sensor.setdefault('scan_static_tf', {})['enabled'] = True
        arguments = transform_arguments(sensor)
        settings.update({'_sensor_tf_' + flag[2:].replace('-', '_'): arg
                         for flag, arg in zip(arguments[::2], arguments[1::2])})
    if value('start_driver').lower() == 'true':
        minimum, maximum = float(settings['_sensor_min_ang']), float(settings['_sensor_max_ang'])
        if not all(math.isfinite(v) for v in (minimum, maximum)) or minimum >= maximum:
            raise ValueError('min_ang and max_ang must be finite with min_ang < max_ang')
        settings['_sensor_driver_file'] = os.path.join(
            get_package_share_directory('sick_scan_xd'), 'launch', 'sick_tim_5xx.launch')
    return [SetLaunchConfiguration(name, setting) for name, setting in settings.items()]


def generate_launch_description():
    # 기본 설정 파일
    bringup_dir = get_package_share_directory('rwh_beyond_bringup')
    default_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    boolean_choices = ['true', 'false', 'True', 'False']

    # Launch arguments
    declare_args = [
        DeclareLaunchArgument('namespace', default_value='a200_0724', description='Sensor and TF namespace'),
        DeclareLaunchArgument('hostname', default_value='', description='Legacy sensor IP argument'),
        DeclareLaunchArgument('sensor_hostname', default_value=LaunchConfiguration('hostname'),
                              description='SICK sensor hostname or IP address'),
        DeclareLaunchArgument('min_ang', default_value='', description='Minimum scan angle (rad)'),
        DeclareLaunchArgument('max_ang', default_value='', description='Maximum scan angle (rad)'),
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Shared Nav2, sensor/TF and server parameter YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=boolean_choices),
        DeclareLaunchArgument('publish_scan_tf', default_value='', choices=[''] + boolean_choices,
                              description='Legacy static TF switch'),
        DeclareLaunchArgument('use_tf_publisher', default_value=LaunchConfiguration('publish_scan_tf'),
                              choices=[''] + boolean_choices, description='Empty: use YAML static TF enabled setting'),
        DeclareLaunchArgument('start_driver', default_value='true', choices=boolean_choices,
                              description='Disable for TF-only operation without sensor access'),
        DeclareLaunchArgument('log_level', default_value='info', description='Console log level'),
    ]

    # LaunchConfiguration
    namespace = LaunchConfiguration('_sensor_namespace')
    sensor_hostname = LaunchConfiguration('_sensor_hostname')
    min_ang = LaunchConfiguration('_sensor_min_ang')
    max_ang = LaunchConfiguration('_sensor_max_ang')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_tf_publisher = LaunchConfiguration('_sensor_use_tf_publisher')
    start_driver = LaunchConfiguration('start_driver')
    log_level = LaunchConfiguration('log_level')

    # 공통 설정: SICK는 vendor XML을 읽는 일반 실행 파일을 사용한다.
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    time_parameters = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    log_arguments = ['--ros-args', '--log-level', log_level]

    # Static TF: 값은 nav2_params.yaml / sensor_config의 translation / rotation_rpy에서 읽는다.
    static_tf_node = Node(
        condition=IfCondition(use_tf_publisher),
        package='tf2_ros', executable='static_transform_publisher',
        name='rwh_scan_static_tf', namespace=namespace, output='screen',
        arguments=[item for flag in ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'frame-id', 'child-frame-id')
                   for item in ('--' + flag, LaunchConfiguration('_sensor_tf_' + flag.replace('-', '_')))]
                  + log_arguments,
        parameters=[time_parameters],
        remappings=remappings,
    )

    # Stand-alone 드라이버: 일반 노드이므로 lifecycle manager / component container는 사용하지 않는다.
    load_nodes = GroupAction(
        condition=IfCondition(start_driver),
        actions=[
            Node(
                package='sick_scan_xd', executable='sick_generic_caller',
                namespace=namespace, output='screen',
                arguments=[LaunchConfiguration('_sensor_driver_file'),
                           ['hostname:=', sensor_hostname],
                           ['min_ang:=', min_ang], ['max_ang:=', max_ang],
                           ['frame_id:=', LaunchConfiguration('_scan_frame')],
                           ['publish_frame_id:=', LaunchConfiguration('_scan_frame')],
                           'tf_publish_rate:=0'] + log_arguments,
                parameters=[time_parameters],
                remappings=remappings + [('/cloud', 'cloud'), ('/scan', 'scan'),
                                         ('/diagnostics', 'sick_tim_5xx/diagnostics')],
            ),
        ],
    )

    # LaunchDescription: 선언 → 공통 설정/YAML 로드 → TF → 드라이버
    ld = LaunchDescription()
    for argument in declare_args:
        ld.add_action(argument)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(OpaqueFunction(function=prepare_sensor_configuration))
    ld.add_action(static_tf_node)
    ld.add_action(load_nodes)
    return ld
