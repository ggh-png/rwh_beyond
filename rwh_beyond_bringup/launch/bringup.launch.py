# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def prepare_launch(context):
    """Normalize namespace and boolean values before evaluating child launches."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    params_path = os.path.expanduser(value('params_file'))
    if not os.path.isfile(params_path):
        raise ValueError('Parameter YAML does not exist: ' + params_path)
    if value('slam').lower() == 'false' and value('use_localization').lower() == 'true':
        map_path = os.path.expanduser(value('map_yaml_file'))
        if not os.path.isfile(map_path):
            raise ValueError('Map YAML does not exist: ' + map_path)
        with open(map_path, encoding='utf-8') as stream:
            image = yaml.safe_load(stream).get('image', '')
        if not image or not os.path.isfile(os.path.join(os.path.dirname(map_path), image)):
            raise ValueError('Map image does not exist for: ' + map_path)
    namespace = value('namespace').strip('/') if value('use_namespace').lower() == 'true' else ''
    actions = [SetLaunchConfiguration('params_file', params_path),
               SetLaunchConfiguration('namespace', namespace),
               SetLaunchConfiguration('_robot_namespace_prefix', '/' + namespace if namespace else '')]
    for name in ('slam', 'use_localization', 'use_composition', 'use_respawn'):
        setting = value(name).lower()
        if setting not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        actions.append(SetLaunchConfiguration(name, setting.title()))
    if value('slam').lower() == 'false' and value('use_localization').lower() == 'true':
        actions.append(SetLaunchConfiguration('map_yaml_file', map_path))
    return actions


def generate_launch_description():
    # 패키지의 launch / 단일 파라미터 / 지도 경로
    bringup_dir = get_package_share_directory('rwh_beyond_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    default_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    default_map_yaml_file = os.path.join(bringup_dir, 'maps', 'real_map_0505.yaml')

    # LaunchConfiguration
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')
    container_name = LaunchConfiguration('container_name')
    slam = LaunchConfiguration('slam')
    use_localization = LaunchConfiguration('use_localization')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 공통 파라미터: 모든 하위 launch에 같은 전체 YAML을 전달한다.
    params_file = ReplaceString(
        source_file=LaunchConfiguration('params_file'),
        replacements={'<robot_namespace>': LaunchConfiguration('_robot_namespace_prefix')},
    )
    configured_params = ParameterFile(RewrittenYaml(
        source_file=params_file, root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time}, convert_types=True), allow_substs=True)
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Launch arguments
    declare_args = [
        DeclareLaunchArgument('namespace', default_value='a200_0724', description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock; skip real sensor'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Single shared parameter YAML'),
        DeclareLaunchArgument('map', default_value=default_map_yaml_file, description='Legacy map YAML argument'),
        DeclareLaunchArgument('map_yaml_file', default_value=LaunchConfiguration('map'), description='Map YAML path'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='True', description='Start RViz'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('use_localization', default_value='True'),
        DeclareLaunchArgument('start_sensor', default_value='true', description='Disable if sensor runs separately'),
        DeclareLaunchArgument('start_driver', default_value='true', description='Disable for TF-only operation'),
        DeclareLaunchArgument('sensor_hostname', default_value='', description='Empty: use sensor_config.hostname in YAML'),
        DeclareLaunchArgument('use_tf_publisher', default_value='', description='Empty: use YAML static TF enabled setting'),
    ]

    # Navigation / localization / SLAM
    common_arguments = {
        'namespace': namespace, 'use_sim_time': use_sim_time, 'autostart': autostart,
        'params_file': params_file, 'use_composition': use_composition,
        'use_respawn': use_respawn, 'container_name': container_name, 'log_level': log_level,
    }
    bringup_cmd_group = GroupAction([
        PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        Node(
            condition=IfCondition(use_composition), name=container_name,
            package='rclcpp_components', executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings, output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments=common_arguments.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
            launch_arguments={**common_arguments, 'map': map_yaml_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
            launch_arguments=common_arguments.items(),
        ),
    ])

    # 센서는 자체 namespace를 지정하므로 PushROSNamespace 그룹 밖에서 실행한다.
    sensor_cmd_group = GroupAction(condition=IfCondition(LaunchConfiguration('start_sensor')), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensor.launch.py')),
            condition=UnlessCondition(use_sim_time),
            launch_arguments={
                'namespace': namespace, 'use_sim_time': use_sim_time, 'params_file': params_file,
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'use_tf_publisher': LaunchConfiguration('use_tf_publisher'),
                'start_driver': LaunchConfiguration('start_driver'), 'log_level': log_level,
            }.items(),
        ),
    ])
    rviz_cmd = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace, 'use_namespace': use_namespace,
                          'use_sim_time': use_sim_time}.items(),
    )])

    # LaunchDescription: 선언 → 공통 설정 → 하위 launch → RViz
    ld = LaunchDescription()
    for argument in declare_args:
        ld.add_action(argument)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(OpaqueFunction(function=prepare_launch))
    ld.add_action(sensor_cmd_group)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_cmd)
    return ld
