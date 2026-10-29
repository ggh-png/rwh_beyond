# Copyright (c) 2020 Samsung Research Russia
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml


def prepare_namespace(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    return [SetLaunchConfiguration('namespace', namespace),
            SetLaunchConfiguration('_slam_topic_prefix', '/' + namespace if namespace else '')]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rwh_beyond_bringup')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    params_file = ReplaceString(source_file=LaunchConfiguration('params_file'),
        replacements={'<robot_namespace>': LaunchConfiguration('_slam_topic_prefix')})
    rewritten_params = RewrittenYaml(source_file=params_file, root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time}, convert_types=True)
    configured_params = ParameterFile(rewritten_params, allow_substs=True)
    declare_args = [
        DeclareLaunchArgument('namespace', default_value='a200_0724'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]
    load_nodes = GroupAction(actions=[
        # Absolute namespace also allows this launch to run without a parent bringup.
        PushROSNamespace(namespace=['/', namespace]),
        SetRemap(src='/scan', dst='scan'), SetRemap(src='/map', dst='map'),
        SetRemap(src='/tf', dst='tf'), SetRemap(src='/tf_static', dst='tf_static'),
        Node(package='nav2_map_server', executable='map_saver_server', name='map_saver',
             output='screen', parameters=[configured_params],
             respawn=LaunchConfiguration('use_respawn'), respawn_delay=2.0,
             arguments=['--ros-args', '--log-level', log_level]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam', output='screen', parameters=[{
                 'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                 'autostart': ParameterValue(autostart, value_type=bool), 'node_names': ['map_saver']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'slam_params_file': rewritten_params, 'use_sim_time': use_sim_time,
                              'use_lifecycle_manager': 'false'}.items()),
    ])
    ld = LaunchDescription(declare_args)
    ld.add_action(OpaqueFunction(function=prepare_namespace))
    ld.add_action(load_nodes)
    return ld
