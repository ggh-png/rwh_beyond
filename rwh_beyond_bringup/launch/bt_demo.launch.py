"""Launch the grasp pipeline, BT navigator and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def grasp_bridge_nodes(context, simulation):
    params_file = LaunchConfiguration('params_file')
    use_sim_time = ParameterValue(simulation, value_type=bool)
    segmentation_endpoint = LaunchConfiguration('segmentation_endpoint').perform(context)
    grasp_endpoint = LaunchConfiguration('endpoint').perform(context)
    return [
        Node(
            package='rwh_beyond_sam3_bridge', executable='segmentation_action_server',
            name='sam3_bridge', namespace='', output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time},
                        *([{'endpoint': segmentation_endpoint}] if segmentation_endpoint else [])],
        ),
        Node(
            package='rwh_beyond_graspgenx_bridge', executable='inference_action_server',
            name='graspgenx_bridge', namespace='', output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time},
                        *([{'endpoint': grasp_endpoint}] if grasp_endpoint else [])],
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rwh_beyond_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # LaunchConfiguration
    mode = LaunchConfiguration('mode')
    params_file = LaunchConfiguration('params_file')
    camera_type = LaunchConfiguration('camera_type')
    start_sensors = LaunchConfiguration('start_sensors')
    use_rviz = LaunchConfiguration('rviz')
    simulation = PythonExpression(["'", mode, "' == 'sim'"])
    use_sim_time = ParameterValue(simulation, value_type=bool)

    # Launch arguments
    declare_args = [
        DeclareLaunchArgument('d405_mount_parent', default_value=''),
        DeclareLaunchArgument('d405_mount_xyz', default_value='', description='D405 mount XYZ [m]'),
        DeclareLaunchArgument('d405_mount_rpy', default_value='', description='D405 mount RPY [rad]'),
        DeclareLaunchArgument('d435_mount_parent', default_value=''),
        DeclareLaunchArgument('d435_mount_xyz', default_value='', description='D435 mount XYZ [m]'),
        DeclareLaunchArgument('d435_mount_rpy', default_value='', description='D435 mount RPY [rad]'),
        DeclareLaunchArgument('camera_type', default_value='d405', choices=['d405', 'd435']),
        DeclareLaunchArgument('enable_d435', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('d435_serial', default_value=''),
        DeclareLaunchArgument('d435_depth_profile', default_value='480x270x6'),
        DeclareLaunchArgument('d435_color_profile', default_value='424x240x6'),
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_d405', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sensor_params_file', default_value=os.path.join(
            bringup_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('d405_serial', default_value=''),
        DeclareLaunchArgument('d405_profile', default_value='640x480x15'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'bt_demo.yaml'),
            description='Shared BT, camera and bridge parameter YAML',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the BT demo'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_dir, 'rviz', 'depth_planning.rviz'),
            description='RViz configuration for the BT demo'),
        DeclareLaunchArgument(
            'tcp_offset', default_value='[0.0,0.0,0.1425,0.0,0.0,0.0]',
            description='Flange-to-TCP XYZ/RPY; match the running simulator'),
        DeclareLaunchArgument(
            'segmentation_endpoint',
            default_value=EnvironmentVariable(
                'RWH_SEGMENTATION_ENDPOINT', default_value=''),
            description='SAM segmentation server endpoint'),
        DeclareLaunchArgument(
            'target_text',
            default_value=EnvironmentVariable('RWH_GRASP_TARGET_TEXT', default_value=''),
            description='Target object description'),
        DeclareLaunchArgument(
            'endpoint',
            default_value=EnvironmentVariable(
                'RWH_GRASP_ENDPOINT', default_value=''),
            description='Grasp inference server endpoint'),
    ]

    # Sensors
    sensor_cmd_group = GroupAction(condition=IfCondition(start_sensors), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensor.launch.py')),
            launch_arguments={
                'mode': mode,
                'namespace': '',
                'params_file': LaunchConfiguration('sensor_params_file'),
                'camera_params_file': params_file,
                'use_sim_time': simulation,
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'enable_d405': PythonExpression([
                    "'false' if '", camera_type, "' == 'd435' else '",
                    LaunchConfiguration('enable_d405'), "'",
                ]),
                'd405_mount_parent': LaunchConfiguration('d405_mount_parent'),
                'd405_mount_xyz': LaunchConfiguration('d405_mount_xyz'),
                'd405_mount_rpy': LaunchConfiguration('d405_mount_rpy'),
                'd405_serial': LaunchConfiguration('d405_serial'),
                'd405_profile': LaunchConfiguration('d405_profile'),
            }.items(),
        ),
    ])
    d435_cmd_group = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'd435.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", mode, "' == 'real' and '", start_sensors, "' == 'true' and '",
                camera_type, "' == 'd435' and '", LaunchConfiguration('enable_d435'), "' == 'true'",
            ])),
            launch_arguments={
                'serial_no': LaunchConfiguration('d435_serial'),
                'depth_profile': LaunchConfiguration('d435_depth_profile'),
                'color_profile': LaunchConfiguration('d435_color_profile'),
                'camera_params_file': params_file,
                'pipeline_topics': 'true',
                'rviz': 'false',
            }.items(),
        ),
    ])

    # Grasp bridges
    grasp_bridges_cmd = OpaqueFunction(function=grasp_bridge_nodes, args=[simulation])

    # BT navigation and lifecycle
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'bt_motion.simulation_only': ParameterValue(simulation, value_type=bool),
                'bt_perception.target_text_override': ParameterValue(
                    LaunchConfiguration('target_text'), value_type=str),
            },
        ],
    )
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='bt_lifecycle_manager',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # RViz
    rviz_cmd_group = GroupAction(condition=IfCondition(use_rviz), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            launch_arguments={
                'use_moveit': 'true',
                'rviz_config': LaunchConfiguration('rviz_config'),
                'use_sim_time': PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"]),
            }.items(),
        ),
    ])

    return LaunchDescription([
        *declare_args,
        sensor_cmd_group,
        d435_cmd_group,
        grasp_bridges_cmd,
        rviz_cmd_group,
        bt_navigator_cmd,
        lifecycle_manager_cmd,
        LogInfo(msg=['BT mode: ', mode,
                     '. Shared tree: home, observe, pick, lift, observation return, home.']),
    ])
