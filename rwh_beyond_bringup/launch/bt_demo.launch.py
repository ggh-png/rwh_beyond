"""Launch the grasp pipeline, BT navigator and integrated MoveIt/grasp RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'sim'"])
    return LaunchDescription([
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
        DeclareLaunchArgument('sensor_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rwh_beyond_bringup'), 'params', 'nav2_params.yaml'])),
        DeclareLaunchArgument('d405_serial', default_value=''),
        DeclareLaunchArgument('d405_profile', default_value='640x480x15'),
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration('start_sensors')),
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rwh_beyond_bringup'), 'launch', 'sensor.launch.py'])),
                launch_arguments={
                    'mode': LaunchConfiguration('mode'), 'namespace': '',
                    'params_file': LaunchConfiguration('sensor_params_file'),
                    'use_sim_time': simulation,
                    'enable_lidar': LaunchConfiguration('enable_lidar'),
                    'enable_d405': PythonExpression(["'false' if '", LaunchConfiguration('camera_type'), "' == 'd435' else '", LaunchConfiguration('enable_d405'), "'"]),
                    'd405_mount_parent': LaunchConfiguration('d405_mount_parent'),
                    'd405_mount_xyz': LaunchConfiguration('d405_mount_xyz'),
                    'd405_mount_rpy': LaunchConfiguration('d405_mount_rpy'),
                    'd405_serial': LaunchConfiguration('d405_serial'),
                    'd405_profile': LaunchConfiguration('d405_profile'),
                }.items())]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rwh_beyond_bringup'), 'launch', 'd435.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'real' and '",
                LaunchConfiguration('start_sensors'), "' == 'true' and '",
                LaunchConfiguration('camera_type'), "' == 'd435' and '",
                LaunchConfiguration('enable_d435'), "' == 'true'"])),
            launch_arguments={
                'serial_no': LaunchConfiguration('d435_serial'),
                'depth_profile': LaunchConfiguration('d435_depth_profile'),
                'color_profile': LaunchConfiguration('d435_color_profile'),
                'pipeline_topics': 'true', 'rviz': 'false',
            }.items())]),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start the integrated MoveIt, SAM and grasp debug RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('moma_bringup'), 'config', 'depth_planning.rviz']),
            description='MoveIt and grasp debug RViz configuration'),
        DeclareLaunchArgument(
            'tcp_offset', default_value='[0.0,0.0,0.1425,0.0,0.0,0.0]',
            description='Flange-to-TCP XYZ/RPY; match the running simulator'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rwh_beyond_bringup'),
                'config',
                'bt_demo.yaml',
            ]),
            description='Shared BT, bridge and coordinator parameter YAML',
        ),
        DeclareLaunchArgument(
            'segmentation_endpoint',
            default_value=EnvironmentVariable(
                'RWH_SEGMENTATION_ENDPOINT', default_value=''),
            description='SAM segmentation server endpoint'),
        DeclareLaunchArgument(
            'segmentation_backend',
            default_value=EnvironmentVariable('RWH_SEGMENTATION_BACKEND', default_value=''),
            description='Segmentation backend'),
        DeclareLaunchArgument(
            'target_text',
            default_value=EnvironmentVariable('RWH_GRASP_TARGET_TEXT', default_value=''),
            description='Target object description'),
        DeclareLaunchArgument(
            'endpoint',
            default_value=EnvironmentVariable(
                'RWH_GRASP_ENDPOINT', default_value=''),
            description='Grasp inference server endpoint'),
        # The same multi-node YAML supplies every BT pipeline node.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rwh_beyond_grasp_pipeline'), 'launch', 'grasp_pipeline.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'sam3_params_file': LaunchConfiguration('params_file'),
                'graspgenx_params_file': LaunchConfiguration('params_file'),
                'segmentation_endpoint': LaunchConfiguration('segmentation_endpoint'),
                'segmentation_backend': LaunchConfiguration('segmentation_backend'),
                'target_text': LaunchConfiguration('target_text'),
                'endpoint': LaunchConfiguration('endpoint'),
                'use_sim_time': simulation,
                'start_coordinator': 'true',
                'mode': LaunchConfiguration('mode'),
            }.items())]),
        # RViz belongs to this BT launch, not the simulator dependency.
        # The included launch starts only RViz using the same MoveIt robot parameters.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rwh_beyond_bringup'), 'launch', 'moveit_rviz.launch.py'])),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'd405_mount_parent': LaunchConfiguration('d405_mount_parent'),
                'd405_mount_xyz': LaunchConfiguration('d405_mount_xyz'),
                'd405_mount_rpy': LaunchConfiguration('d405_mount_rpy'),
                'd435_mount_parent': LaunchConfiguration('d435_mount_parent'),
                'd435_mount_xyz': LaunchConfiguration('d435_mount_xyz'),
                'd435_mount_rpy': LaunchConfiguration('d435_mount_rpy'),
                'camera_type': LaunchConfiguration('camera_type'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'tcp_offset': LaunchConfiguration('tcp_offset'),
                'use_sim_time': PythonExpression([
                    "'true' if '", LaunchConfiguration('mode'), "' == 'sim' else 'false'"]),
            }.items()),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator', namespace='',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        # Tree paths come from params_file. The navigator supplies
                        # package defaults when those parameters are absent.
                        {'use_sim_time': ParameterValue(simulation, value_type=bool)}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='bt_lifecycle_manager', namespace='',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': ParameterValue(simulation, value_type=bool)}],
        ),
        LogInfo(msg=['BT mode: ', LaunchConfiguration('mode'),
                     '. Shared tree: home, observe, pick, lift, observation return, home.']),
    ])
