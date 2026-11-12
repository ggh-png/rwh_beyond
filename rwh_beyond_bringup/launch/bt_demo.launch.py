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
                    'enable_d405': LaunchConfiguration('enable_d405'),
                    'd405_serial': LaunchConfiguration('d405_serial'),
                    'd405_profile': LaunchConfiguration('d405_profile'),
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
                        {'use_sim_time': ParameterValue(simulation, value_type=bool),
                         'default_demo_bt_xml': PathJoinSubstitution([
                             FindPackageShare('rwh_beyond_bt'), 'behavior_trees',
                             'demo.xml']),
                         'default_place_bt_xml': PathJoinSubstitution([
                             FindPackageShare('rwh_beyond_bt'), 'behavior_trees',
                             'place.xml'])}],
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
