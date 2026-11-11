"""Launch the grasp pipeline, BT navigator and integrated MoveIt/grasp RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
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
                'params',
                'bt_demo_params.yaml',
            ]),
            description='BT navigator and lifecycle manager parameter YAML',
        ),
        DeclareLaunchArgument(
            'grasp_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rwh_beyond_grasp_bridge'), 'config', 'grasp_pipeline.yaml']),
            description='Grasp coordinator and inference bridge parameter YAML'),
        DeclareLaunchArgument(
            'segmentation_endpoint',
            default_value=EnvironmentVariable(
                'RWH_SEGMENTATION_ENDPOINT', default_value='tcp://127.0.0.1:5567'),
            description='SAM segmentation server endpoint'),
        DeclareLaunchArgument(
            'segmentation_backend',
            default_value=EnvironmentVariable('RWH_SEGMENTATION_BACKEND', default_value='sam3'),
            description='Segmentation backend'),
        DeclareLaunchArgument(
            'target_text',
            default_value=EnvironmentVariable('RWH_GRASP_TARGET_TEXT', default_value='red box'),
            description='Target object description'),
        DeclareLaunchArgument(
            'endpoint',
            default_value=EnvironmentVariable(
                'RWH_GRASP_ENDPOINT', default_value='tcp://127.0.0.1:5566'),
            description='Grasp inference server endpoint'),
        # Both launch files expose params_file. Scope the child configuration so
        # its grasp YAML neither inherits nor overwrites the navigator YAML.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rwh_beyond_grasp_bridge'), 'launch', 'grasp_pipeline.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('grasp_params_file'),
                'segmentation_endpoint': LaunchConfiguration('segmentation_endpoint'),
                'segmentation_backend': LaunchConfiguration('segmentation_backend'),
                'target_text': LaunchConfiguration('target_text'),
                'endpoint': LaunchConfiguration('endpoint'),
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
            }.items()),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='bt_lifecycle_manager',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
