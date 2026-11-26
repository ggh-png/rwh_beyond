"""Launch PiPER MoveIt with the physical or MuJoCo backend."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

from rwh_beyond_bringup.piper_hardware import DEFAULT_TCP, hardware_nodes, simulation_nodes


def piper_nodes(context):
    if LaunchConfiguration('mode').perform(context) == 'sim':
        return simulation_nodes(context)
    return hardware_nodes(context)


def generate_launch_description():
    speed_default = EnvironmentVariable(
        'RWH_PIPER_SPEED_PERCENT',
        default_value=PythonExpression(["100 if '", LaunchConfiguration('mode'), "' == 'sim' else 10"]),
    )
    defaults = (
        ('namespace', ''), ('can_port', 'can0'), ('effector_type', 'agx_gripper'),
        ('auto_enable', 'false'), ('speed_percent', speed_default), ('auto_control_gate', 'true'),
        ('fast_mode', 'false'), ('pub_rate', '200'), ('enable_timeout', '5.0'),
        ('gripper_default_effort', '1.0'),
        ('log_level', 'info'), ('fw_version', ''), ('tcp_offset', DEFAULT_TCP),
        ('use_rviz', 'true'), ('camera_model', 'true'), ('octomap', 'true'),
        ('start_sensors', 'true'), ('enable_d405', 'true'), ('enable_lidar', 'false'),
        ('d405_mount_parent', ''), ('d405_mount_xyz', ''), ('d405_mount_rpy', ''),
        ('d435_mount_parent', ''), ('d435_mount_xyz', ''), ('d435_mount_rpy', ''),
        ('camera_type', 'd405'), ('enable_d435', 'false'),
        ('d435_serial', ''), ('d435_depth_profile', '480x270x6'),
        ('d435_color_profile', '424x240x6'),
        ('d405_serial', ''), ('d405_profile', '640x480x15'),
        ('sensor_params_file', str(Path(get_package_share_directory('rwh_beyond_bringup')) /
                                 'params/nav2_params.yaml')),
        ('params_file', str(Path(get_package_share_directory('rwh_beyond_bringup')) /
                            'config/bt_demo.yaml')),
        ('execution_transport', 'official'),
        ('feedback_topic', 'feedback/joint_states'),
        ('point_cloud_topic', '/camera/depth/planning_points'),
        ('rviz_config', str(Path(get_package_share_directory('moma_bringup')) /
                            'config/depth_planning.rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='real', choices=['real', 'sim']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera_rate', default_value='5.0'),
        DeclareLaunchArgument('gripper_contact_force_threshold', default_value='1.0'),
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults],
        OpaqueFunction(function=piper_nodes),
    ])
