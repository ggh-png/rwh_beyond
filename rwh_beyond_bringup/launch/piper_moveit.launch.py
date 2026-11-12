"""Real PiPER CAN driver with the simulation MoveIt planning baseline."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from rwh_beyond_bringup.piper_hardware import DEFAULT_TCP, hardware_nodes


def generate_launch_description():
    defaults = (
        ('namespace', ''), ('can_port', 'can0'), ('effector_type', 'agx_gripper'),
        ('auto_enable', 'false'), ('speed_percent', '10'), ('auto_control_gate', 'true'),
        ('log_level', 'info'), ('fw_version', ''), ('tcp_offset', DEFAULT_TCP),
        ('use_rviz', 'true'), ('camera_model', 'true'), ('octomap', 'true'),
        ('start_sensors', 'true'), ('enable_d405', 'true'), ('enable_lidar', 'false'),
        ('d405_serial', ''), ('d405_profile', '640x480x15'),
        ('sensor_params_file', str(Path(get_package_share_directory('rwh_beyond_bringup')) /
                                 'params/nav2_params.yaml')),
        ('execution_transport', 'official'),
        ('feedback_topic', 'feedback/joint_states'),
        ('point_cloud_topic', '/camera/depth/planning_points'),
        ('rviz_config', str(Path(get_package_share_directory('moma_bringup')) /
                            'config/depth_planning.rviz')),
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults],
        OpaqueFunction(function=hardware_nodes),
    ])
