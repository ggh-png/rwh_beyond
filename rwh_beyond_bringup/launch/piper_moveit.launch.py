"""Launch the VCS-managed official AgileX PiPER MoveIt stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    official_launch = os.path.join(
        get_package_share_directory('agx_arm_ctrl'),
        'launch',
        'start_single_agx_arm_moveit.launch.py',
    )
    arguments = (
        ('namespace', ''),
        ('can_port', 'can0'),
        ('effector_type', 'none'),
        ('auto_enable', 'true'),
        ('speed_percent', '0'),
        ('follow', 'true'),
        ('auto_control_gate', 'true'),
        ('log_level', 'info'),
    )

    return LaunchDescription([
        *[
            DeclareLaunchArgument(
                name,
                default_value=default,
                description=f'Forwarded to the official AgileX PiPER MoveIt launch: {name}.',
            )
            for name, default in arguments
        ],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(official_launch),
            launch_arguments={
                'arm_type': 'piper',
                **{name: LaunchConfiguration(name) for name, _ in arguments},
            }.items(),
        ),
    ])
