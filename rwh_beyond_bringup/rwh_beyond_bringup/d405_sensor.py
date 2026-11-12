"""D405 driver composition; the robot model owns its mount, the SDK owns internals."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_mount():
    path = (Path(get_package_share_directory('moma_mujoco_ros')) /
            'project/config/cameras/d405.yaml')
    return yaml.safe_load(path.read_text())


def d405_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('mode') != 'real' or value('enable_d405').lower() != 'true':
        return []
    profile = value('d405_profile')
    # D405 produces RGB from its depth module, rather than a separate RGB sensor.
    params = {'use_sim_time': False, 'camera_name': 'd405',
              'device_type': 'd405', 'serial_no': value('d405_serial'),
              'enable_depth': True, 'enable_color': True, 'enable_sync': True,
              'enable_infra1': False, 'enable_infra2': False,
              'depth_module.depth_profile': profile,
              'depth_module.color_profile': profile,
              'align_depth.enable': True, 'pointcloud.enable': True,
              'publish_tf': True, 'base_frame_id': 'link',
              'depth_qos': 'SENSOR_DATA', 'color_qos': 'SENSOR_DATA'}
    topics = {
        'color/image_raw': 'color/image_raw', 'color/camera_info': 'color/camera_info',
        'depth/image_rect_raw': 'depth/image_raw',
        'depth/camera_info': 'depth/raw_camera_info',
        'aligned_depth_to_color/image_raw': 'depth/image_rect_raw',
        'aligned_depth_to_color/camera_info': 'depth/camera_info',
        'depth/color/points': 'depth/color/points',
    }
    nodes = [Node(package='realsense2_camera', executable='realsense2_camera_node',
                  name='d405', output='screen', parameters=[params],
                  respawn=True, respawn_delay=3.0,
                  remappings=[('~/' + source, '/camera/' + target)
                              for source, target in topics.items()])]
    if value('publish_d405_mount_tf').lower() == 'true':
        mount = camera_mount()
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='d405_mount_tf', output='screen', arguments=[
                '--frame-id', mount['parent_frame'], '--child-frame-id', 'd405_link',
                *[item for flag, number in zip(
                    ('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                    mount['translation_m'] + mount['body_rpy_rad'])
                  for item in ('--' + flag, str(number))]],
            parameters=[{'use_sim_time': False}]))
    nodes.append(LogInfo(msg='D405: aligned RGB-D and point cloud on /camera; mount is nominal, not hand-eye calibrated'))
    return nodes
