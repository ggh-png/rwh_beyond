"""Sensor mode selection and TF ownership, without opening a device or CAN socket."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from rwh_beyond_bringup.d405_sensor import d405_nodes
from rwh_beyond_bringup.piper_hardware import DEFAULT_TCP, moveit_parameters


def sensor_context(**settings):
    path = Path(__file__).resolve().parents[1] / 'launch/sensor.launch.py'
    spec = importlib.util.spec_from_file_location('selected_sensors', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument): action.execute(context)
    context.launch_configurations.update(settings)
    for action in module.prepare_sensor_configuration(context): action.execute(context)
    return context


def test_simulation_never_opens_physical_sensors():
    context = sensor_context(mode='sim', enable_lidar='true', enable_d405='true')
    assert context.launch_configurations['_sensor_start_lidar'] == 'false'
    assert context.launch_configurations['_sensor_use_tf_publisher'] == 'false'
    assert d405_nodes(context) == []


def test_d405_only_uses_aligned_rgbd_and_real_time_without_lidar():
    context = sensor_context(mode='real', enable_lidar='false', enable_d405='true')
    assert context.launch_configurations['_sensor_start_lidar'] == 'false'
    nodes = [node for node in d405_nodes(context) if isinstance(node, Node)]
    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('realsense2_camera', 'realsense2_camera_node')]
    params = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
    assert params['use_sim_time'] is False
    assert params['align_depth.enable'] is True
    assert params['depth_module.depth_profile'] == params['depth_module.color_profile']
    assert params['enable_infra1'] is params['enable_infra2'] is False
    assert params['publish_tf'] is True


def test_all_sensors_can_be_disabled():
    context = sensor_context(mode='real', enable_lidar='false', enable_d405='false')
    assert d405_nodes(context) == []


def test_camera_mount_is_shared_but_sdk_internal_tf_is_not_duplicated():
    sim = ET.fromstring(moveit_parameters(DEFAULT_TCP, camera_model=True)['robot_description'])
    real = ET.fromstring(moveit_parameters(DEFAULT_TCP, camera_model=True,
                                         nominal_camera_tf=False)['robot_description'])
    for robot in (sim, real):
        joint = robot.find("joint[@name='d405_mount_joint']")
        assert [float(x) for x in joint.find('origin').get('xyz').split()] == [-.06, 0., .065]
        mesh = robot.find("link[@name='d405_link']/visual/geometry/mesh")
        assert mesh.get('filename').endswith('/d405.stl')
        assert mesh.get('scale') == '0.001 0.001 0.001'
    assert sim.find("link[@name='d405_color_optical_frame']") is not None
    assert real.find("link[@name='d405_color_optical_frame']") is None
