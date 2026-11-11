"""Hardware planning parity without starting a driver or sending robot commands."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from launch_ros.parameter_descriptions import ParameterValue
from rwh_beyond_bringup.piper_hardware import (
    DEFAULT_TCP, controller_parameters, hardware_nodes, moveit_parameters,
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(module):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def test_hardware_matches_simulation_parameters_except_clock():
    simulator = load(Path(get_package_share_directory('moma_bringup')) /
                     'launch/simulation.launch.py', 'parity_sim')
    context = defaults(simulator)
    context.launch_configurations.update(moveit='true', gui='false', rviz='false')
    node = next(n for n in simulator.nodes(context) if n.node_executable == 'move_group')
    expected = evaluate_parameters(context, node._Node__parameters)[0]
    params = moveit_parameters(DEFAULT_TCP, camera_model=True)
    for key in ('robot_description', 'robot_description_semantic'):
        params[key] = ParameterValue(params[key], value_type=str)
    actual = evaluate_parameters(context, normalize_parameters([params]))[0]
    assert actual.pop('use_sim_time') is False
    assert expected.pop('use_sim_time') is True
    assert actual == expected


def test_camera_mount_is_not_assumed_and_robot_bounds_are_preserved():
    real = moveit_parameters(DEFAULT_TCP)
    sim = moveit_parameters(DEFAULT_TCP, camera_model=True)
    urdf = ET.fromstring(real['robot_description'])
    assert urdf.find("link[@name='d405_link']") is None
    assert urdf.find("joint[@name='tcp_joint']/origin").get('xyz') == '0.0 0.0 0.1425'
    bounds = lambda text: {j.get('name'): j.find('limit').attrib for j in
                           ET.fromstring(text).findall('joint') if j.find('limit') is not None}
    assert bounds(real['robot_description']) == bounds(sim['robot_description'])
    assert real['robot_description_planning'] == sim['robot_description_planning']
    assert real['robot_description_kinematics'] == sim['robot_description_kinematics']


def test_hardware_transport_keeps_closed_gate_and_real_feedback():
    module = load(Path(__file__).resolve().parents[1] /
                  'launch/piper_moveit.launch.py', 'parity_hardware')
    context = defaults(module)
    assert context.launch_configurations['auto_enable'] == 'false'
    driver, group = hardware_nodes(context)
    assert isinstance(driver, IncludeLaunchDescription)
    assert isinstance(group, GroupAction)
    args = dict(driver.launch_arguments)
    assert args['control_enabled'] == 'false'
    assert args['tcp_offset'].perform(context) == DEFAULT_TCP
    nodes = [n for n in group.get_sub_entities() if hasattr(n, 'node_package')]
    assert all(n.node_package != 'moma_mujoco_ros' for n in nodes)
    move_group = next(n for n in nodes if n.node_executable == 'move_group')
    params = evaluate_parameters(context, move_group._Node__parameters)[0]
    assert params['use_sim_time'] is False
    assert params['planning_pipelines'] == ('ompl',)
    assert params['depth_cloud.padding_offset'] == .02


@pytest.mark.parametrize('offset', ['[0,0,0]', '[0,0,1e999,0,0,0]', '123'])
def test_invalid_tcp_rejected_before_hardware_launch(offset):
    with pytest.raises(ValueError):
        moveit_parameters(offset)


def test_hardware_controller_names_and_groups_match_simulator():
    params, names = controller_parameters('arm1', 'agx_gripper')
    assert names == ['arm_controller', 'gripper_controller']
    assert params['/arm1/arm_controller']['ros__parameters']['joints'] == [
        f'joint{i}' for i in range(1, 7)]
    assert params['/arm1/gripper_controller']['ros__parameters']['joints'] == ['gripper']
