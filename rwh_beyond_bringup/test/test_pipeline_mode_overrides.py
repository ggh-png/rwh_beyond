"""Use real rcl parameter parsing to verify launch overrides with shared YAML."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node as LaunchNode
from rclpy.context import Context
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[2]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('mode', ['sim', 'real'])
def test_mode_overrides_survive_shared_yaml_wildcard(mode):
    pipeline = load(ROOT/'rwh_beyond_grasp_pipeline/launch/grasp_pipeline.launch.py', 'mode_pipeline')
    demo = load(ROOT/'rwh_beyond_bringup/launch/bt_demo.launch.py', 'mode_demo')
    shared = str(ROOT/'rwh_beyond_bringup/config/bt_demo.yaml')
    context = LaunchContext()
    for action in pipeline.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(mode=mode, use_sim_time=str(mode == 'sim'),
        params_file=shared, sam3_params_file=shared, graspgenx_params_file=shared)
    nodes = pipeline.pipeline_nodes(context)
    for action in demo.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.name not in context.launch_configurations:
            action.execute(context)
        if isinstance(action, LaunchNode):
            nodes.append(action)
    for launched in nodes:
        launched._perform_substitutions(context)
        args = ['--ros-args']
        for path, is_file in launched._Node__expanded_parameter_arguments:
            assert is_file
            args.extend(['--params-file', path])
        ros = Context()
        ros.init(args=args)
        node = Node(launched.node_name.lstrip('/'), context=ros)
        try:
            assert node.get_parameter('use_sim_time').value is (mode == 'sim')
            assert node.declare_parameter('octomap_enabled', True).value is False
            if launched.node_name == '/grasp_coordinator':
                assert node.declare_parameter('simulation_only', True).value is (mode == 'sim')
        finally:
            node.destroy_node()
            ros.shutdown()
