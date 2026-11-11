"""Verify the standalone RViz uses the same robot configuration as the simulator."""

import importlib.util
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters


def load_launch(path):
    spec = importlib.util.spec_from_file_location('test_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MoveitRvizLaunchTest(unittest.TestCase):
    def test_rviz_matches_simulator_robot_parameters(self):
        rviz = load_launch(Path(__file__).resolve().parents[1] / 'launch/moveit_rviz.launch.py')
        simulator = load_launch(
            Path(get_package_share_directory('moma_bringup')) / 'launch/simulation.launch.py')
        context = LaunchContext()
        for entity in simulator.generate_launch_description().entities:
            if isinstance(entity, DeclareLaunchArgument):
                context.launch_configurations[entity.name] = ''.join(
                    part.perform(context) for part in entity.default_value)
        context.launch_configurations.update(
            rviz_only='true', tcp_offset='[0.01,0.02,0.15,0.0,0.0,0.0]')
        sim_node, = simulator.nodes(context)
        rviz_node, = rviz.create_rviz(context)
        self.assertEqual((rviz_node.node_package, rviz_node.node_executable), ('rviz2', 'rviz2'))
        self.assertEqual(
            evaluate_parameters(context, rviz_node._Node__parameters),
            evaluate_parameters(context, sim_node._Node__parameters))

    def test_invalid_tcp_offset_is_rejected(self):
        rviz = load_launch(Path(__file__).resolve().parents[1] / 'launch/moveit_rviz.launch.py')
        context = LaunchContext()
        for offset in ('[0, 0, 0]', '[0, 0, 0, 0, 0, 1e999]'):
            with self.subTest(offset=offset):
                context.launch_configurations['tcp_offset'] = offset
                with self.assertRaises(ValueError):
                    rviz.create_rviz(context)


if __name__ == '__main__':
    unittest.main()
