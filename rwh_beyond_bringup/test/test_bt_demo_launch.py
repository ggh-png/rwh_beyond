"""Verify that the BT demo launch does not start a simulation."""

import importlib.util
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml


class BtDemoLaunchTest(unittest.TestCase):
    def test_only_navigator_and_lifecycle_manager_are_launched(self):
        launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_demo_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        entities = module.generate_launch_description().entities

        arguments = [entity for entity in entities if isinstance(entity, DeclareLaunchArgument)]
        nodes = [entity for entity in entities if isinstance(entity, Node)]
        self.assertEqual(len(entities), 3)
        self.assertEqual([argument.name for argument in arguments], ['params_file'])
        self.assertEqual(
            [(node.node_package, node.node_executable) for node in nodes],
            [('nav2_bt_navigator', 'bt_navigator'),
             ('nav2_lifecycle_manager', 'lifecycle_manager')],
        )

    def test_default_parameters_preserve_previous_values(self):
        params_path = Path(__file__).resolve().parents[1] / 'params' / 'bt_demo_params.yaml'
        params = yaml.safe_load(params_path.read_text())
        self.assertEqual(params, {
            'bt_navigator': {'ros__parameters': {
                'use_sim_time': True,
                'global_frame': 'base_link',
                'robot_base_frame': 'base_link',
                'odom_topic': '/odom',
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                'wait_for_service_timeout': 30000,
                'action_server_result_timeout': 900.0,
                'plugin_lib_names': ['rwh_beyond_bt_nodes'],
                'navigators': ['demo'],
                'demo': {'plugin': 'rwh_beyond_navigator::DemoNavigator'},
                'run_demo': {
                    'enable_groot_monitoring': True,
                    'groot_server_port': 5555,
                },
            }},
            'bt_lifecycle_manager': {'ros__parameters': {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['bt_navigator'],
            }},
        })


if __name__ == '__main__':
    unittest.main()
