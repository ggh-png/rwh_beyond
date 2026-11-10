"""Verify that the BT demo launch does not start a simulation."""

import importlib.util
from pathlib import Path
import unittest

from launch_ros.actions import Node


class BtDemoLaunchTest(unittest.TestCase):
    def test_only_navigator_and_lifecycle_manager_are_launched(self):
        launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_demo_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        entities = module.generate_launch_description().entities

        self.assertEqual(len(entities), 2)
        self.assertTrue(all(isinstance(entity, Node) for entity in entities))
        self.assertEqual(
            [(entity.node_package, entity.node_executable) for entity in entities],
            [('nav2_bt_navigator', 'bt_navigator'),
             ('nav2_lifecycle_manager', 'lifecycle_manager')],
        )


if __name__ == '__main__':
    unittest.main()
