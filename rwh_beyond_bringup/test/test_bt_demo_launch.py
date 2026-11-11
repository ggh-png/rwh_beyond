"""Verify the BT demo launch includes the grasp pipeline, but not a simulation."""

import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch_ros.actions import Node
import yaml


class BtDemoLaunchTest(unittest.TestCase):
    def test_grasp_pipeline_and_navigator_with_optional_rviz(self):
        launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_demo_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        entities = module.generate_launch_description().entities

        arguments = [entity for entity in entities if isinstance(entity, DeclareLaunchArgument)]
        nodes = [entity for entity in entities if isinstance(entity, Node)]
        argument_map = {argument.name: argument for argument in arguments}
        self.assertEqual(
            set(argument_map),
            {'params_file', 'rviz', 'rviz_config', 'tcp_offset', 'segmentation_endpoint',
             'segmentation_backend', 'target_text', 'endpoint', 'grasp_params_file'})
        group = next(entity for entity in entities if isinstance(entity, GroupAction))
        pipeline = next(entity for entity in group.get_sub_entities()
                        if isinstance(entity, IncludeLaunchDescription))
        rviz = next(entity for entity in entities if isinstance(entity, IncludeLaunchDescription))
        self.assertIsNone(pipeline.condition)
        context = LaunchContext()
        self.assertIn("FindPackageShare(pkg='rwh_beyond_bringup')",
                      rviz.launch_description_source.location)
        self.assertIn('moveit_rviz.launch.py', rviz.launch_description_source.location)
        names = ('segmentation_endpoint', 'segmentation_backend', 'target_text', 'endpoint')
        context.launch_configurations.update({name: f'test_{name}' for name in names})
        context.launch_configurations['grasp_params_file'] = '/tmp/grasp.yaml'
        self.assertEqual(
            {name: value.perform(context) for name, value in pipeline.launch_arguments},
            {**{name: f'test_{name}' for name in names}, 'params_file': '/tmp/grasp.yaml'})
        env_vars = {
            'segmentation_endpoint': ('RWH_SEGMENTATION_ENDPOINT', 'tcp://127.0.0.1:5567'),
            'segmentation_backend': ('RWH_SEGMENTATION_BACKEND', 'sam3'),
            'target_text': ('RWH_GRASP_TARGET_TEXT', 'red box'),
            'endpoint': ('RWH_GRASP_ENDPOINT', 'tcp://127.0.0.1:5566'),
        }
        with patch.dict(os.environ, {}, clear=True):
            context = LaunchContext()
            for name, (env_name, default) in env_vars.items():
                self.assertEqual(argument_map[name].default_value[0].perform(context), default)
                os.environ[env_name] = f'env_{name}'
                self.assertEqual(
                    argument_map[name].default_value[0].perform(context), f'env_{name}')
        context.launch_configurations['rviz'] = 'true'
        self.assertTrue(rviz.condition.evaluate(context))
        context.launch_configurations['rviz'] = 'false'
        self.assertFalse(rviz.condition.evaluate(context))
        self.assertEqual(''.join(part.perform(context) for part in argument_map['rviz'].default_value),
                         'true')
        self.assertEqual(
            [(node.node_package, node.node_executable) for node in nodes],
            [('nav2_bt_navigator', 'bt_navigator'),
             ('nav2_lifecycle_manager', 'lifecycle_manager')],
        )

    def test_grasp_yaml_is_scoped_and_does_not_replace_navigator_yaml(self):
        launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_demo_scoping', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        group = next(entity for entity in module.generate_launch_description().entities
                     if isinstance(entity, GroupAction))
        context = LaunchContext()
        context.launch_configurations.update({
            'params_file': '/tmp/navigator.yaml',
            'grasp_params_file': '/tmp/grasp.yaml',
            'segmentation_endpoint': 'tcp://127.0.0.1:5567',
            'segmentation_backend': 'sam3', 'target_text': 'red box',
            'endpoint': 'tcp://127.0.0.1:5566',
        })
        for action in group.execute(context):
            if isinstance(action, IncludeLaunchDescription):
                # IncludeLaunchDescription applies these SetLaunchConfiguration
                # actions before evaluating the child launch's Node parameters.
                for name, value in action.launch_arguments:
                    SetLaunchConfiguration(name, value).execute(context)
                self.assertEqual(context.launch_configurations['params_file'], '/tmp/grasp.yaml')
            else:
                action.execute(context)
        self.assertEqual(context.launch_configurations['params_file'], '/tmp/navigator.yaml')

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
