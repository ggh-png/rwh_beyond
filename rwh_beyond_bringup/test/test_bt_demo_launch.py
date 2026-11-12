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


def pipeline_group(entities):
    return next(group for group in entities if isinstance(group, GroupAction)
                and any(isinstance(child, IncludeLaunchDescription) and
                        'grasp_pipeline.launch.py' in child.launch_description_source.location
                        for child in group.get_sub_entities()))


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
             'segmentation_backend', 'target_text', 'endpoint', 'mode', 'start_sensors', 'enable_lidar',
             'enable_d405', 'd405_serial', 'd405_profile', 'sensor_params_file'})
        group = pipeline_group(entities)
        pipeline = next(entity for entity in group.get_sub_entities()
                        if isinstance(entity, IncludeLaunchDescription))
        rviz = next(entity for entity in entities if isinstance(entity, IncludeLaunchDescription))
        self.assertIsNone(pipeline.condition)
        context = LaunchContext()
        context.launch_configurations['mode'] = 'sim'
        self.assertIn("FindPackageShare(pkg='rwh_beyond_bringup')",
                      rviz.launch_description_source.location)
        self.assertIn('moveit_rviz.launch.py', rviz.launch_description_source.location)
        names = ('segmentation_endpoint', 'segmentation_backend', 'target_text', 'endpoint')
        context.launch_configurations.update({name: f'test_{name}' for name in names})
        context.launch_configurations['params_file'] = '/tmp/bt.yaml'
        self.assertEqual(
            {name: value.perform(context) if hasattr(value, 'perform') else value for name, value in pipeline.launch_arguments},
            {**{name: f'test_{name}' for name in names}, 'params_file': '/tmp/bt.yaml', 'sam3_params_file': '/tmp/bt.yaml', 'graspgenx_params_file': '/tmp/bt.yaml', 'use_sim_time': 'True', 'start_coordinator': 'true', 'mode': 'sim'})
        env_vars = {
            'segmentation_endpoint': ('RWH_SEGMENTATION_ENDPOINT', ''),
            'segmentation_backend': ('RWH_SEGMENTATION_BACKEND', ''),
            'target_text': ('RWH_GRASP_TARGET_TEXT', ''),
            'endpoint': ('RWH_GRASP_ENDPOINT', ''),
        }
        with patch.dict(os.environ, {}, clear=True):
            context = LaunchContext()
            context.launch_configurations['mode'] = 'sim'
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

    def test_shared_yaml_remains_scoped(self):
        launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_demo_scoping', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        group = pipeline_group(module.generate_launch_description().entities)
        context = LaunchContext()
        context.launch_configurations.update({
            'mode': 'sim',
            'params_file': '/tmp/navigator.yaml',
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
                self.assertEqual(context.launch_configurations['params_file'], '/tmp/navigator.yaml')
            else:
                action.execute(context)
        self.assertEqual(context.launch_configurations['params_file'], '/tmp/navigator.yaml')

    def test_real_mode_starts_physical_bt_with_wall_clock(self):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_real', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        entities = module.generate_launch_description().entities
        context = LaunchContext()
        for entity in entities:
            if isinstance(entity, DeclareLaunchArgument): entity.execute(context)
        context.launch_configurations['mode'] = 'real'
        for node in entities:
            if isinstance(node, Node): self.assertIsNone(node.condition)
        include = next(child for child in pipeline_group(entities).get_sub_entities()
                       if isinstance(child, IncludeLaunchDescription))
        args = {name: value.perform(context) if hasattr(value, 'perform') else value for name, value in include.launch_arguments}
        self.assertEqual(args['use_sim_time'], 'False')
        self.assertEqual(args['start_coordinator'], 'true')
        self.assertEqual(args['mode'], 'real')
        rviz = next(entity for entity in entities if isinstance(entity, IncludeLaunchDescription))
        context.launch_configurations['rviz'] = 'true'
        self.assertTrue(rviz.condition.evaluate(context))
        rviz_args = {name: value.perform(context) if hasattr(value, 'perform') else value
                     for name, value in rviz.launch_arguments}
        self.assertEqual(rviz_args['use_sim_time'], 'false')
        # Evaluate the actual child argument declaration, including its choices.
        rviz_path = Path(__file__).resolve().parents[1] / 'launch/moveit_rviz.launch.py'
        rviz_spec = importlib.util.spec_from_file_location('real_rviz_args', rviz_path)
        rviz_module = importlib.util.module_from_spec(rviz_spec)
        rviz_spec.loader.exec_module(rviz_module)
        child_context = LaunchContext()
        child_context.launch_configurations.update(rviz_args)
        for declaration in rviz_module.generate_launch_description().entities:
            if isinstance(declaration, DeclareLaunchArgument):
                declaration.execute(child_context)
        context.launch_configurations['mode'] = 'sim'
        self.assertEqual(dict(rviz.launch_arguments)['use_sim_time'].perform(context), 'true')
        context.launch_configurations['mode'] = 'real'
        context.launch_configurations['rviz'] = 'false'
        self.assertFalse(rviz.condition.evaluate(context))
        from launch_ros.utilities import evaluate_parameters
        navigator = next(n for n in entities if isinstance(n, Node) and n.node_executable == 'bt_navigator')
        params = evaluate_parameters(context, navigator._Node__parameters)[-1]
        self.assertFalse(params['use_sim_time'])
        self.assertTrue(params['default_demo_bt_xml'].endswith('/demo.xml'))
        self.assertTrue(params['default_place_bt_xml'].endswith('/place.xml'))
        real_tree = params['default_demo_bt_xml']
        real_place_tree = params['default_place_bt_xml']
        context.launch_configurations['mode'] = 'sim'
        sim_navigator = next(n for n in module.generate_launch_description().entities
                             if isinstance(n, Node) and n.node_executable == 'bt_navigator')
        sim_params = evaluate_parameters(context, sim_navigator._Node__parameters)[-1]
        self.assertTrue(sim_params['use_sim_time'])
        self.assertEqual(sim_params['default_demo_bt_xml'], real_tree)
        self.assertEqual(sim_params['default_place_bt_xml'], real_place_tree)

    def test_default_parameters_preserve_previous_values(self):
        params_path = Path(__file__).resolve().parents[1] / 'config' / 'bt_demo.yaml'
        params = yaml.safe_load(params_path.read_text())
        self.assertEqual({key: params[key] for key in ('bt_navigator', 'bt_lifecycle_manager')}, {
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
