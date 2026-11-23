"""Verify the BT demo launch starts both grasp bridges and the BT navigator."""

import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml


def launch_group(entities, filename):
    for group in entities:
        if isinstance(group, GroupAction):
            for action in group.get_sub_entities():
                if isinstance(action, IncludeLaunchDescription):
                    source = action.launch_description_source
                    source.get_launch_description(LaunchContext())
                    if Path(source.location).name == filename:
                        return group, action
    raise AssertionError(f'Launch group not found for {filename}')


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
             'target_text', 'endpoint', 'mode', 'start_sensors', 'enable_lidar',
             'enable_d405', 'd405_serial', 'd405_profile', 'sensor_params_file',
             'camera_type', 'enable_d435', 'd435_serial', 'd435_depth_profile', 'd435_color_profile',
             'd435_mount_parent', 'd435_mount_xyz', 'd435_mount_rpy',
             'd405_mount_parent', 'd405_mount_xyz', 'd405_mount_rpy'})
        rviz_group, _ = launch_group(entities, 'rviz.launch.py')
        env_vars = {
            'segmentation_endpoint': ('RWH_SEGMENTATION_ENDPOINT', ''),
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
        self.assertTrue(rviz_group.condition.evaluate(context))
        context.launch_configurations['rviz'] = 'false'
        self.assertFalse(rviz_group.condition.evaluate(context))
        self.assertEqual(''.join(part.perform(context) for part in argument_map['rviz'].default_value),
                         'true')
        self.assertEqual(
            [(node.node_package, node.node_executable) for node in nodes],
            [('nav2_bt_navigator', 'bt_navigator'),
             ('nav2_lifecycle_manager', 'lifecycle_manager')],
        )

    def test_direct_bridges_use_shared_yaml_and_optional_endpoints(self):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'bt_demo.launch.py'
        spec = importlib.util.spec_from_file_location('bt_bridges', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        action, = (entity for entity in module.generate_launch_description().entities
                   if isinstance(entity, OpaqueFunction))
        context = LaunchContext()
        context.launch_configurations.update(
            mode='sim', params_file='/tmp/bt.yaml',
            segmentation_endpoint='tcp://127.0.0.1:5567', endpoint='',
        )
        sam3, graspgenx = action.execute(context)
        self.assertEqual((sam3.node_package, sam3.node_executable),
                         ('rwh_beyond_sam3_bridge', 'segmentation_action_server'))
        self.assertEqual((graspgenx.node_package, graspgenx.node_executable),
                         ('rwh_beyond_graspgenx_bridge', 'inference_action_server'))
        self.assertEqual(evaluate_parameters(context, sam3._Node__parameters),
                         (Path('/tmp/bt.yaml'), {'use_sim_time': True},
                          {'endpoint': 'tcp://127.0.0.1:5567'}))
        self.assertEqual(evaluate_parameters(context, graspgenx._Node__parameters),
                         (Path('/tmp/bt.yaml'), {'use_sim_time': True}))

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
        rviz_group, rviz = launch_group(entities, 'rviz.launch.py')
        context.launch_configurations['rviz'] = 'true'
        self.assertTrue(rviz_group.condition.evaluate(context))
        rviz_args = {name: value.perform(context) if hasattr(value, 'perform') else value
                     for name, value in rviz.launch_arguments}
        self.assertEqual(rviz_args['use_sim_time'], 'false')
        self.assertEqual(rviz_args['use_moveit'], 'true')
        self.assertTrue(rviz_args['rviz_config'].endswith('/rwh_beyond_bringup/rviz/depth_planning.rviz'))
        # Evaluate the actual child argument declaration, including its choices.
        rviz_path = Path(__file__).resolve().parents[1] / 'launch/rviz.launch.py'
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
        self.assertFalse(rviz_group.condition.evaluate(context))
        navigator = next(n for n in entities if isinstance(n, Node) and n.node_executable == 'bt_navigator')
        params = evaluate_parameters(context, navigator._Node__parameters)[-1]
        self.assertFalse(params['use_sim_time'])
        # YAML-selected tree paths must not be overwritten by launch defaults.
        self.assertNotIn('default_demo_bt_xml', params)
        self.assertNotIn('default_place_bt_xml', params)
        yaml_layer = evaluate_parameters(context, navigator._Node__parameters)[0]
        self.assertEqual(str(yaml_layer), context.launch_configurations['params_file'])
        context.launch_configurations['mode'] = 'sim'
        sim_navigator = next(n for n in module.generate_launch_description().entities
                             if isinstance(n, Node) and n.node_executable == 'bt_navigator')
        sim_params = evaluate_parameters(context, sim_navigator._Node__parameters)[-1]
        self.assertTrue(sim_params['use_sim_time'])
        self.assertNotIn('default_demo_bt_xml', sim_params)
        self.assertNotIn('default_place_bt_xml', sim_params)

    def test_shared_parameters_preserve_required_navigator_settings(self):
        params_path = Path(__file__).resolve().parents[1] / 'config' / 'bt_demo.yaml'
        params = yaml.safe_load(params_path.read_text())
        expected = {
            'bt_navigator': {'ros__parameters': {
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
                'autostart': True,
                'node_names': ['bt_navigator'],
            }},
        }
        for node, settings in expected.items():
            actual = params[node]['ros__parameters']
            for key, value in settings['ros__parameters'].items():
                self.assertEqual(actual[key], value)
            self.assertIsInstance(actual['use_sim_time'], bool)


if __name__ == '__main__':
    unittest.main()



def test_d435_sensor_include_is_scoped_and_does_not_override_main_rviz():
    spec=importlib.util.spec_from_file_location('d435_bt',Path(__file__).resolve().parents[1]/'launch/bt_demo.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    entities=module.generate_launch_description().entities
    context=LaunchContext()
    for action in entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    context.launch_configurations.update(mode='real',camera_type='d435',rviz='true')
    group, include = launch_group(entities, 'd435.launch.py')
    assert include.condition.evaluate(context)
    assert dict(include.launch_arguments)['pipeline_topics']=='true'
    assert dict(include.launch_arguments)['rviz']=='false'
    for action in group.execute(context):
        if isinstance(action,IncludeLaunchDescription):
            for key,value in action.launch_arguments:SetLaunchConfiguration(key,value).execute(context)
        else:action.execute(context)
    assert context.launch_configurations['rviz']=='true'
    context.launch_configurations['start_sensors']='false'
    assert not include.condition.evaluate(context)
    rviz_group, _ = launch_group(entities, 'rviz.launch.py')
    assert rviz_group.condition.evaluate(context)
