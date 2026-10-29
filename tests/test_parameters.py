from pathlib import Path
import copy
import unittest
import yaml
from ament_index_python.packages import get_package_share_directory
from rwh_beyond_bringup.parameters import render_parameters
from rwh_beyond_bringup.scan_tf import transform_arguments


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.share = Path(get_package_share_directory('rwh_beyond_bringup'))
        self.config = yaml.safe_load((self.share / 'params/nav2_params.yaml').read_text())

    def test_all_upstream_sections_are_present(self):
        upstream = yaml.safe_load((self.share / 'params/upstream/nav2_params.jazzy-1.3.13.yaml').read_text())
        self.assertEqual(set(upstream), set(self.config))
        self.assertEqual(len(self.config), 15)

    def test_tuning_edits_are_preserved(self):
        controller = self.config['controller_server']['ros__parameters']['FollowPath']
        controller['vx_max'] = 0.073
        controller['plugin'] = 'my_robot::CustomController'
        self.config['global_costmap']['global_costmap']['ros__parameters']['resolution'] = 0.07
        self.config['custom_server'] = {'ros__parameters': {'my_parameter': [1, 2, 3]}}
        before = copy.deepcopy(self.config)
        result = render_parameters(self.config, 'fixture')
        self.assertEqual(result['controller_server']['ros__parameters']['FollowPath'], controller)
        self.assertEqual(result['global_costmap']['global_costmap']['ros__parameters']['resolution'], 0.07)
        self.assertEqual(result['custom_server']['ros__parameters']['my_parameter'], [1, 2, 3])
        self.assertEqual(before, self.config)

    def test_namespace_and_cloud_frames(self):
        result = render_parameters(self.config, 'a200_0724')
        self.assertEqual(result['amcl']['ros__parameters']['scan_topic'], '/a200_0724/scan')
        self.assertEqual(result['velocity_smoother']['ros__parameters']['odom_topic'],
                         '/a200_0724/platform/odom/filtered')
        sensor = yaml.safe_load((self.share / 'params/sensor_tf.yaml').read_text())
        self.assertIsNone(transform_arguments(sensor))
        sensor['scan_static_tf']['enabled'] = True
        arguments = transform_arguments(sensor)
        flags = dict(zip(arguments[::2], arguments[1::2]))
        self.assertEqual(flags['--child-frame-id'], 'cloud')
        self.assertEqual(float(flags['--roll']), 3.141592)

    def test_global_namespace(self):
        result = render_parameters(self.config, '')
        self.assertEqual(result['amcl']['ros__parameters']['scan_topic'], '/scan')
        self.assertNotIn('//', result['collision_monitor']['ros__parameters']['cmd_vel_out_topic'])

    def test_velocity_type_override(self):
        nodes = ('controller_server', 'behavior_server', 'velocity_smoother',
                 'collision_monitor', 'docking_server')
        for kind in ('twist', 'twist_stamped'):
            result = render_parameters(self.config, 'fixture', cmd_vel_type=kind)
            for name in nodes:
                self.assertEqual(result[name]['ros__parameters']['enable_stamped_cmd_vel'],
                                 kind == 'twist_stamped')

    def test_time_override(self):
        result = render_parameters(self.config, 'fixture', use_sim_time=True)
        self.assertTrue(result['amcl']['ros__parameters']['use_sim_time'])
        self.assertTrue(result['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'])

    def test_invalid_velocity_type(self):
        with self.assertRaises(ValueError):
            render_parameters(self.config, 'fixture', cmd_vel_type='invalid')


if __name__ == '__main__':
    unittest.main()
