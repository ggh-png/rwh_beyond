#!/usr/bin/env python3
"""Inspect resolved sensor actions without starting a driver or contacting hardware."""
import importlib.util
from pathlib import Path
from unittest.mock import patch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sensor_launch', root / 'rwh_beyond_bringup/launch/sensor.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def inspect(overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    nodes = []
    with patch.object(module, 'Node', side_effect=lambda **kwargs: nodes.append(kwargs)):
        description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for action in module.prepare_sensor_configuration(context):
        action.execute(context)
    return context, nodes


def arguments(node, context):
    return [perform_substitutions(context, normalize_to_list_of_substitutions(arg))
            for arg in node['arguments']]


context, nodes = inspect({'namespace': '/a200_0724'})
static_tf, driver = nodes
assert context.launch_configurations['_sensor_namespace'] == 'a200_0724'
assert Path(arguments(driver, context)[0]).is_file()
assert arguments(driver, context)[1:7] == [
    'hostname:=172.16.0.161', 'min_ang:=-1.570796327', 'max_ang:=1.570796327',
    'frame_id:=cloud', 'publish_frame_id:=cloud', 'tf_publish_rate:=0']
assert ('/scan', 'scan') in driver['remappings']
assert static_tf['condition'].evaluate(context)
flags = arguments(static_tf, context)
assert flags[flags.index('--roll') + 1] == '3.141592'

# Existing runner arguments remain valid; explicit new names take precedence.
context, nodes = inspect({'hostname': '10.0.0.1', 'sensor_hostname': '10.0.0.2',
                          'publish_scan_tf': 'true', 'use_tf_publisher': 'False',
                          'start_driver': 'false', 'use_sim_time': 'True'})
assert arguments(nodes[1], context)[1] == 'hostname:=10.0.0.2'
assert not nodes[0]['condition'].evaluate(context)
assert context.launch_configurations['_sensor_driver_file'] == ''
try:
    inspect({'min_ang': '2', 'max_ang': '1'})
    raise AssertionError('Invalid angle interval accepted')
except ValueError as error:
    assert 'min_ang' in str(error)
print('PASS sensor arguments, legacy aliases, TF configuration, disabled-driver mode and angle validation')
