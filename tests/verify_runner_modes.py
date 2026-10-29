#!/usr/bin/env python3
"""Exercise actual runner navigation/SLAM switching with synthetic inputs, no hardware or GUI."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runner-config', type=Path, required=True)
parser.add_argument('--workspace', type=Path, required=True)
parser.add_argument('--log-dir', type=Path, required=True)
args = parser.parse_args()
os.environ['ROS_DOMAIN_ID'] = '187'
os.environ['RWH_WORKSPACE'] = str(args.workspace)
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

args.log_dir.mkdir(parents=True, exist_ok=True)
source_map = Path(get_package_share_directory('rwh_beyond_bringup')) / 'maps/real_map_0505.yaml'
test_map = args.log_dir / 'custom map/navigation.yaml'
test_map.parent.mkdir(parents=True, exist_ok=True)
map_data = yaml.safe_load(source_map.read_text())
shutil.copy2(source_map.parent / map_data['image'], test_map.with_suffix('.pgm'))
map_data['image'] = test_map.with_suffix('.pgm').name
test_map.write_text(yaml.safe_dump(map_data))
os.environ['RWH_MAP_FILE'] = str(test_map)
config = yaml.safe_load(args.runner_config.read_text())
for runner in config['runners']:
    assert not runner['on_launch']
    if runner['name'] == 'lidar':
        runner['activate_script'][0] += ' start_driver:=false'
    if runner['name'] in ('navigation', 'slam'):
        runner['activate_script'] = [c.replace('use_rviz:=true', 'use_rviz:=false') + ' start_driver:=false' for c in runner['activate_script']]
    runner['activate_script'] = [command.replace('a200_0724', 'fixture') for command in runner['activate_script']]
fixture = args.log_dir / 'runners.yaml'
fixture.write_text(yaml.safe_dump(config, sort_keys=False))
rclpy.init()
node = rclpy.create_node('runner_modes_verifier')
status = {}
maps = []
mode = 'navigation'

def status_callback(message):
    status.clear()
    status.update({r['name']: r for r in json.loads(message.data)['runners']})

node.create_subscription(String, '/ros_runner/runner_status', status_callback, 10)
node.create_subscription(OccupancyGrid, '/fixture/map', maps.append, 10)
tf_pub = node.create_publisher(TFMessage, '/fixture/tf', 10)
scan_pub = node.create_publisher(LaserScan, '/fixture/scan', 10)

def publish():
    stamp = node.get_clock().now().to_msg()
    transforms = []
    edges = [('odom', 'base_link')]
    if mode == 'navigation':
        edges.append(('map', 'odom'))
    for parent, child in edges:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    tf_pub.publish(TFMessage(transforms=transforms))
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = 'cloud'
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2 * math.pi / 359
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.ranges = [3.0] * 360
    scan_pub.publish(scan)

timer = node.create_timer(0.05, publish)

def spin_until(predicate, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError('Timed out; inspect runner.log: ' + str(status))


def call(name):
    client = node.create_client(Empty, '/ros_runner/' + name)
    try:
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done(), name
    finally:
        node.destroy_client(client)


def wait_active(names):
    clients = {name: node.create_client(GetState, '/fixture/' + name + '/get_state') for name in names}
    try:
        deadline = time.monotonic() + 75
        while time.monotonic() < deadline:
            states = {}
            for name, client in clients.items():
                if not client.wait_for_service(timeout_sec=0.1):
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                if future.done() and future.result():
                    states[name] = future.result().current_state.label
            if all(states.get(name) == 'active' for name in names):
                return
        raise AssertionError(states)
    finally:
        for client in clients.values():
            node.destroy_client(client)


with open(args.log_dir / 'runner.log', 'w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'ros_runner', 'runner.launch.py',
        'config_yaml:=' + str(fixture), 'log_dir:=' + str(args.log_dir / 'logs')],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        spin_until(lambda: bool(status), 15)
        assert not any(s['is_running'] for s in status.values())
        call('activate_lidar')
        spin_until(lambda: status['lidar']['is_running'])
        call('activate_navigation')
        navigation = ['map_server', 'amcl', 'controller_server', 'planner_server', 'smoother_server',
                      'behavior_server', 'velocity_smoother', 'bt_navigator', 'waypoint_follower',
                      'route_server', 'collision_monitor', 'docking_server']
        wait_active(navigation)
        spin_until(lambda: status['navigation']['is_running'] and not status['lidar']['is_running'])
        print('PASS runner lidar -> navigation: standalone sensor stopped; bringup sensor and 12 Nav2 nodes active', flush=True)
        mode = 'slam'
        maps.clear()
        call('activate_slam')
        spin_until(lambda: status['slam']['is_running'] and not status['navigation']['is_running'])
        wait_active(['slam_toolbox', 'map_saver'] + navigation[2:])
        spin_until(lambda: maps and maps[-1].info.width > 0)
        spin_until(lambda: not any(name == 'amcl' and ns == '/fixture' for name, ns in node.get_node_names_and_namespaces()))
        print('PASS runner navigation -> SLAM: mapping, Nav2 active, AMCL stopped', flush=True)
        before_save = test_map.stat().st_mtime_ns
        call('activate_map_save')
        saved_map = test_map
        spin_until(lambda: saved_map.is_file() and saved_map.stat().st_mtime_ns > before_save and saved_map.with_suffix('.pgm').is_file() and yaml.safe_load(saved_map.read_text()).get('image') == 'navigation.pgm' and list(Path('/root/wsyoon_data/maps').glob('*/map.yaml')), 25)
        saved = yaml.safe_load(saved_map.read_text())
        assert (saved_map.parent / saved['image']).is_file()
        print('PASS runner map_save: custom YAML/PGM destination, spaces in path and relative image metadata', flush=True)
        mode = 'navigation'
        call('activate_navigation')
        spin_until(lambda: status['navigation']['is_running'] and not status['slam']['is_running'])
        wait_active(navigation)
        spin_until(lambda: not any(name == 'slam_toolbox' and ns == '/fixture' for name, ns in node.get_node_names_and_namespaces()))
        assert not any(status[n]['is_running'] for n in ('panda', 'panda_sol', 'panda_init_sol'))
        print('PASS runner SLAM -> navigation: AMCL restored, SLAM stopped, Panda never started', flush=True)
        call('deactivate_all')
        spin_until(lambda: not any(s['is_running'] for s in status.values()), 45)
        print('PASS runner dependency shutdown', flush=True)
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=20)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
