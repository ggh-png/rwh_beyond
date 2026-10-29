#!/usr/bin/env python3
"""Verify installed RWH launch/params/maps and sensor TF without sensor access or goals."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--log-dir', type=Path, required=True)
parser.add_argument('--composition', choices=['true', 'false'], default='true')
args = parser.parse_args()
os.environ['ROS_DOMAIN_ID'] = '185'
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

share = Path(get_package_share_directory('rwh_beyond_bringup'))
for name in ('sensor', 'navigation', 'localization', 'slam'):
    assert (share / 'launch' / (name + '.launch.py')).is_file()
for name in ('nav2_params.yaml',):
    assert (share / 'params' / name).is_file()
map_path = share / 'maps/real_map_0505.yaml'
map_data = yaml.safe_load(map_path.read_text())
assert (map_path.parent / map_data['image']).is_file()
args.log_dir.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('rwh_bringup_verifier', cli_args=[
    '--ros-args', '-r', '/tf:=/fixture/tf', '-r', '/tf_static:=/fixture/tf_static'])
buffer = Buffer()
listener = TransformListener(buffer, node)
tf_pub = node.create_publisher(TFMessage, '/fixture/tf', 10)
scan_pub = node.create_publisher(LaserScan, '/fixture/scan', 10)


def publish():
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
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
    scan.angle_increment = 2 * math.pi / 63
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.ranges = [float('inf')] * 64
    scan_pub.publish(scan)


timer = node.create_timer(0.025, publish)
with open(args.log_dir / 'launch.log', 'w') as log:
    process = subprocess.Popen([
        'ros2', 'launch', 'rwh_beyond_bringup', 'bringup.launch.py',
        'namespace:=fixture', 'start_driver:=false', 'autostart:=true', 'use_rviz:=false',
        'use_composition:=' + args.composition, 'slam:=false', 'use_localization:=true',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        names = ['map_server', 'amcl', 'controller_server', 'planner_server', 'smoother_server',
                 'behavior_server', 'velocity_smoother', 'bt_navigator', 'waypoint_follower',
                 'route_server', 'collision_monitor', 'docking_server']
        states = {}
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Bringup exited; inspect launch.log'
            for name in names:
                client = node.create_client(GetState, f'/fixture/{name}/get_state')
                try:
                    if not client.wait_for_service(timeout_sec=0.15):
                        continue
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
                    if future.done() and future.result():
                        states[name] = future.result().current_state.label
                finally:
                    node.destroy_client(client)
            if all(states.get(name) == 'active' for name in names):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        assert all(states.get(name) == 'active' for name in names), states
        transform = buffer.lookup_transform('base_link', 'cloud', Time()).transform
        assert math.isclose(transform.translation.x, 0.402)
        assert abs(transform.rotation.x) > 0.999
        graph = node.get_node_names_and_namespaces()
        assert ('rwh_scan_static_tf', '/fixture') in graph, graph
        assert not any('/fixture/fixture' in ns for _, ns in graph), graph
        publishers = node.get_publishers_info_by_topic('/fixture/cmd_vel')
        assert any(p.node_name == 'collision_monitor' and p.topic_type == 'geometry_msgs/msg/Twist'
                   for p in publishers), publishers
        print('PASS RWH local launches/params/packaged map, 12 active Nav2 nodes, namespaced sensor TF; composition=' + args.composition)
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
