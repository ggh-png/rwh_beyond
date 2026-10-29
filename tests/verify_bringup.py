#!/usr/bin/env python3
"""Activate navigation against synthetic TF/scan/map, without sending a goal."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--cmd-vel-type', choices=['twist', 'twist_stamped'], default='twist')
    parser.add_argument('--cloud-frame', action='store_true')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '181'
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.srv import GetState
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    import yaml
    args.log_dir.mkdir(parents=True, exist_ok=True)
    (args.log_dir / 'map.pgm').write_bytes(b'P5\n100 100\n255\n' + bytes([254]) * 10000)
    (args.log_dir / 'map.yaml').write_text(yaml.safe_dump({
        'image': str(args.log_dir / 'map.pgm'), 'resolution': 0.05,
        'origin': [-2.5, -2.5, 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}))
    rclpy.init()
    node = rclpy.create_node('navigation_launch_verifier')
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
        scan.header.frame_id = 'cloud' if args.cloud_frame else 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 63
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [float('inf')] * 64
        scan_pub.publish(scan)

    timer = node.create_timer(0.025, publish)
    with open(args.log_dir / 'launch.log', 'w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'rwh_beyond_bringup', 'navigation.launch.py',
            'map:=' + str(args.log_dir / 'map.yaml'),
            'namespace:=fixture', 'autostart:=true',
            'cmd_vel_type:=' + args.cmd_vel_type,
            'publish_scan_tf:=' + ('true' if args.cloud_frame else 'false')], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            states = {}
            names = ['map_server', 'amcl', 'controller_server', 'planner_server', 'smoother_server',
                     'behavior_server', 'velocity_smoother', 'bt_navigator', 'waypoint_follower',
                     'route_server', 'collision_monitor', 'docking_server', 'beyond_server']
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                for name in names:
                    client = node.create_client(GetState, f'/fixture/{name}/get_state')
                    try:
                        if not client.wait_for_service(timeout_sec=0.2):
                            continue
                        future = client.call_async(GetState.Request())
                        rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                        if future.done():
                            states[name] = future.result().current_state.label
                    finally:
                        node.destroy_client(client)
                if all(states.get(name) == 'active' for name in names):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            assert all(states.get(name) == 'active' for name in names), states
            publishers = node.get_publishers_info_by_topic('/fixture/cmd_vel')
            expected_type = 'geometry_msgs/msg/' + ('Twist' if args.cmd_vel_type == 'twist' else 'TwistStamped')
            assert any(p.node_name == 'collision_monitor' and p.topic_type == expected_type
                       for p in publishers), publishers
            subscribers = node.get_subscriptions_info_by_topic('/fixture/scan')
            assert len(subscribers) >= 3, subscribers
            if args.cloud_frame:
                assert any(p.node_name == 'rwh_scan_static_tf' for p in
                           node.get_publishers_info_by_topic('/fixture/tf_static'))
            from rwh_beyond_msgs.srv import GetStatus
            client = node.create_client(GetStatus, '/fixture/beyond_server/get_status')
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(GetStatus.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result().active
            node.destroy_client(client)
            print('PASS navigation: thirteen lifecycle nodes active; stock nav2_bringup;', expected_type, flush=True)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
