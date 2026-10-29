#!/usr/bin/env python3
"""Exercise mapping and map_saver with synthetic scan/TF, without a robot."""
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
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '184'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node('mapping_verifier')
    tf_pub = node.create_publisher(TFMessage, '/fixture/tf', 10)
    static_pub = node.create_publisher(TFMessage, '/fixture/tf_static',
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scan_pub = node.create_publisher(LaserScan, '/fixture/scan', 10)
    maps = []
    node.create_subscription(OccupancyGrid, '/fixture/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish():
        stamp = node.get_clock().now().to_msg()
        odom = TransformStamped()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[odom]))
        sensor = TransformStamped()
        sensor.header.stamp = stamp
        sensor.header.frame_id = 'base_link'
        sensor.child_frame_id = 'cloud'
        sensor.transform.translation.x = 0.402
        sensor.transform.translation.z = 0.156
        sensor.transform.rotation.x = math.sin(3.141592 / 2)
        sensor.transform.rotation.w = math.cos(3.141592 / 2)
        static_pub.publish(TFMessage(transforms=[sensor]))
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'cloud'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 359
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [3.0] * 360
        scan_pub.publish(scan)

    timer = node.create_timer(0.1, publish)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    with open(args.log_dir / 'slam.log', 'w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'rwh_beyond_bringup', 'slam.launch.py',
            'namespace:=fixture', 'autostart:=true'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        saver = None
        try:
            deadline = time.monotonic() + 30
            while not maps and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert maps and maps[-1].info.width > 0 and maps[-1].info.height > 0
            assert maps[-1].header.frame_id == 'map'
            assert math.isclose(maps[-1].info.resolution, 0.025, rel_tol=1e-5)
            with open(args.log_dir / 'save.log', 'w') as save_log:
                saver = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-t', '/fixture/map', '-f', str(args.log_dir / 'map'),
                    '--ros-args', '-p', 'save_map_timeout:=8.0'],
                    stdout=save_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 15
                while saver.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert saver.poll() == 0
                assert (args.log_dir / 'map.yaml').exists()
                assert (args.log_dir / 'map.pgm').exists()
            print('PASS SLAM cloud scan -> namespaced map at 0.025m -> saved YAML/PGM', flush=True)
        finally:
            for child in (saver, process):
                if child is None or child.poll() is not None:
                    continue
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
