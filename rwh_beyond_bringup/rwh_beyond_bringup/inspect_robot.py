"""Read-only graph inspection. Never creates a velocity publisher."""
import argparse
import os
import time
import rclpy
from rclpy.utilities import remove_ros_args
from rwh_beyond_bringup.parameters import topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='a200_0724')
    parser.add_argument('--scan-topic', default='/a200_0724/scan')
    parser.add_argument('--odom-topic', default='platform/odom/filtered')
    parser.add_argument('--cmd-vel-topic', default='cmd_vel')
    parser.add_argument('--cmd-vel-type', choices=['twist', 'twist_stamped'], default='twist')
    args = parser.parse_args(remove_ros_args()[1:])
    print('ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '(default 0)'))
    print('RMW_IMPLEMENTATION=' + os.environ.get('RMW_IMPLEMENTATION', '(default)'))
    for name in ('ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'CYCLONEDDS_URI'):
        print(name + '=' + os.environ.get(name, '(unset)'))
    print('This checks graph/type/QoS visibility only; it does not prove sensor data delivery or robot distro.')
    rclpy.init()
    node = rclpy.create_node('rwh_interface_inspector')
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        expected = [
            (args.scan_topic, 'sensor_msgs/msg/LaserScan', False),
            (args.odom_topic, 'nav_msgs/msg/Odometry', False),
            (args.cmd_vel_topic, 'geometry_msgs/msg/' + (
                'TwistStamped' if args.cmd_vel_type == 'twist_stamped' else 'Twist'), True),
            ('tf', 'tf2_msgs/msg/TFMessage', False),
            ('tf_static', 'tf2_msgs/msg/TFMessage', False),
        ]
        good = True
        for name, kind, subscriptions in expected:
            name = topic(args.namespace, name)
            endpoints = (node.get_subscriptions_info_by_topic(name) if subscriptions
                         else node.get_publishers_info_by_topic(name))
            matched = any(endpoint.topic_type == kind for endpoint in endpoints)
            good = good and matched
            print(('OK ' if matched else 'MISSING/MISMATCH ') + name + ' expected=' + kind)
            for endpoint in endpoints:
                print(' ', endpoint.node_namespace, endpoint.node_name, endpoint.topic_type,
                      'reliability=', endpoint.qos_profile.reliability,
                      'durability=', endpoint.qos_profile.durability)
        return 0 if good else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
