#!/usr/bin/env python3
"""Compare the real legacy static publisher and the new namespaced scan launch."""
import math
import os
os.environ['ROS_DOMAIN_ID'] = '182'
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    processes = []
    node = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            for command in [
                ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.402', '0', '0.156', '0', '0', '3.141592', 'base_link', 'cloud',
                 '--ros-args', '-r', '/tf_static:=/reference/tf_static'],
                ['ros2', 'launch', 'rwh_beyond_bringup', 'sensor.launch.py', 'namespace:=a200_0724', 'start_driver:=false'],
            ]:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True))
            # Join after publication to exercise transient-local delivery.
            time.sleep(2)
            node = rclpy.create_node('scan_tf_verifier', cli_args=[
                '--ros-args', '-r', '/tf_static:=/a200_0724/tf_static', '-r', '/tf:=/a200_0724/tf'])
            reference = []
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(TFMessage, '/reference/tf_static', reference.append, qos)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if reference and buffer.can_transform('base_link', 'cloud', Time()):
                    break
            assert reference, 'Legacy reference transform missing'
            actual = buffer.lookup_transform('base_link', 'cloud', Time())
            original = next(t for message in reference for t in message.transforms if t.child_frame_id == 'cloud')
            assert actual.header.frame_id == original.header.frame_id == 'base_link'
            assert actual.child_frame_id == original.child_frame_id == 'cloud'
            for field in ('translation', 'rotation'):
                for axis in ('x', 'y', 'z') + (('w',) if field == 'rotation' else ()):
                    assert math.isclose(getattr(getattr(actual.transform, field), axis),
                                        getattr(getattr(original.transform, field), axis), abs_tol=1e-9)
            assert abs(actual.transform.rotation.x) > 0.999, 'Expected roll pi, not yaw pi'
            print('PASS legacy transform equivalence, namespaced /tf_static, late-joiner TF lookup', flush=True)
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            for process in processes:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if node:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
