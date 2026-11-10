"""Exercise Groot2 monitoring through the real navigator without robot commands."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import unittest

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
import yaml


class GrootMonitoringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='groot-monitor-test-')
        self.root = Path(self.directory.name)
        self.node = rclpy.create_node('groot_monitor_test')
        self.process = None
        self.log = (self.root / 'navigator.log').open('w')
        self.addCleanup(self.cleanup_runtime)

    def start_navigator(self, enabled, port):
        tree = self.root / 'tree.xml'
        tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'
                        '<AlwaysSuccess/></BehaviorTree></root>')
        config = self.root / 'params.yaml'
        config.write_text(yaml.safe_dump({'bt_navigator': {'ros__parameters': {
            'use_sim_time': False,
            'navigators': ['demo'],
            'demo.plugin': 'rwh_beyond_navigator::DemoNavigator',
            'plugin_lib_names': ['rwh_beyond_bt_nodes'],
            'default_demo_bt_xml': str(tree),
            'run_demo.enable_groot_monitoring': enabled,
            'run_demo.groot_server_port': port,
        }}}))
        self.process = subprocess.Popen(
            ['ros2', 'run', 'nav2_bt_navigator', 'bt_navigator', '--ros-args',
             '-r', '__node:=bt_navigator', '--params-file', str(config)],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.transition = self.node.create_client(ChangeState, '/bt_navigator/change_state')
        self.assertTrue(self.transition.wait_for_service(timeout_sec=15))

    def change_state(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        future = self.transition.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)
        self.log.flush()
        self.assertTrue(future.done(), (self.root / 'navigator.log').read_text())
        self.assertIsNotNone(future.result())
        return future.result().success

    @staticmethod
    def unused_port_pair():
        for port in range(25000, 30000):
            sockets = [socket.socket(), socket.socket()]
            try:
                sockets[0].bind(('127.0.0.1', port))
                sockets[1].bind(('127.0.0.1', port + 1))
                return port
            except OSError:
                pass
            finally:
                for sock in sockets:
                    sock.close()
        raise RuntimeError('No free adjacent TCP ports for Groot2 test')

    @staticmethod
    def is_listening(port):
        with socket.socket() as sock:
            sock.settimeout(1)
            return sock.connect_ex(('127.0.0.1', port)) == 0

    def test_enabled_monitor_and_cleanup(self):
        port = self.unused_port_pair()
        self.start_navigator(True, port)
        self.assertTrue(self.change_state(Transition.TRANSITION_CONFIGURE))
        self.assertTrue(self.change_state(Transition.TRANSITION_ACTIVATE))
        self.assertTrue(self.is_listening(port))
        self.assertTrue(self.is_listening(port + 1))
        self.assertTrue(self.change_state(Transition.TRANSITION_DEACTIVATE))
        self.assertTrue(self.change_state(Transition.TRANSITION_CLEANUP))
        self.assertFalse(self.is_listening(port))
        self.assertFalse(self.is_listening(port + 1))

    def test_monitor_can_be_disabled(self):
        port = self.unused_port_pair()
        self.start_navigator(False, port)
        self.assertTrue(self.change_state(Transition.TRANSITION_CONFIGURE))
        self.assertTrue(self.change_state(Transition.TRANSITION_ACTIVATE))
        self.assertFalse(self.is_listening(port))
        self.assertFalse(self.is_listening(port + 1))

    def test_invalid_port_fails_configuration(self):
        self.start_navigator(True, 65535)
        self.assertFalse(self.change_state(Transition.TRANSITION_CONFIGURE))
        self.assertIn('must be between 1 and 65534', (self.root / 'navigator.log').read_text())

    def cleanup_runtime(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.node.destroy_node()
        self.log.close()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main()
