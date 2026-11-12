"""Exercise the actual navigator with a mock MoveIt server; no hardware/simulation."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data
from rwh_beyond_msgs.action import RunDemo, RunGraspStage
import yaml


class NavigatorErrorReporting(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='rwh-error-reporting-')
        self.root = Path(self.directory.name)
        tree = self.root / 'tree.xml'
        tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'
                        '<MoveHome name="return_home"/></BehaviorTree></root>')
        config = self.root / 'params.yaml'
        config.write_text(yaml.safe_dump({'bt_navigator': {'ros__parameters': {
            'use_sim_time': False, 'global_frame': 'base_link', 'robot_base_frame': 'base_link',
            'navigators': ['demo'], 'demo.plugin': 'rwh_beyond_navigator::DemoNavigator',
            'plugin_lib_names': ['rwh_beyond_bt_nodes'], 'default_demo_bt_xml': str(tree),
            'wait_for_service_timeout': 3000, 'default_server_timeout': 1000,
            'error_code_names': ['nav2_unused_error_code'],
        }}}))
        self.code = -3
        self.message = 'Trajectory invalid after scene update'
        self.source = 'plan_execution'
        self.reject = False
        self.node = rclpy.create_node('navigator_error_mock')
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(self.node, MoveGroup, 'move_action', self.execute,
                                   goal_callback=self.accept, callback_group=self.group)
        self.action = ActionClient(self.node, RunDemo, 'run_demo', callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.log = (self.root / 'navigator.log').open('w')
        self.process = subprocess.Popen(
            ['ros2', 'run', 'nav2_bt_navigator', 'bt_navigator', '--ros-args',
             '-r', '__node:=bt_navigator', '--params-file', str(config)],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(self.cleanup_runtime)
        transition = self.node.create_client(ChangeState, '/bt_navigator/change_state')
        self.wait(transition.service_is_ready)
        for step in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = step
            future = transition.call_async(request)
            self.wait(future.done)
            self.assertTrue(future.result().success, (self.root / 'navigator.log').read_text())
        self.wait(self.action.server_is_ready)

    def accept(self, _):
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def execute(self, handle):
        feedback = MoveGroup.Feedback()
        feedback.state = 'MONITOR'
        handle.publish_feedback(feedback)
        time.sleep(0.05)
        result = MoveGroup.Result()
        result.error_code.val = self.code
        result.error_code.message = self.message
        result.error_code.source = self.source
        if self.code == 1:
            handle.succeed()
        else:
            handle.abort()
        return result

    def wait(self, predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(predicate(), 'Timed out: ' + str(self.root))

    def run_goal(self, tree="", target_text=""):
        feedback = []
        sent = self.action.send_goal_async(RunDemo.Goal(behavior_tree=tree, target_text=target_text),
                                            feedback_callback=lambda msg: feedback.append(msg.feedback))
        self.wait(sent.done)
        self.assertTrue(sent.result().accepted)
        completed = sent.result().get_result_async()
        self.wait(completed.done)
        return completed.result(), feedback

    def test_signed_failure_feedback_result_and_success_reset(self):
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 6)  # ROS action ABORTED
        result = wrapped.result
        self.assertEqual(result.error_code, -3)
        self.assertEqual(result.error_name, 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE')
        self.assertEqual(result.error_source, 'plan_execution')
        self.assertEqual(result.error_msg, self.message)
        self.assertEqual(result.failed_stage, 'move_home')
        self.assertEqual(result.failed_node, 'return_home')
        self.assertEqual(result.moveit_state, 'MONITOR')
        self.assertTrue(any(f.error_code == -3 and f.error_msg == self.message for f in feedback))
        self.message = ''
        wrapped, feedback = self.run_goal()
        self.assertIn('remaining trajectory became invalid', wrapped.result.error_msg)
        self.code = 1
        self.source = ''
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 4)  # ROS action SUCCEEDED
        self.assertEqual(wrapped.result.error_code, 0)
        self.assertEqual(wrapped.result.failed_stage, '')
        self.assertEqual(wrapped.result.error_source, '')
        self.assertTrue(all(f.error_code == 0 for f in feedback))
        self.reject = True
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 6)
        self.assertEqual(wrapped.result.error_name, 'ACTION_REQUEST_FAILED')
        self.assertNotEqual(wrapped.result.error_code, 0)

    def test_clear_plugin_runs_in_tree_and_reuses_success_across_goals(self):
        counts = {'clear': 0}

        def clear(request, response):
            counts['clear'] += 1
            return response

        def scene(request, response):
            response.scene.world.octomap.octomap.data = [1, 2]
            return response

        publisher = self.node.create_publisher(PointCloud2, '/camera/depth/filtered_points', qos_profile_sensor_data)

        def publish():
            if counts['clear']:
                message = PointCloud2(height=1, width=1)
                message.header.stamp = self.node.get_clock().now().to_msg()
                publisher.publish(message)

        clear_service = self.node.create_service(Empty, '/clear_octomap', clear, callback_group=self.group)
        scene_service = self.node.create_service(GetPlanningScene, '/get_planning_scene', scene, callback_group=self.group)
        timer = self.node.create_timer(0.02, publish, callback_group=self.group)
        tree = self.root / 'clear_tree.xml'
        tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test"><Sequence>'
                        '<ClearOctomap timeout="2"/>'
                        '<MakeTcpPose x="0" y="0" z="0.3" pose="{target}"/>'
                        '</Sequence></BehaviorTree></root>')
        for _ in range(2):
            wrapped, feedback = self.run_goal(str(tree))
            self.assertEqual(wrapped.status, 4)
            self.assertEqual(wrapped.result.error_code, 0)
        self.assertEqual(counts['clear'], 1)
        self.node.destroy_timer(timer)
        self.node.destroy_service(clear_service)
        self.node.destroy_service(scene_service)
        self.node.destroy_publisher(publisher)

    def test_target_text_reaches_segmentation_and_resets_between_goals(self):
        received = []

        def segment(handle):
            received.append(handle.request.target_text)
            result = RunGraspStage.Result(success=True, observation_id='test-capture')
            handle.succeed()
            return result

        server = ActionServer(self.node, RunGraspStage, 'grasp/segment_object', segment,
                              callback_group=self.group)
        self.addCleanup(server.destroy)
        tree = self.root / 'target_tree.xml'
        tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'
                        '<SegmentObject observation_id="{obs}" mask="{mask}"/>'
                        '</BehaviorTree></root>')
        for target in ('yellow block', 'blue cup', ''):
            wrapped, _ = self.run_goal(str(tree), target_text=target)
            self.assertEqual(wrapped.status, 4)
        self.assertEqual(received, ['yellow block', 'blue cup', ''])

    def cleanup_runtime(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.server.destroy()
        self.action.destroy()
        self.node.destroy_node()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main()
