"""Exercise the actual navigator with a mock controller; no hardware/simulation."""
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
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, JointState
from std_srvs.srv import Empty
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import qos_profile_sensor_data
from rwh_beyond_msgs.action import RunDemo, SegmentImage
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
                        '<OpenGripper name="open" opening="" duration=""/>'
                        '</BehaviorTree></root>')
        place_tree = self.root / 'place_tree.xml'
        place_tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'
                              '<AlwaysSuccess name=""/>'
                              '</BehaviorTree></root>')
        config = self.root / 'params.yaml'
        config.write_text(yaml.safe_dump({'bt_navigator': {'ros__parameters': {
            'use_sim_time': False, 'global_frame': 'base_link', 'robot_base_frame': 'base_link',
            'navigators': ['demo'], 'demo.plugin': 'rwh_beyond_navigator::DemoNavigator',
            'plugin_lib_names': ['rwh_beyond_bt_nodes'], 'default_demo_bt_xml': str(tree),
            'default_place_bt_xml': str(place_tree),
            'bt_perception.target_text': 'default object',
            'bt_motion.gripper_open_width': .08,
            'wait_for_service_timeout': 3000, 'default_server_timeout': 1000,
            'error_code_names': ['nav2_unused_error_code'],
        }}}))
        self.success = False
        self.executions = 0
        self.message = 'Trajectory invalid after scene update'
        self.reject = False
        self.node = rclpy.create_node('navigator_error_mock')
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(self.node, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory', self.execute,
                                   goal_callback=self.accept, callback_group=self.group)
        self.action = ActionClient(self.node, RunDemo, 'run_demo', callback_group=self.group)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.joint_pub = self.node.create_publisher(JointState, 'feedback/joint_states', 10)
        def publish_joints():
            message = JointState(name=[f'joint{i}' for i in range(1,7)]+['gripper'], position=[0.]*6+[.08])
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.joint_pub.publish(message)
        self.timer = self.node.create_timer(.02, publish_joints, callback_group=self.group)
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
        self.executions += 1
        self.assertAlmostEqual(handle.request.trajectory.points[0].positions[0], .08)
        time.sleep(0.05)
        result = FollowJointTrajectory.Result(error_code=0 if self.success else -1, error_string=self.message)
        if self.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def wait(self, predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(predicate(), 'Timed out: ' + str(self.root))

    def run_goal(self, tree="", target_text="", mode=0):
        feedback = []
        sent = self.action.send_goal_async(
            RunDemo.Goal(behavior_tree=tree, target_text=target_text, mode=mode),
            feedback_callback=lambda msg: feedback.append(msg.feedback),
        )
        self.wait(sent.done)
        self.assertTrue(sent.result().accepted)
        completed = sent.result().get_result_async()
        self.wait(completed.done)
        return completed.result(), feedback

    def test_mode_selects_pick_or_place_and_legacy_zero_is_pick(self):
        legacy, _ = self.run_goal()
        self.assertEqual(legacy.status, 6)
        pick, _ = self.run_goal(mode=RunDemo.Goal.MODE_PICK)
        self.assertEqual(pick.status, 6)
        place, _ = self.run_goal(mode=RunDemo.Goal.MODE_PLACE)
        self.assertEqual(place.status, 4)

    def test_direct_controller_failure_feedback_result_and_success_reset(self):
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 6)  # ROS action ABORTED
        result = wrapped.result
        self.assertEqual(result.error_code, -1201)
        self.assertEqual(result.error_name, 'BT_MOTION_FAILED')
        self.assertEqual(result.error_source, 'OpenGripper')
        self.assertEqual(result.error_msg, 'Action execution failed')
        self.assertEqual(result.failed_stage, 'OpenGripper')
        self.assertEqual(result.failed_node, 'open')
        self.assertEqual(result.moveit_state, '')
        self.assertTrue(any(f.error_code == -1201 and f.error_msg == 'Action execution failed' for f in feedback))
        self.success = True
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 4)  # ROS action SUCCEEDED
        self.assertEqual(wrapped.result.error_code, 0)
        self.assertEqual(wrapped.result.failed_stage, '')
        self.assertEqual(wrapped.result.error_source, '')
        self.assertTrue(all(f.error_code == 0 for f in feedback))
        self.reject = True
        wrapped, feedback = self.run_goal()
        self.assertEqual(wrapped.status, 6)
        self.assertEqual(wrapped.result.error_name, 'BT_MOTION_FAILED')
        self.assertNotEqual(wrapped.result.error_code, 0)

    def test_groot_custom_include_and_same_path_resave(self):
        self.success = True
        child = self.root / 'child.xml'
        child.write_text('<root BTCPP_format="4"><BehaviorTree ID="Child">'
                         '<OpenGripper opening="" duration=""/></BehaviorTree></root>')
        tree = self.root / 'groot.xml'
        original = ('<root BTCPP_format="4" main_tree_to_execute="Main">'
                    '<include path="child.xml"/>'
                    '<BehaviorTree ID="Main"><SubTree ID="Child" _autoremap="true"/></BehaviorTree></root>')
        tree.write_text(original)
        wrapped, _ = self.run_goal(str(tree))
        self.assertEqual(wrapped.status, 4)
        self.assertEqual(tree.read_text(), original)
        self.assertIn('opening=""', child.read_text())
        # Re-save only the include at the same source path, while Nav2's
        # always_reload_bt_xml remains false. The next goal must see the edit.
        child.write_text('<root BTCPP_format="4"><BehaviorTree ID="Child">'
                         '<AlwaysFailure/></BehaviorTree></root>')
        wrapped, _ = self.run_goal(str(tree))
        self.assertEqual(wrapped.status, 6)
        self.assertFalse(tree.with_suffix('.xml.bak').exists())
        self.assertEqual(self.executions, 1)
        child.write_text('<root>')
        wrapped, _ = self.run_goal(str(tree))
        self.assertEqual(wrapped.status, 6)
        self.assertEqual(self.executions, 1)
        self.assertEqual(child.read_text(), '<root>')

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
            result = SegmentImage.Result(success=True)
            result.object_mask = Image(header=handle.request.rgb_image.header, width=12, height=12,
                                       encoding="mono8", step=12, data=bytes([1])*144)
            handle.succeed()
            return result

        server = ActionServer(self.node, SegmentImage, 'segment_image', segment,
                              callback_group=self.group)
        self.addCleanup(server.destroy)
        tree = self.root / 'target_tree.xml'
        tree.write_text('<root BTCPP_format="4"><BehaviorTree ID="Test">'
                        '<Sequence><CaptureRgbd capture="{capture}" observation_id="{obs}"/>'
                        '<SegmentObject capture="{capture}" mask="{mask}"/></Sequence>'
                        '</BehaviorTree></root>')
        rgb = self.node.create_publisher(Image, 'camera/color/image_raw', qos_profile_sensor_data)
        depth = self.node.create_publisher(Image, 'camera/depth/image_rect_raw', qos_profile_sensor_data)
        info = self.node.create_publisher(CameraInfo, 'camera/depth/camera_info', qos_profile_sensor_data)
        joints = self.node.create_publisher(JointState, 'feedback/joint_states', qos_profile_sensor_data)
        def publish_capture():
            from std_msgs.msg import Header
            header = Header(frame_id='camera', stamp=self.node.get_clock().now().to_msg())
            rgb.publish(Image(header=header, width=12, height=12, encoding='rgb8', step=36, data=bytes(432)))
            depth.publish(Image(header=header, width=12, height=12, encoding='16UC1', step=24, data=bytes(288)))
            info.publish(CameraInfo(header=header, width=12, height=12))
            joints.publish(JointState(header=header, name=[f'joint{i}' for i in range(1,7)], position=[0.]*6))
        timer = self.node.create_timer(.02, publish_capture, callback_group=self.group)
        self.addCleanup(lambda:self.node.destroy_timer(timer))
        for target in ('yellow block', 'blue cup', ''):
            wrapped, _ = self.run_goal(str(tree), target_text=target)
            self.assertEqual(wrapped.status, 4)
        self.assertEqual(received, ['yellow block', 'blue cup', 'default object'])

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
