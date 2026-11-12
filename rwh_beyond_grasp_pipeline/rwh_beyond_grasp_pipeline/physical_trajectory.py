"""FollowJointTrajectory endpoints driven by measured PiPER feedback, not mock state."""

import math
import threading
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from control_msgs.action import FollowJointTrajectory
from .hardware_coordinator import feedback_values


def trajectory_points(trajectory, current, expected, initial_tolerance=0.03):
    if not math.isfinite(initial_tolerance) or not 0 < initial_tolerance <= 0.05:
        raise ValueError('initial_position_tolerance must be in (0, 0.05] rad')
    names = list(trajectory.joint_names)
    if len(names) != len(expected) or set(names) != set(expected):
        raise ValueError('Trajectory joint names do not match controller')
    if not trajectory.points:
        raise ValueError('Empty trajectory')
    times, points = [0.0], [[current[n] for n in names]]
    previous = -1.0
    prefix_duration = 0.0
    for point in trajectory.points:
        stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        values = list(point.positions)
        if (
            stamp < 0
            or stamp <= previous
            or len(values) != len(names)
            or not all(map(math.isfinite, values))
        ):
            raise ValueError('Expected finite positions and increasing nonnegative times')
        if any((n == 'gripper' and not 0 <= v <= 0.1) for n, v in zip(names, values)):
            raise ValueError('Gripper aperture outside [0, .1] m')
        if stamp == 0:
            discrepancy = max(abs(a - b) for a, b in zip(points[0], values))
            allowed = 0.004 if names == ['gripper'] else initial_tolerance
            if discrepancy > allowed:
                raise ValueError('Initial trajectory point differs from physical feedback')
            if discrepancy > 1e-6:
                # Keep the measured starting command and smoothly join the planned
                # boundary state at <=0.1 rad/s, rather than jumping to it at t=0.
                prefix_duration = max(0.5, discrepancy / 0.1)
                times.append(prefix_duration)
                points.append(values)
        else:
            times.append(stamp + prefix_duration)
            points.append(values)
        previous = stamp
    if previous <= 0:
        raise ValueError('Trajectory must have positive duration')
    return names, np.array(times), np.array(points)


class PhysicalTrajectory(Node):
    def __init__(self):
        super().__init__('piper_physical_trajectory')
        self.declare_parameter('initial_position_tolerance', 0.05)
        value = self.get_parameter('initial_position_tolerance').value
        if not math.isfinite(value) or not 0 < value <= 0.05:
            raise ValueError('initial_position_tolerance must be in (0, 0.05] rad')
        self.declare_parameter('gripper_close_width', 0.04)
        self.declare_parameter('gripper_close_duration', 4.0)
        width = self.get_parameter('gripper_close_width').value
        duration = self.get_parameter('gripper_close_duration').value
        if not math.isfinite(width) or not 0 <= width <= 0.1:
            raise ValueError('gripper_close_width must be in [0, 0.1] m')
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('gripper_close_duration must be finite and positive')
        self.lock = threading.Lock()
        self.joints, self.received_at = None, 0.0
        self.group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            JointState, 'feedback/joint_states', self.feedback_cb, 10
        )
        self.publisher = self.create_publisher(JointState, 'control/joint_states', 10)
        self.gate = self.create_client(SetBool, 'control_enable', callback_group=self.group)
        self.ready_service = self.create_service(
            Trigger, 'grasp/hardware_transport_ready', self.ready
        )
        self.servers = [
            ActionServer(
                self,
                FollowJointTrajectory,
                name + '/follow_joint_trajectory',
                lambda handle, j=joints: self.execute(handle, j),
                callback_group=self.group,
                goal_callback=self.goal,
                cancel_callback=lambda handle: CancelResponse.ACCEPT,
            )
            for name, joints in [
                ('arm_controller', [f'joint{i}' for i in range(1, 7)]),
                ('gripper_controller', ['gripper']),
            ]
        ]

    def ready(self, request, response):
        try:
            self.values()
            response.success = True
            response.message = 'physical_feedback_transport'
        except RuntimeError as error:
            response.success = False
            response.message = str(error)
        return response

    def feedback_cb(self, msg):
        if self.joints is not None and msg.header.stamp == self.joints.header.stamp:
            return
        self.joints, self.received_at = msg, time.monotonic()

    def values(self):
        return feedback_values(self.joints, self.received_at, time.monotonic())

    def goal(self, request):
        return GoalResponse.ACCEPT if self.lock.acquire(False) else GoalResponse.REJECT

    def set_gate(self, enabled):
        if not self.gate.wait_for_service(timeout_sec=3):
            raise RuntimeError('Physical control_enable service unavailable')
        future = self.gate.call_async(SetBool.Request(data=enabled))
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.01)
        if not future.done() or not future.result().success:
            raise RuntimeError('Physical control gate request failed')

    def execute(self, handle, expected):
        result = FollowJointTrajectory.Result()
        opened = False
        try:
            values = self.values()
            trajectory = handle.request.trajectory
            if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
                raise ValueError('Only immediate trajectories (zero header stamp) are supported')
            names, times, points = trajectory_points(
                trajectory, values, expected, self.get_parameter('initial_position_tolerance').value
            )
            timed_close = names == ['gripper'] and math.isclose(
                float(points[-1, 0]),
                self.get_parameter('gripper_close_width').value,
                rel_tol=0.0,
                abs_tol=1e-6,
            )
            close_duration = (
                self.get_parameter('gripper_close_duration').value if timed_close else 0.0
            )
            # Publish a measured starting command before opening the gate.
            self.publisher.publish(JointState(name=names, position=[values[n] for n in names]))
            opened = True
            self.set_gate(True)
            start = time.monotonic()
            deadline = start + float(times[-1]) + 30
            stable_since = None
            while rclpy.ok():
                if handle.is_cancel_requested:
                    result.error_code = result.SUCCESSFUL
                    self.publisher.publish(
                        JointState(name=names, position=[self.values()[n] for n in names])
                    )
                    self.set_gate(False)
                    opened = False
                    handle.canceled()
                    return result
                values = self.values()
                elapsed = time.monotonic() - start
                # Closing commands the requested width for a fixed interval. Its
                # completion means elapsed command time, not contact or aperture.
                command = (
                    [float(points[-1, 0])]
                    if timed_close
                    else [float(np.interp(elapsed, times, points[:, i])) for i in range(len(names))]
                )
                self.publisher.publish(JointState(name=names, position=command))
                if timed_close:
                    if elapsed >= close_duration:
                        result.error_code = result.SUCCESSFUL
                        self.set_gate(False)
                        opened = False
                        handle.succeed()
                        return result
                    time.sleep(0.02)
                    continue
                reached = all(
                    abs(values[n] - v) < (0.004 if n == 'gripper' else 0.02)
                    for n, v in zip(names, points[-1])
                )
                if elapsed >= times[-1] and reached:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= 0.5:
                        result.error_code = result.SUCCESSFUL
                        self.set_gate(False)
                        opened = False
                        handle.succeed()
                        return result
                else:
                    stable_since = None
                if time.monotonic() > deadline:
                    raise RuntimeError('Physical feedback did not reach trajectory endpoint')
                time.sleep(0.02)
            raise RuntimeError('ROS shutdown during physical execution')
        except Exception as error:
            result.error_code = result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(error)
            self.get_logger().error(result.error_string)
            if opened:
                try:
                    current = self.values()
                    self.publisher.publish(
                        JointState(name=expected, position=[current[n] for n in expected])
                    )
                except Exception as stop_error:
                    self.get_logger().error("Physical hold unavailable: " + str(stop_error))
                try:
                    self.set_gate(False)
                    opened = False
                except Exception as stop_error:
                    self.get_logger().error("Physical gate close failed: " + str(stop_error))
            handle.abort()
            return result
        finally:
            if opened:
                try:
                    self.set_gate(False)
                except Exception as error:
                    self.get_logger().error('Failed to close physical control gate: ' + str(error))
            self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalTrajectory()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
