"""Shared trajectory action transport for simulated and physical grippers."""

import math

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperMotion:
    def command_gripper(self, opening):
        self.ensure_backend()
        self.wait(self.gripper_client.server_is_ready, 10)
        simulation = self.get_parameter('simulation_only').value
        backend = 'Simulation' if simulation else 'Physical'
        closing = math.isclose(
            opening, self.get_parameter('gripper_close_width').value, rel_tol=0.0, abs_tol=1e-6
        )
        duration = self.get_parameter('gripper_close_duration').value if closing else 3.0
        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=['gripper'],
                points=[
                    JointTrajectoryPoint(
                        positions=[opening], time_from_start=Duration(seconds=duration).to_msg()
                    )
                ],
            )
        )
        if simulation and closing:
            # MuJoCo may contact the object before reaching the requested width.
            # Physical transport implements its own timed-close completion policy.
            goal.path_tolerance = [JointTolerance(name='gripper', position=-1.0, velocity=-1.0)]
        sent = self.gripper_client.send_goal_async(goal)
        self.wait(sent.done, 10)
        self.motion_handle = sent.result()
        if not self.motion_handle.accepted:
            self.motion_handle = None
            raise RuntimeError(backend + ' gripper goal rejected')
        future = self.motion_handle.get_result_async()
        try:
            self.wait(future.done, 45)
            result = future.result()
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError(backend + ' gripper failed: ' + result.result.error_string)
        except Exception:
            self.motion_handle.cancel_goal_async()
            raise
        finally:
            self.motion_handle = None
