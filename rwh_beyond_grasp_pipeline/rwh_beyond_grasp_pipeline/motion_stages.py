"""Shared MoveIt IK, Cartesian planning and trajectory execution."""

from .motion_parameters import motion_value
from .grasp_motion_policy import trajectory_within_wrist_limits
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


class MotionStages:
    def ik(self, pose, state):
        request = GetPositionIK.Request()
        r = request.ik_request
        r.group_name = 'arm'
        r.ik_link_name = self.tcp
        r.robot_state = state
        r.avoid_collisions = True
        r.pose_stamped = PoseStamped(header=Header(frame_id=self.base), pose=pose)
        r.timeout = Duration(seconds=motion_value(self, 'moveit_ik_timeout')).to_msg()
        result = self.call('ik', request, max(5, motion_value(self, 'moveit_ik_timeout') + 2))
        return result.solution if result.error_code.val == 1 else None

    def cartesian(self, pose, state):
        request = GetCartesianPath.Request(
            header=Header(frame_id=self.base),
            start_state=state,
            group_name='arm',
            link_name=self.tcp,
            waypoints=[pose],
            max_step=0.004,
            avoid_collisions=True,
            revolute_jump_threshold=0.35,
            max_velocity_scaling_factor=motion_value(self, 'moveit_cartesian_velocity_scaling_factor'),
            max_acceleration_scaling_factor=motion_value(self, 'moveit_cartesian_acceleration_scaling_factor'),
        )
        return self.call('cartesian', request, 15)

    def execute_cartesian(self, pose, wrist_limits=None):
        path = self.cartesian(pose, self.state())
        if path.error_code.val != 1 or path.fraction < 0.999:
            raise RuntimeError(
                f'Cartesian path rejected: fraction={path.fraction}, code={path.error_code.val}'
            )
        if wrist_limits is not None and not trajectory_within_wrist_limits(path.solution, wrist_limits):
            raise RuntimeError('Cartesian path exceeds selected wrist rotation limits')
        self.execute_trajectory(path.solution)

    def execute_trajectory(self, trajectory):
        self.ensure_backend()  # Check backend-specific readiness immediately before motion.
        self.wait(self.execute_client.server_is_ready, 10)
        sent = self.execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        self.wait(sent.done, 10)
        self.motion_handle = sent.result()
        if not self.motion_handle.accepted:
            raise RuntimeError('ExecuteTrajectory rejected')
        future = self.motion_handle.get_result_async()
        try:
            self.wait(future.done, 90)
        except Exception:
            self.motion_handle.cancel_goal_async()
            raise
        finally:
            self.motion_handle = None
        result = future.result()
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError('MoveIt execution failed: ' + str(result.result.error_code))
