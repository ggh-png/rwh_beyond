"""Physical PiPER backend: real feedback, no MuJoCo verification services."""

import math
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from .planning_state import arm_position_limits, bounded_planning_state
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from moveit_msgs.srv import GetPlanningScene
from .stage_constants import TARGET
from moveit_msgs.msg import PlanningSceneComponents as PC
from std_srvs.srv import Trigger

from .grasp_coordinator import GraspCoordinator


def feedback_values(joints, received_at, now, max_age=1.0):
    if joints is None or now - received_at > max_age:
        raise RuntimeError('Physical joint feedback is missing or stale')
    values = dict(zip(joints.name, joints.position))
    if not all(
        name in values and math.isfinite(values[name])
        for name in [*[f'joint{i}' for i in range(1, 7)], 'gripper']
    ):
        raise RuntimeError('Physical arm/gripper positions are missing or nonfinite')
    return values


class HardwareGraspCoordinator(GraspCoordinator):
    def __init__(self):
        self.joint_received_at = 0.0
        super().__init__()
        for name, value in {
            'hardware_feedback_timeout': 1.0,
            'physical_position_tolerance': 0.03,
            'planning_start_bounds_tolerance_deg': 10.0,
        }.items():
            self.declare_parameter(name, value)
            number = self.get_parameter(name).value
            if not math.isfinite(number) or number <= 0:
                raise ValueError(name + ' must be finite and positive')
        if self.get_parameter('planning_start_bounds_tolerance_deg').value > 180.0:
            raise ValueError('planning_start_bounds_tolerance_deg must be in (0, 180] degrees')
        self.planning_limits = None
        self.stage_clients['robot_description'] = self.create_client(
            GetParameters, 'move_group/get_parameters', callback_group=self.group
        )
        self.stage_clients['hardware_transport'] = self.create_client(
            Trigger, 'grasp/hardware_transport_ready', callback_group=self.group
        )
        self.get_logger().info(
            'Physical grasp backend: grasp detection disabled; attachments are planning assumptions'
        )

    def joint_callback(self, msg):
        if self.joints is not None and msg.header.stamp == self.joints.header.stamp:
            return
        self.joints = msg
        self.joint_received_at = time.monotonic()

    def feedback(self):
        return feedback_values(
            self.joints,
            self.joint_received_at,
            time.monotonic(),
            self.get_parameter('hardware_feedback_timeout').value,
        )

    def ensure_backend(self):
        if self.get_parameter('simulation_only').value:
            raise RuntimeError('Physical backend requires simulation_only=false')
        self.feedback()
        ready = self.call('hardware_transport', Trigger.Request(), 3)
        if not ready.success or ready.message != 'physical_feedback_transport':
            raise RuntimeError('Physical feedback transport not ready: ' + ready.message)

    def physics(self, check_cancel=True):
        raise RuntimeError('MuJoCo physics must not be used by the physical backend')

    def check_ready(self):
        self.ensure_backend()
        if self.get_parameter('segmentation_backend').value != 'sam3':
            raise RuntimeError('Physical BT requires SAM segmentation')
        query = GetPlanningScene.Request()
        query.components.components = PC.ROBOT_STATE_ATTACHED_OBJECTS
        if self.attached or TARGET in {
            o.object.id
            for o in self.call('scene', query).scene.robot_state.attached_collision_objects
        }:
            self.attached = True
            raise RuntimeError(
                'Target remains attached; place/release and clear attachment before another pick'
            )
        for key in ('grasp_health', 'sam_health'):
            result = self.call(key, Trigger.Request(), 10)
            if not result.success:
                raise RuntimeError(key + ': ' + result.message)
        self.wait(
            lambda: all(
                self.stage_clients[key].service_is_ready()
                for key in ('ik', 'cartesian', 'plan', 'region', 'clear')
            ),
            15,
        )
        # Require a fresh synchronized physical RGB-D frame and its capture-time TF.
        frame = self.snapshot()
        self.buffer.lookup_transform(
            self.base,
            frame['depth'].header.frame_id,
            Time.from_msg(frame['depth'].header.stamp),
            timeout=Duration(seconds=2),
        )

    def state(self):
        self.feedback()
        if self.planning_limits is None:
            response = self.call(
                'robot_description', GetParameters.Request(names=['robot_description'])
            )
            if len(response.values) != 1 or not response.values[0].string_value:
                raise RuntimeError(
                    'MoveIt robot_description is unavailable for joint bounds checking'
                )
            self.planning_limits = arm_position_limits(response.values[0].string_value)
            self.feedback()  # Model lookup may have waited; require fresh feedback again.
        state, corrections = bounded_planning_state(
            super().state(),
            self.planning_limits,
            math.radians(self.get_parameter('planning_start_bounds_tolerance_deg').value),
        )
        if corrections:
            self.get_logger().warning(
                'Planning-only start bounds correction (physical feedback unchanged): '
                + '; '.join(corrections),
                throttle_duration_sec=5.0,
            )
        return state

    def execute_trajectory(self, trajectory):
        self.ensure_backend()
        # Payload remains attached for collision planning, without judging grip.
        super().execute_trajectory(trajectory)
        names = trajectory.joint_trajectory.joint_names
        endpoint = dict(zip(names, trajectory.joint_trajectory.points[-1].positions))

        def reached():
            values = self.feedback()
            return all(
                abs(values[name] - value) <= self.get_parameter('physical_position_tolerance').value
                for name, value in endpoint.items()
                if name != 'gripper'
            )

        self.wait(reached, 20)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareGraspCoordinator()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
