"""Shared sim/real home, observation, gripper and pick scenario motions."""

import copy
import math
import time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneWorld, Constraints, JointConstraint,
)
from moveit_msgs.msg import PlanningSceneComponents as PC
from moveit_msgs.srv import GetPlanningScene, GetMotionPlan, GetPositionFK
from rclpy.time import Time
from std_msgs.msg import Header
from .observation_motion import observation_pose, validate_observation_parameters
from .stage_constants import TARGET, LEGACY_SUPPORT_TABLE, FINGERS


class ScenarioMotion:
    def prepare_observation_motion(self):
        self.ensure_backend()
        query = GetPlanningScene.Request()
        query.components.components = PC.ROBOT_STATE_ATTACHED_OBJECTS | PC.WORLD_OBJECT_NAMES
        scene = self.call('scene', query).scene
        attached_ids = {
            item.object.id for item in scene.robot_state.attached_collision_objects
        }
        # MoveIt rejects REMOVE for an absent object. Clean up only a retained table.
        if any(obj.id == LEGACY_SUPPORT_TABLE for obj in scene.world.collision_objects):
            self.apply(PlanningScene(world=PlanningSceneWorld(collision_objects=[
                CollisionObject(id=LEGACY_SUPPORT_TABLE, operation=CollisionObject.REMOVE),
            ])))
        if attached_ids - {TARGET}:
            raise RuntimeError('Unknown attached objects prevent home/observation motion')
        if self.attached and TARGET not in attached_ids:
            raise RuntimeError('Planned target attachment is missing from the collision scene')
        if TARGET in attached_ids:
            self.attached = True
            # Keep attached payload geometry, target exclusion and closed gripper.
            self.stage('carried_target_retained_for_collision_checked_motion')
            return True
        self.cleanup()
        return False

    def move_home(self, home=None):
        home = self.get_parameter('home_joint_positions').value if home is None else home
        if len(home) != 6 or not all(math.isfinite(v) for v in home):
            raise ValueError('home_joint_positions requires six finite radians')
        self.prepare_observation_motion()
        tolerance = self.get_parameter('home_joint_tolerance').value

        def at_home():
            values = self.feedback()
            return all(
                abs(values[f'joint{i+1}'] - value) <= tolerance for i, value in enumerate(home)
            )

        # MoveIt can return a single t=0 point when the goal is already reached.
        # Confirm with fresh joint feedback instead of sending a zero-time goal.
        if at_home():
            self.stage('home_already_reached_skip_motion')
            return
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = 'arm'
        plan.pipeline_id = 'ompl'
        plan.planner_id = 'RRTConnectkConfigDefault'
        plan.allowed_planning_time = 10.0
        plan.max_velocity_scaling_factor = 0.1
        plan.max_acceleration_scaling_factor = 0.1
        plan.start_state = self.state()
        plan.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=f'joint{i+1}',
                        position=float(value),
                        tolerance_above=0.001,
                        tolerance_below=0.001,
                        weight=1.0,
                    )
                    for i, value in enumerate(home)
                ]
            )
        ]
        response = self.call('plan', request, 15).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'Home planning failed: {response.error_code.val}')
        points = response.trajectory.joint_trajectory.points
        if not points or (
            points[-1].time_from_start.sec == 0 and points[-1].time_from_start.nanosec == 0
        ):
            if at_home():
                self.stage('home_already_reached_skip_motion')
                return
            raise RuntimeError('Home plan has no positive duration but joints are not at home')
        self.execute_trajectory(response.trajectory)
        self.wait(at_home, 20)

    def move_observation(self, rise_m=None, tilt_deg=None, home=None, settle_time=None,
                         x=None, y=None, z=None):
        rise = self.get_parameter('observation_z_offset').value if rise_m is None else rise_m
        x_offset = 0.0 if x is None else x
        y_offset = 0.0 if y is None else y
        rise = rise if z is None else z
        tilt = list(self.get_parameter('observation_tilt_rpy_deg').value)
        if tilt_deg is not None:
            tilt[1] = tilt_deg
        home = self.get_parameter('home_joint_positions').value if home is None else home
        settle = self.get_parameter('observation_settle_time').value if settle_time is None else settle_time
        validate_observation_parameters(
            home,
            rise,
            tilt,
            settle,
            self.get_parameter('home_joint_tolerance').value,
            offset_xyz=[x_offset, y_offset, rise] if any(v is not None for v in (x, y, z)) else None,
        )
        carrying = self.prepare_observation_motion()
        tolerance = self.get_parameter('home_joint_tolerance').value
        values = self.feedback()
        if not carrying and any(
            abs(values[f'joint{i+1}'] - value) > tolerance for i, value in enumerate(home)
        ):
            raise RuntimeError('Move home before requesting the observation motion')
        # FK of the configured home, rather than a drifting measured starting pose.
        state = self.state()
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        positions.update({f'joint{i+1}': value for i, value in enumerate(home)})
        state.joint_state.position = [positions[name] for name in state.joint_state.name]
        request = GetPositionFK.Request(
            header=Header(frame_id=self.base), fk_link_names=[self.tcp], robot_state=state
        )
        response = self.call('fk', request)
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError('Cannot compute the configured home TCP pose')
        target = observation_pose(response.pose_stamped[0].pose, rise, tilt, x_offset, y_offset)
        self.stage(f'observation_target xyz_m={[x_offset, y_offset, rise]} tilt_rpy_deg={tilt}')
        # One Cartesian request interpolates translation and orientation together.
        # execute_cartesian rejects incomplete or colliding paths before execution.
        self.execute_cartesian(target)
        until = time.monotonic() + settle
        self.wait(
            lambda: time.monotonic() >= until,
            settle + 1.0,
        )
        self.stage('observation_pose_reached_ready_for_fresh_rgbd')

    def open_gripper(self, opening=None, duration=None):
        if self.attached:
            raise RuntimeError('Refusing automatic release of attached target')
        opening = self.get_parameter('gripper_open_width').value if opening is None else opening
        if not math.isfinite(opening) or not 0 <= opening <= .10:
            raise ValueError('gripper opening must be finite in [0, 0.10] metres')
        self.command_gripper(opening, duration=duration, closing=False)
        self.wait(lambda: abs(self.feedback()['gripper'] - opening) < 0.005, 10)

    def attach(self):
        if not self.prepared:
            raise RuntimeError('No prepared target')
        self.command_gripper(self.get_parameter('gripper_close_width').value)
        attached = AttachedCollisionObject(
            link_name=self.tcp,
            object=copy.deepcopy(self.object),
            touch_links=list(FINGERS) + [self.tcp],
        )
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        self.apply(scene)
        self.attached = True
        self.stage('gripper_close_completed_planning_attachment_unverified')

    def lift(self):
        if not self.attached:
            raise RuntimeError('No planned target attachment for lift')
        transform = self.buffer.lookup_transform(self.base, self.tcp, Time()).transform
        pose = Pose()
        pose.position.x, pose.position.y = transform.translation.x, transform.translation.y
        pose.position.z = transform.translation.z + self.get_parameter('lift_distance').value
        pose.orientation = transform.rotation
        self.execute_cartesian(pose)
        self.stage('lift_motion_completed_grasp_not_checked')
