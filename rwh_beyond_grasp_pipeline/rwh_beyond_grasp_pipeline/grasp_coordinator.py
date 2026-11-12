"""RGBD/GraspGenX/MoveIt coordinator with a MuJoCo backend. BT owns stage sequencing."""

import uuid
import copy
import json
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from std_srvs.srv import Trigger, SetBool, Empty
from moveit_msgs.msg import (
    PlanningScene,
    PlanningSceneComponents as PC,
    CollisionObject,
    RobotState,
)
from moveit_msgs.srv import (
    GetPlanningScene,
    ApplyPlanningScene,
    GetPositionIK,
    GetCartesianPath,
    GetPositionFK,
    GetMotionPlan,
)
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
from rwh_beyond_msgs.action import InferGrasps, RunGraspStep, RunGraspStage, SegmentImage
from rwh_beyond_msgs.srv import SetGraspRegion
from .red_perception import pose_matrix, set_pair

from .stage_constants import TARGET

from .stage_errors import Canceled

from .perception_stages import PerceptionStages

from .inference_stages import InferenceStages

from .selection_stages import SelectionStages

from .motion_stages import MotionStages
from .scenario_motion import ScenarioMotion
from .gripper_motion import GripperMotion
from .observation_motion import validate_observation_parameters
from control_msgs.action import FollowJointTrajectory
from .visualization import GraspVisualization


class GraspCoordinator(
    GripperMotion,
    ScenarioMotion,
    MotionStages,
    SelectionStages,
    InferenceStages,
    PerceptionStages,
    Node,
):
    def __init__(self):
        super().__init__('grasp_coordinator')
        defaults = {
            'segmentation_backend': 'sam3',
            'target_text': 'red box',
            'target_box': [0.0, 0.0, 0.0, 0.0],
            'selection_point': [-1, -1],
            'selection_policy': 'unique',
            'base_frame': 'base_link',
            'tcp_link': 'tcp_link',
            'pregrasp_distance': 0.08,
            'lift_distance': 0.08,
            'object_dimensions': [0.04, 0.04, 0.05],
            # Model gripper +X is closing; PiPER physical closing is +/-Y.
            # Model fingertip is +Z 0.130 m (official piper_hand config).
            'model_to_tcp_xyz': [0.0, 0.0, 0.13],
            'model_to_tcp_rpy': [0.0, 0.0, -math.pi / 2],
            'home_joint_positions': [0.0] * 6,
            'home_joint_tolerance': 0.01,
            'observation_z_offset': 0.2,
            'observation_tilt_rpy_deg': [0.0, 30.0, 0.0],
            'observation_settle_time': 0.5,
            'gripper_open_width': 0.10,
            'gripper_close_width': 0.04,
            'gripper_close_duration': 4.0,
            'debug_max_candidates': 10,
            'num_grasps': 100,
            'top_k': 100,
            'simulation_only': True,
            'max_candidates': 100,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        for name in ('object_dimensions', 'model_to_tcp_xyz', 'model_to_tcp_rpy'):
            value = np.asarray(self.get_parameter(name).value, dtype=float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(name + ' must have three finite values')
            if name == 'object_dimensions' and np.any(value <= 0):
                raise ValueError('object dimensions must be positive')
        if (
            not 0.02 <= self.get_parameter('pregrasp_distance').value <= 0.2
            or not 0.065 <= self.get_parameter('lift_distance').value <= 0.2
        ):
            raise ValueError('Invalid pregrasp/lift distance')
        validate_observation_parameters(
            *[
                self.get_parameter(name).value
                for name in (
                    'home_joint_positions',
                    'observation_z_offset',
                    'observation_tilt_rpy_deg',
                    'observation_settle_time',
                    'home_joint_tolerance',
                )
            ]
        )
        for name in ('gripper_open_width', 'gripper_close_width'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or not 0 <= value <= 0.1:
                raise ValueError(name + ' must be in [0, 0.1] m')
        duration = self.get_parameter('gripper_close_duration').value
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('gripper_close_duration must be finite and positive')
        self.base = self.get_parameter('base_frame').value
        self.tcp = self.get_parameter('tcp_link').value
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.scene_lock = threading.RLock()
        self.active_handle = None
        self.motion_handle = None
        self.frames = {}
        self.joints = None
        self.cloud_stamp = 0
        self.cloud_seq = 0
        self.prepared = False
        self.attached = False
        self.scene_active = False
        self.original_pairs = {}
        self.cleanup_requested = False
        self.perception = None
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        self.subs = []
        for key, typ, topic in [
            ('rgb', Image, 'camera/color/image_raw'),
            ('depth', Image, 'camera/depth/image_rect_raw'),
            ('info', CameraInfo, 'camera/depth/camera_info'),
        ]:
            self.subs.append(
                self.create_subscription(
                    typ, topic, lambda m, k=key: self.capture(k, m), qos_profile_sensor_data
                )
            )
        self.subs.append(
            self.create_subscription(JointState, 'feedback/joint_states', self.joint_callback, 10)
        )
        self.subs.append(
            self.create_subscription(
                PointCloud2,
                'camera/depth/filtered_points',
                self.cloud_callback,
                qos_profile_sensor_data,
            )
        )
        self.visualization = GraspVisualization(
            self, self.base, self.get_parameter('debug_max_candidates').value
        )
        self.stage_clients = {}
        for key, typ, name in [
            ('scene', GetPlanningScene, 'get_planning_scene'),
            ('apply', ApplyPlanningScene, 'apply_planning_scene'),
            ('ik', GetPositionIK, 'compute_ik'),
            ('cartesian', GetCartesianPath, 'compute_cartesian_path'),
            ('fk', GetPositionFK, 'compute_fk'),
            ('plan', GetMotionPlan, 'plan_kinematic_path'),
            (
                'region',
                SetGraspRegion,
                (
                    'simulation/set_grasp_region'
                    if self.get_parameter('simulation_only').value
                    else 'grasp/set_grasp_region'
                ),
            ),
            ('sam_health', Trigger, 'sam3/health'),
            ('grasp_health', Trigger, 'graspgenx/health'),
            ('clear', Empty, 'clear_octomap'),
        ]:
            self.stage_clients[key] = self.create_client(typ, name, callback_group=self.group)
        if self.get_parameter('simulation_only').value:
            self.stage_clients['exclude'] = self.create_client(
                SetBool, 'simulation/exclude_red_target', callback_group=self.group
            )
            self.stage_clients['physics'] = self.create_client(
                Trigger, 'simulation/grasp_state', callback_group=self.group
            )
        self.segment = ActionClient(self, SegmentImage, 'segment_image', callback_group=self.group)
        self.infer = ActionClient(self, InferGrasps, 'infer_grasps', callback_group=self.group)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.group
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            callback_group=self.group,
        )
        self.active_action = RunGraspStep
        self.observation_id = ""
        self.plan_id = ""
        self.plan_phase = ""
        self.stage_servers = [
            ActionServer(
                self,
                RunGraspStage,
                "grasp/" + command,
                lambda handle, c=command: self.execute_stage(handle, c),
                callback_group=self.group,
                goal_callback=self.goal,
                cancel_callback=self.cancel,
            )
            for command in (
                "check_ready",
                "segment_object",
                "build_object_cloud",
                "infer_grasps",
                "select_feasible_grasp",
                "move_pregrasp",
                "approach",
                "attach",
                "lift",
            )
        ]
        self.server = ActionServer(
            self,
            RunGraspStep,
            'grasp_step',
            self.execute,
            callback_group=self.group,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
        )
        self.cleanup_service = self.create_service(
            Trigger, 'grasp/cleanup', self.cleanup_service_cb, callback_group=self.group
        )

    def capture(self, key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.frames.setdefault(stamp, {})[key] = msg
        for old in sorted(self.frames)[:-8]:
            self.frames.pop(old, None)

    def joint_callback(self, msg):
        self.joints = msg

    def cloud_callback(self, msg):
        # MoveIt publishes a processed frame even when every return belongs to
        # known collision geometry. Empty is not the same as a missing update.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id and msg.height and stamp > 0:
            self.cloud_stamp = stamp
            self.cloud_seq += 1

    def goal(self, goal):
        return GoalResponse.ACCEPT if self.lock.acquire(False) else GoalResponse.REJECT

    def cancel(self, handle):
        if self.motion_handle:
            self.motion_handle.cancel_goal_async()
        self.cleanup_requested = True
        return CancelResponse.ACCEPT

    def stage(self, text):
        self.get_logger().info(text)
        if self.active_handle:
            self.active_handle.publish_feedback(self.active_action.Feedback(stage=text))

    def wait(self, predicate, timeout=15, check_cancel=True):
        end = time.monotonic() + timeout
        while rclpy.ok() and not predicate():
            if check_cancel and self.active_handle and self.active_handle.is_cancel_requested:
                raise Canceled('stage canceled')
            if time.monotonic() > end:
                raise TimeoutError('Timed out waiting for stage dependency/result')
            time.sleep(0.01)
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')
        if check_cancel and self.active_handle and self.active_handle.is_cancel_requested:
            raise Canceled('stage canceled')

    def call(self, key, value, timeout=15, check_cancel=True):
        client = self.stage_clients[key]
        self.wait(client.service_is_ready, timeout, check_cancel)
        future = client.call_async(value)
        self.wait(future.done, timeout, check_cancel)
        return future.result()

    def state(self):
        self.wait(lambda: self.joints is not None, 5)
        return RobotState(joint_state=copy.deepcopy(self.joints), is_diff=True)

    def apply(self, scene, check_cancel=True):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.call(
            'apply', ApplyPlanningScene.Request(scene=scene), check_cancel=check_cancel
        ).success:
            raise RuntimeError('Planning Scene update rejected')

    def physics(self, check_cancel=True):
        response = self.call('physics', Trigger.Request(), check_cancel=check_cancel)
        data = json.loads(response.message)
        if data.get('simulation') != 'mujoco':
            raise RuntimeError('Simulation-only coordinator requires MuJoCo verification service')
        return response, data

    def execute(self, handle):
        self.scene_lock.acquire()
        self.active_handle = handle
        self.active_action = RunGraspStep
        result = RunGraspStep.Result()
        try:
            command = handle.request.command
            self.stage(command)
            if command != 'cleanup':
                self.ensure_backend()
            if command == 'check_ready':
                self.check_ready()
            elif command == 'move_home':
                self.move_home()
            elif command == 'move_observation':
                if handle.request.override_observation:
                    self.move_observation(
                        handle.request.observation_z_offset, handle.request.observation_tilt_deg
                    )
                else:
                    self.move_observation()
            elif command == 'open_gripper':
                self.open_gripper()
            elif command == 'segment_target':
                self.segment_target()
            elif command == 'build_object_cloud':
                self.build_object_cloud()
            elif command == 'prepare':
                result.pregrasp = self.prepare()
            elif command == 'move_pregrasp':
                if not self.prepared:
                    raise RuntimeError('No prepared grasp')
                self.execute_trajectory(self.pre_trajectory)
            elif command == 'approach':
                if not self.prepared:
                    raise RuntimeError('No prepared grasp')
                self.execute_cartesian(self.grasp_pose)
            elif command == 'attach':
                self.attach()
            elif command == 'lift':
                self.lift()
            elif command == 'cleanup':
                self.cleanup()
            else:
                raise ValueError('Unknown grasp command: ' + command)
            if handle.is_cancel_requested:
                raise Canceled('stage canceled')
            result.success = True
            result.message = 'Completed ' + command
            handle.succeed()
        except Canceled as error:
            result.message = str(error)
            handle.canceled()
        except Exception as error:
            result.message = f'{type(error).__name__}: {error}'
            self.get_logger().error(result.message)
            handle.abort()
        finally:
            if self.cleanup_requested or (
                not result.success and handle.request.command != 'check_ready'
            ):
                try:
                    self.cleanup()
                except Exception as error:
                    self.get_logger().error('Scene cleanup failed: ' + str(error))
            self.cleanup_requested = False
            self.active_handle = None
            self.scene_lock.release()
            self.lock.release()
        return result

    def feedback(self):
        self.wait(lambda: self.joints is not None, 5)
        values = dict(zip(self.joints.name, self.joints.position))
        if not all(
            name in values and math.isfinite(values[name])
            for name in [*[f'joint{i}' for i in range(1, 7)], 'gripper']
        ):
            raise RuntimeError('Simulation arm/gripper feedback is missing or nonfinite')
        return values

    def ensure_backend(self):
        if not self.get_parameter('simulation_only').value:
            raise RuntimeError('Use the physical coordinator for hardware execution')
        self.physics()

    def check_ready(self):
        if not self.get_parameter('simulation_only').value:
            raise RuntimeError('Hardware execution is not supported')
        _, physics_state = self.physics()
        attached_query = GetPlanningScene.Request()
        attached_query.components.components = PC.ROBOT_STATE_ATTACHED_OBJECTS
        existing = self.call('scene', attached_query).scene.robot_state.attached_collision_objects
        if TARGET in {o.object.id for o in existing}:
            self.attached = True
            self.scene_active = True
        if physics_state.get('bilateral_contact') and not self.attached:
            raise RuntimeError('Target already in finger contact; reset/release before a new pick')
        if self.attached:
            raise RuntimeError('Already holding a target; place/release before another pick')
        health = self.call('grasp_health', Trigger.Request(), 10)
        if not health.success:
            raise RuntimeError('A real GraspGenX server is required: ' + health.message)
        if self.get_parameter('segmentation_backend').value == 'sam3':
            health = self.call('sam_health', Trigger.Request(), 10)
            if not health.success:
                raise RuntimeError('Real SAM 3 server required: ' + health.message)

    def require_observation(self, request, artifact=None):
        context = self.perception_context()
        if not request.observation_id or request.observation_id != self.observation_id:
            raise ValueError('Observation ID mismatch; reobserve target')
        if (
            artifact
            and getattr(request, artifact)
            != context[{'object_mask': 'mask', 'object_cloud': 'cloud'}[artifact]]
        ):
            raise ValueError('Input ' + artifact + ' does not match the observation')
        return context

    def execute_stage(self, handle, command):
        self.scene_lock.acquire()
        self.active_handle = handle
        self.active_action = RunGraspStage
        result = RunGraspStage.Result()
        try:
            request = handle.request
            self.stage(command)
            self.ensure_backend()
            if command == 'check_ready':
                self.check_ready()
            elif command == 'segment_object':
                self.segment_target(request.target_text)
                result.object_mask = self.perception['mask']
            elif command == 'build_object_cloud':
                self.require_observation(request, 'object_mask')
                self.build_object_cloud()
                result.object_cloud = self.perception['cloud']
            elif command == 'infer_grasps':
                context = self.require_observation(request, 'object_cloud')
                result.candidates = self.infer_grasps(context)['candidates']
            elif command == 'select_feasible_grasp':
                context = self.require_observation(request, 'object_cloud')
                if not request.candidates:
                    raise ValueError('No grasp candidates supplied')
                for candidate in request.candidates:
                    if candidate.pose.header != request.object_cloud.header:
                        raise ValueError('Candidate frame/stamp differs from object cloud')
                    values = [candidate.score, *pose_matrix(candidate.pose.pose).ravel()]
                    if not np.isfinite(values).all():
                        raise ValueError('Nonfinite grasp candidate')
                result.pregrasp = self.select_feasible_grasp(context, request.candidates)
                self.plan_id = uuid.uuid4().hex
                self.plan_phase = "selected"
                result.plan_id = self.plan_id
            else:
                if not request.plan_id or request.plan_id != self.plan_id or not self.prepared:
                    raise ValueError('Plan ID mismatch or no prepared plan')
                expected = {
                    'move_pregrasp': 'selected',
                    'approach': 'pregrasp',
                    'attach': 'approached',
                    'lift': 'attached',
                }
                if self.plan_phase != expected.get(command):
                    raise ValueError('Grasp stage out of order or already executed')
                if command == 'move_pregrasp':
                    self.perception_context()
                    self.execute_trajectory(self.pre_trajectory)
                elif command == 'approach':
                    self.execute_cartesian(self.grasp_pose)
                elif command == 'attach':
                    self.attach()
                elif command == 'lift':
                    self.lift()
                else:
                    raise ValueError('Unknown fixed stage')
                self.plan_phase = {
                    'move_pregrasp': 'pregrasp',
                    'approach': 'approached',
                    'attach': 'attached',
                    'lift': 'lifted',
                }[command]
            if handle.is_cancel_requested:
                raise Canceled('stage canceled')
            result.observation_id = self.observation_id
            result.success = True
            result.message = 'Completed ' + command
            handle.succeed()
        except Canceled as error:
            result.message = str(error)
            handle.canceled()
        except Exception as error:
            result.message = f'{type(error).__name__}: {error}'
            self.get_logger().error(result.message)
            handle.abort()
        finally:
            if self.cleanup_requested or (not result.success and command != 'check_ready'):
                try:
                    self.cleanup()
                except Exception as error:
                    self.get_logger().error('Scene cleanup failed: ' + str(error))
            self.cleanup_requested = False
            self.active_handle = None
            self.scene_lock.release()
            self.lock.release()
        return result

    def prepare(self):
        if self.attached:
            raise RuntimeError('Already holding object')
        # Legacy prepare remains usable; explicit BT stages reuse the same snapshot.
        if self.perception is None:
            self.segment_target()
        context = self.perception_context()
        if 'cloud' not in context:
            self.build_object_cloud()
        candidates = self.infer_grasps(context)['candidates']
        return self.select_feasible_grasp(context, candidates)

    def cleanup_service_cb(self, request, response):
        if not self.scene_lock.acquire(False):
            self.cleanup_requested = True
            response.success = True
            response.message = 'Cleanup deferred to active stage'
            return response
        try:
            self.cleanup()
            response.success = True
            response.message = 'Contact rules restored; planned attachment retained if present'
        except Exception as error:
            response.success = False
            response.message = str(error)
        finally:
            self.scene_lock.release()
        return response

    def cleanup(self):
        if not self.attached:
            self.perception = None
            self.observation_id = ""
            self.plan_id = ""
            self.plan_phase = ""
            self.prepared = False
        if not self.scene_active:
            return
        query = GetPlanningScene.Request()
        query.components.components = (
            PC.ALLOWED_COLLISION_MATRIX | PC.ROBOT_STATE_ATTACHED_OBJECTS | PC.WORLD_OBJECT_NAMES
        )
        current = self.call('scene', query, check_cancel=False).scene
        matrix = current.allowed_collision_matrix
        if TARGET in {o.object.id for o in current.robot_state.attached_collision_objects}:
            self.attached = True
        # Retain the planned payload collision model until explicitly released.
        if not self.attached:
            for finger, allowed in self.original_pairs.items():
                set_pair(matrix, finger, TARGET, allowed)
            scene = PlanningScene(allowed_collision_matrix=matrix)
            if TARGET in {o.id for o in current.world.collision_objects}:
                scene.world.collision_objects = [
                    CollisionObject(id=TARGET, operation=CollisionObject.REMOVE)
                ]
            self.apply(scene, check_cancel=False)
            if self.get_parameter('segmentation_backend').value == 'sam3':
                self.call('region', SetGraspRegion.Request(enabled=False), check_cancel=False)
            else:
                self.call('exclude', SetBool.Request(data=False), check_cancel=False)
            self.scene_active = False
            self.prepared = False
        else:
            self.get_logger().warning(
                'Object remains attached and gripper remains closed; no automatic release'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinator()
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
