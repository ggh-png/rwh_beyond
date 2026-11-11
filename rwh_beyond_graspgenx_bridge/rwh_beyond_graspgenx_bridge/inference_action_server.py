import json
import copy
import threading
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from rwh_beyond_msgs.action import InferGrasps, SegmentImage
from rwh_beyond_msgs.msg import GraspCandidate
from rwh_beyond_model_utils.cloud_converter import cloud_xyz
from rwh_beyond_model_utils.depth_converter import depth_xyz
from rwh_beyond_sam3_bridge.action_client import segment_via_action
from rwh_beyond_model_utils.pose_converter import validate_candidates, quaternion_xyzw
from rwh_beyond_model_utils.zmq_client import request, RequestCanceled, ServerError, ProtocolError

class InferenceActionServer(Node):
    def __init__(self):
        super().__init__('graspgenx_bridge')
        for name, value in [('endpoint', 'tcp://127.0.0.1:5566'), ('timeout', 60.0),
                            ('allow_mock_segmentation', False), ('min_points', 100), ('max_points', 200000), ('max_grasps', 1000),
                            ('max_depth_pixels', 2073600), ('sensor_sync_tolerance', 0.1)]:
            self.declare_parameter(name, value)
        self.endpoint = self.get_parameter('endpoint').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.minimum = self.get_parameter('min_points').value
        self.maximum = self.get_parameter('max_points').value
        self.max_grasps = self.get_parameter('max_grasps').value
        if not 0 < self.timeout <= 3600 or not 1 <= self.minimum <= self.maximum or self.max_grasps < 1:
            raise ValueError('invalid bridge timeout/point/grasp limits')
        self.max_pixels = self.get_parameter('max_depth_pixels').value
        self.sync_tolerance = float(self.get_parameter('sensor_sync_tolerance').value)
        if self.max_pixels < 1 or not 0 <= self.sync_tolerance <= 10:
            raise ValueError('invalid image size/synchronization parameters')
        self.segment = ActionClient(self, SegmentImage, 'segment_image',
                                    callback_group=ReentrantCallbackGroup())
        self.health_service = self.create_service(
            Trigger, 'graspgenx/health', self.health, callback_group=ReentrantCallbackGroup())
        self.lock = threading.Lock()
        self.server = ActionServer(self, InferGrasps, 'infer_grasps', self.execute,
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT)

    def health(self, _, response):
        try:
            data = request(self.endpoint, {'action': 'health'}, 3.0)
            response.success = data.get('status') == 'ok' and data.get('mock') is not True
            response.message = json.dumps(data)
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def goal(self, goal):
        if not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        result = InferGrasps.Result()
        def feedback(stage):
            handle.publish_feedback(InferGrasps.Feedback(stage=stage))
        try:
            feedback('validating_input')
            goal = handle.request
            if not goal.gripper_name.strip() or not 1 <= goal.num_grasps <= self.max_grasps or not 1 <= goal.top_k <= goal.num_grasps:
                raise ValueError('gripper_name required; 1 <= top_k <= num_grasps <= max_grasps')
            segmentation_mock = False
            if goal.input_mode != InferGrasps.Goal.RGBD and goal.rgb_image.data:
                raise ValueError('rgb_image requires RGBD mode')
            if goal.input_mode == InferGrasps.Goal.RGBD:
                if goal.object_cloud.data or goal.object_mask.data:
                    raise ValueError('RGBD does not accept presegmented cloud/mask')
                feedback('segmenting_object')
                box = list(goal.target_box) if any(goal.target_box) else []
                point = [goal.selection_u, goal.selection_v] if goal.use_selection_point else []
                mask, score, count, segmentation_mock = segment_via_action(
                    self.segment, goal.rgb_image, goal.depth_image,
                    goal.target_text, box, point, goal.selection_policy or 'unique', self.timeout,
                    lambda: handle.is_cancel_requested or not rclpy.ok(),
                    self.get_parameter('allow_mock_segmentation').value)
                result.object_mask = mask
                result.segmentation_score = score
                result.segmentation_instances = count
                feedback('converting_depth')
                xyz = depth_xyz(goal.depth_image, goal.camera_info, mask,
                    self.minimum, self.maximum, self.max_pixels, self.sync_tolerance)
                header = copy.deepcopy(goal.depth_image.header)
            elif goal.input_mode == InferGrasps.Goal.POINT_CLOUD:
                if goal.depth_image.data or goal.object_mask.data:
                    raise ValueError('POINT_CLOUD mode does not accept depth/mask; choose one source')
                header = copy.deepcopy(goal.object_cloud.header)
                xyz = cloud_xyz(goal.object_cloud, self.minimum, self.maximum)
            elif goal.input_mode == InferGrasps.Goal.DEPTH_IMAGE:
                if goal.object_cloud.data:
                    raise ValueError('DEPTH_IMAGE mode does not accept object_cloud; choose one source')
                feedback('converting_depth')
                xyz = depth_xyz(goal.depth_image, goal.camera_info, goal.object_mask,
                    self.minimum, self.maximum, self.max_pixels, self.sync_tolerance)
                header = copy.deepcopy(goal.depth_image.header)
            else:
                raise ValueError('unknown input_mode')
            feedback('waiting_for_inference')
            response = request(self.endpoint, dict(action='infer', point_cloud=xyz,
                gripper_name=goal.gripper_name, num_grasps=goal.num_grasps,
                grasp_threshold=-1.0, topk_num_grasps=goal.top_k), self.timeout,
                lambda: handle.is_cancel_requested or not rclpy.ok())
            feedback('converting_candidates')
            try:
                if response.get('gripper_name') != goal.gripper_name:
                    raise ValueError('response gripper_name differs from requested gripper')
                matrices, scores = validate_candidates(response, goal.top_k)
            except ValueError as error:
                raise ProtocolError(str(error)) from error
            for index, (matrix, score) in enumerate(zip(matrices, scores)):
                candidate = GraspCandidate(id=str(index), score=float(score))
                candidate.pose.header = copy.deepcopy(header)
                p = candidate.pose.pose.position
                p.x, p.y, p.z = map(float, matrix[:3, 3])
                q = candidate.pose.pose.orientation
                q.x, q.y, q.z, q.w = map(float, quaternion_xyzw(matrix[:3,:3]))
                result.candidates.append(candidate)
            if handle.is_cancel_requested:
                raise RequestCanceled('canceled before result publication')
            if not result.candidates:
                result.message = 'NO_CANDIDATES: inference completed with no grasps'
                handle.abort()
            else:
                result.success = True
                prefix = 'MOCK_ONLY' if response.get('mock') is True or segmentation_mock else 'OK'
                result.message = f'{prefix}: {len(result.candidates)} candidates in input frame'
                handle.succeed()
        except RequestCanceled as error:
            result.candidates = []
            result.message = 'CANCELED: ' + str(error)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception as error:
            prefix = ('TIMEOUT' if isinstance(error, TimeoutError) else
                      'SERVER_ERROR' if isinstance(error, ServerError) else
                      'INVALID_RESPONSE' if isinstance(error, ProtocolError) else
                      'INVALID_INPUT' if isinstance(error, (ValueError, AssertionError)) else 'INTERNAL_ERROR')
            result.message = f'{prefix}: {error}'
            result.candidates = []
            self.get_logger().error(result.message)
            handle.abort()
        finally:
            self.lock.release()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = InferenceActionServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
