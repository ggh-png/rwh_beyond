from rwh_beyond_model_utils.errors import ActionFailure, set_failure
from rwh_beyond_msgs.msg import ErrorCode
"""ROS segmentation server; the SAM GPU model stays in its own container."""
import json
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from rwh_beyond_msgs.action import SegmentImage
from rwh_beyond_model_utils.zmq_client import (
    ProtocolError, RequestCanceled, ServerError, request,
)
from rwh_beyond_model_utils.file_exchange import Exchange, frame_header
from .segmentation_client import segment_image


class SegmentationActionServer(Node):
    def __init__(self):
        super().__init__('sam3_bridge')
        self.declare_parameter('endpoint', 'tcp://127.0.0.1:5567')
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('allow_mock', False)
        self.exchange = Exchange(self)
        self.lock = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, SegmentImage, 'segment_image', self.execute,
            callback_group=self.group, goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.health_service = self.create_service(
            Trigger, 'sam3/health', self.health, callback_group=self.group)

    def goal(self, _):
        return GoalResponse.ACCEPT if self.lock.acquire(False) else GoalResponse.REJECT

    def health(self, _, response):
        try:
            data = request(self.get_parameter('endpoint').value, {'action': 'health'}, 3.0)
            response.success = (data.get('status') == 'ok' and
                                data.get('backend') == 'sam3' and data.get('mock') is False)
            response.message = json.dumps(data)
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def execute(self, handle):
        result = SegmentImage.Result()
        try:
            goal = handle.request
            rpc = request
            stage = 'segmenting_on_sam3'
            if goal.exchange_request_id:
                rpc = self.exchange.request(goal.exchange_request_id, goal.exchange_view,
                                            frame_header(goal.rgb_image.header))
                stage = 'waiting_for_sam_file'
            handle.publish_feedback(SegmentImage.Feedback(stage=stage))
            mask, score, count, mock = segment_image(
                self.get_parameter('endpoint').value, goal.rgb_image,
                text=goal.target_text,
                box=list(goal.target_box) if any(goal.target_box) else [],
                point=[goal.selection_u, goal.selection_v] if goal.use_selection_point else [],
                policy=goal.selection_policy or 'unique',
                timeout=float(self.get_parameter('timeout').value),
                canceled=lambda: handle.is_cancel_requested or not rclpy.ok(),
                allow_mock=self.get_parameter('allow_mock').value, rpc=rpc)
            if handle.is_cancel_requested:
                raise RequestCanceled('segmentation canceled before publication')
            result.object_mask = mask
            result.score, result.instance_count, result.mock = score, count, mock
            result.success = True
            result.message = ('MOCK_ONLY' if mock else 'OK') + ': segmented one target'
            handle.succeed()
        except RequestCanceled as error:
            set_failure(result, error, 'sam3', '/segment_image')
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception as error:
            set_failure(result, error, 'sam3', '/segment_image')
            self.get_logger().error(result.message)
            handle.abort()
        finally:
            self.lock.release()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationActionServer()
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
