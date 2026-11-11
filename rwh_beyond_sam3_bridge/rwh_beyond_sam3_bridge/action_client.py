"""Blocking ROS Action client for registered RGB-D workflows; no ZMQ calls."""
import time

import rclpy
from rwh_beyond_msgs.action import SegmentImage
from rwh_beyond_model_utils.rgb_converter import rgb_array
from rwh_beyond_model_utils.zmq_client import ProtocolError, RequestCanceled, ServerError


def segment_via_action(client, rgb, depth, text='', box=(), point=(), policy='unique',
                       timeout=60.0, canceled=lambda: False, allow_mock=False):
    rgb_array(rgb, depth)  # Validate registration before sending any model request.
    deadline = time.monotonic() + timeout

    def check():
        if canceled() or not rclpy.ok():
            raise RequestCanceled('segmentation client canceled')
        if time.monotonic() >= deadline:
            raise TimeoutError('SAM ROS Action deadline exceeded')

    def wait(future):
        while not future.done():
            check()
            time.sleep(.01)
        check()
        return future.result()

    while not client.server_is_ready():
        check()
        time.sleep(.01)
    goal = SegmentImage.Goal(
        rgb_image=rgb, target_text=text, target_box=list(box) if box else [0.0]*4,
        use_selection_point=bool(point), selection_policy=policy)
    if point:
        goal.selection_u, goal.selection_v = map(int, point)
    accepted = client.send_goal_async(goal)
    try:
        handle = wait(accepted)
    except Exception:
        # If acceptance arrives after the caller stopped waiting, cancel it too.
        def cancel_late(future):
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        accepted.add_done_callback(cancel_late)
        raise
    if not handle.accepted:
        raise ServerError('SAM ROS Action rejected: bridge busy')
    try:
        response = wait(handle.get_result_async())
    except Exception:
        handle.cancel_goal_async()
        raise
    result = response.result
    if not result.success:
        prefix, _, message = result.message.partition(':')
        kind = {'TIMEOUT': TimeoutError, 'INVALID_INPUT': ValueError,
                'INVALID_RESPONSE': ProtocolError, 'SERVER_ERROR': ServerError,
                'CANCELED': RequestCanceled}.get(prefix, ServerError)
        raise kind(message.strip() or result.message)
    if result.mock and not allow_mock:
        raise ProtocolError('Real SAM 3 segmentation required; no automatic fallback')
    if response.status != 4 or not result.message.startswith(('OK:', 'MOCK_ONLY:')):
        raise ProtocolError('Invalid segmentation action result')
    return result.object_mask, result.score, result.instance_count, result.mock
