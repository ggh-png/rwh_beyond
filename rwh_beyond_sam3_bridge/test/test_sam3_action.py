"""Independent SAM ROS boundary: no coordinator, MoveIt or GraspGenX is started."""
import threading
import time

import msgpack
import msgpack_numpy
import numpy as np
import pytest
import rclpy
import zmq
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from rwh_beyond_msgs.action import SegmentImage
from rwh_beyond_sam3_bridge.action_server import SegmentationActionServer


def wait(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done(), 'ROS future timeout'
    return future.result()


@pytest.fixture
def sam():
    context = zmq.Context()
    ready, stop = threading.Event(), threading.Event()
    state = {'requests': [], 'delay': 0.0}
    def serve():
        socket = context.socket(zmq.REP)
        port = socket.bind_to_random_port('tcp://127.0.0.1')
        state['endpoint'] = f'tcp://127.0.0.1:{port}'
        ready.set()
        try:
            while not stop.is_set():
                if not socket.poll(20):
                    continue
                payload = msgpack.unpackb(socket.recv(), raw=False,
                                          object_hook=msgpack_numpy.decode)
                state['requests'].append(payload)
                time.sleep(state['delay'])
                result = dict(mask=np.ones((12, 12), np.uint8), score=.8,
                              instance_count=1, backend='mock', mock=True)
                socket.send(msgpack.packb(result, default=msgpack_numpy.encode,
                                          use_bin_type=True))
        finally:
            socket.close(0)
    transport = threading.Thread(target=serve, daemon=True)
    transport.start()
    assert ready.wait(5)
    rclpy.init()
    server = SegmentationActionServer()
    server.set_parameters([Parameter('endpoint', value=state['endpoint']),
                           Parameter('allow_mock', value=True)])
    caller = Node('independent_sam_test')
    client = ActionClient(caller, SegmentImage, 'segment_image')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(caller)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    assert client.wait_for_server(timeout_sec=5)
    yield server, client, state
    client.destroy()
    executor.shutdown(timeout_sec=3)
    worker.join(3)
    server.destroy_node()
    caller.destroy_node()
    rclpy.try_shutdown()
    stop.set()
    transport.join(3)
    assert not transport.is_alive()
    context.term()


def goal():
    header = Header(frame_id='camera_optical')
    header.stamp.sec = 123
    return SegmentImage.Goal(
        rgb_image=Image(header=header, height=12, width=12, encoding='rgb8',
                        step=36, data=bytes([80])*432),
        target_text='box', selection_policy='unique')


def test_independent_segmentation_action_preserves_header_and_uses_rgb_only(sam):
    handle = wait(sam[1].send_goal_async(goal()))
    outcome = wait(handle.get_result_async())
    assert outcome.status == 4 and outcome.result.success
    assert outcome.result.mock and outcome.result.message.startswith('MOCK_ONLY:')
    assert outcome.result.object_mask.header == goal().rgb_image.header
    assert outcome.result.object_mask.encoding == 'mono8'
    assert len(outcome.result.object_mask.data) == 144
    request = sam[2]['requests'][0]
    assert request['rgb'].shape == (12, 12, 3) and request['text'] == 'box'
    assert 'depth' not in request and 'point_cloud' not in request


def test_mock_segmentation_is_rejected_by_default(sam):
    sam[0].set_parameters([Parameter('allow_mock', value=False)])
    handle = wait(sam[1].send_goal_async(goal()))
    outcome = wait(handle.get_result_async())
    assert outcome.status == 6 and not outcome.result.success
    assert outcome.result.message.startswith('INVALID_RESPONSE:')
    assert not outcome.result.object_mask.data


def test_invalid_rgb_is_rejected_before_network_request(sam):
    value = goal()
    value.rgb_image.encoding = 'mono8'
    handle = wait(sam[1].send_goal_async(value))
    outcome = wait(handle.get_result_async())
    assert not outcome.result.success and outcome.result.message.startswith('INVALID_INPUT:')
    assert not sam[2]['requests']


def test_cancel_and_busy_rejection_release_the_server(sam):
    sam[2]['delay'] = .5
    handle = wait(sam[1].send_goal_async(goal()))
    deadline = time.monotonic() + 3
    while not sam[2]['requests'] and time.monotonic() < deadline:
        time.sleep(.01)
    assert sam[2]['requests']
    assert not wait(sam[1].send_goal_async(goal())).accepted
    assert wait(handle.cancel_goal_async()).goals_canceling
    outcome = wait(handle.get_result_async())
    assert outcome.status == 5 and not outcome.result.success
    assert not outcome.result.object_mask.data
    assert not sam[0].lock.locked()
