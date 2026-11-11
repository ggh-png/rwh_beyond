import copy
import threading
import time
import msgpack
import msgpack_numpy
import numpy as np
import pytest
import rclpy
import zmq
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from rwh_beyond_sam3_bridge.action_server import SegmentationActionServer
from rwh_beyond_msgs.action import InferGrasps
from rwh_beyond_grasp_bridge.inference_action_server import InferenceActionServer
from rwh_beyond_grasp_bridge.cloud_converter import cloud_xyz
from rwh_beyond_grasp_bridge.pose_converter import quaternion_xyzw
from rwh_beyond_grasp_bridge.zmq_client import request, ProtocolError

class MockServer:
    def __init__(self):
        self.context = zmq.Context()
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.handler = lambda _: self.good()
        self.requests = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        assert self.ready.wait(5)
    @staticmethod
    def good():
        matrices = np.repeat(np.eye(4, dtype=np.float32)[None], 2, axis=0)
        matrices[0,:3,3] = [.1,.2,.3]
        matrices[1,:3,:3] = np.diag([1,-1,-1])
        return dict(grasps=matrices, confidences=np.array([.6,.9],np.float32),
                    gripper_name='piper_hand', timing={'infer_ms': 1.0})
    def run(self):
        socket = self.context.socket(zmq.REP)
        port = socket.bind_to_random_port('tcp://127.0.0.1')
        self.endpoint = f'tcp://127.0.0.1:{port}'
        self.ready.set()
        try:
            while not self.stop.is_set():
                if socket.poll(50):
                    payload = msgpack.unpackb(socket.recv(), raw=False, object_hook=msgpack_numpy.decode)
                    self.requests.append(payload)
                    response = self.handler(payload)
                    socket.send(msgpack.packb(response, default=msgpack_numpy.encode, use_bin_type=True))
        finally:
            socket.close(0)
    def close(self):
        self.stop.set()
        self.thread.join(3)
        assert not self.thread.is_alive()
        self.context.term()

@pytest.fixture
def mock():
    server = MockServer()
    yield server
    server.close()

def wait(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done(), 'ROS future timeout'
    return future.result()

@pytest.fixture
def bridge(mock, request):
    # Use an unreachable action name for the independent case; DDS discovery
    # can briefly retain the SAM server from the preceding fixture context.
    include_sam = getattr(request, 'param', True)
    args = None if include_sam else [
        '--ros-args', '-r', 'segment_image:=/intentionally_absent_sam_test']
    rclpy.init(args=args)
    node = InferenceActionServer()
    node.endpoint = mock.endpoint
    node.timeout = .3
    sam = SegmentationActionServer() if include_sam else None
    if sam is not None:
        sam.set_parameters([rclpy.parameter.Parameter('timeout', value=.3)])
    node.test_sam_node = sam
    client_node = Node('test_graspgenx_client')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    if sam is not None:
        executor.add_node(sam)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, InferGrasps, 'infer_grasps')
    assert client.wait_for_server(timeout_sec=5)
    if sam is not None:
        assert node.segment.wait_for_server(timeout_sec=5)
    yield node, client, client_node
    client.destroy()
    executor.shutdown(timeout_sec=3)
    thread.join(3)
    node.destroy_node()
    if sam is not None:
        sam.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()

def goal():
    header = Header(frame_id='camera_depth_optical_frame')
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    cloud = create_cloud_xyz32(header, [[.1,.2,.3]]*100)
    return InferGrasps.Goal(object_cloud=cloud, gripper_name='piper_hand', num_grasps=100, top_k=20)

def run(bridge, value=None):
    handle = wait(bridge[1].send_goal_async(value or goal()))
    assert handle.accepted
    return wait(handle.get_result_async())

def test_success_official_wire_header_and_quaternion(bridge, mock):
    outcome = run(bridge)
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED
    result = outcome.result
    assert result.success and len(result.candidates) == 2
    best = result.candidates[0]
    assert best.pose.header == goal().object_cloud.header
    assert best.score == pytest.approx(.9)
    assert abs(best.pose.pose.orientation.x) == pytest.approx(1)
    assert result.candidates[1].pose.pose.position.z == pytest.approx(.3)
    wire = mock.requests[0]
    assert wire['action'] == 'infer' and wire['point_cloud'].dtype == np.float32
    assert wire['point_cloud'].shape == (100,3)
    assert wire['topk_num_grasps'] == 20

@pytest.mark.parametrize('kind,prefix', [('server','SERVER_ERROR'),('empty','NO_CANDIDATES'),
    ('rotation','INVALID_RESPONSE'),('nan','INVALID_RESPONSE'),('missing','INVALID_RESPONSE'),
    ('gripper','INVALID_RESPONSE'),('shape','INVALID_RESPONSE')])
def test_failed_response(bridge,mock,kind,prefix):
    response = mock.good()
    if kind == 'server': response={'error':'RuntimeError: GPU unavailable'}
    elif kind == 'empty': response.update(grasps=np.empty((0,4,4)),confidences=np.empty(0))
    elif kind == 'rotation': response['grasps'][0,0,0] = -1
    elif kind == 'nan': response['grasps'][0,0,3] = np.nan
    elif kind == 'missing': del response['confidences']
    elif kind == 'gripper': response['gripper_name']='other'
    elif kind == 'shape': response['confidences']=np.zeros(3)
    mock.handler=lambda _:response
    outcome=run(bridge)
    assert outcome.status == GoalStatus.STATUS_ABORTED
    assert not outcome.result.success and not outcome.result.candidates
    assert outcome.result.message.startswith(prefix)

def test_timeout_then_next_request_recovers(bridge,mock):
    def delayed(_):
        time.sleep(.5)
        return mock.good()
    mock.handler=delayed
    assert run(bridge).result.message.startswith('TIMEOUT')
    time.sleep(.3)
    mock.handler=lambda _:mock.good()
    assert run(bridge).result.success

@pytest.mark.parametrize('kind', ['frame','points','counts','stride'])
def test_invalid_input_does_not_reach_server(bridge,mock,kind):
    value=goal()
    if kind=='frame': value.object_cloud.header.frame_id=''
    elif kind=='points': value.object_cloud=create_cloud_xyz32(Header(frame_id='camera'),[[0,0,0]])
    elif kind=='counts': value.top_k=101
    else: value.object_cloud.row_step=1
    result=run(bridge,value).result
    assert result.message.startswith('INVALID_INPUT') and not mock.requests

def test_cancellation_and_busy_rejection(bridge,mock):
    mock.handler=lambda _: (time.sleep(.5) or mock.good())
    handle=wait(bridge[1].send_goal_async(goal()))
    assert handle.accepted
    second=wait(bridge[1].send_goal_async(goal()))
    assert not second.accepted
    cancel=wait(handle.cancel_goal_async())
    assert cancel.goals_canceling
    result=wait(handle.get_result_async())
    assert result.status==GoalStatus.STATUS_CANCELED
    assert result.result.message.startswith('CANCELED')

def test_padded_rows_big_endian_and_nonfinite_filter():
    import struct
    cloud=PointCloud2(header=Header(frame_id='camera'),height=2,width=2,
       fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')],
       is_bigendian=True,point_step=12,row_step=28,is_dense=False)
    cloud.data=struct.pack('>ffffff',1,2,3,4,5,6)+b'pad!'+struct.pack('>ffffff',7,8,9,float('nan'),0,0)+b'pad!'
    assert np.array_equal(cloud_xyz(cloud,3),[[1,2,3],[4,5,6],[7,8,9]])

def test_quaternion_known_rotations():
    assert np.allclose(quaternion_xyzw(np.eye(3)), [0,0,0,1])
    q=quaternion_xyzw(np.array([[0,-1,0],[1,0,0],[0,0,1]]))
    assert np.allclose(q,[0,0,2**-.5,2**-.5])

def test_non_map_response_rejected(mock):
    mock.handler=lambda _: ['bad']
    with pytest.raises(ProtocolError): request(mock.endpoint, {'action':'health'}, 1)


def test_object_dtype_decode_is_rejected():
    from rwh_beyond_grasp_bridge.zmq_client import decode_numpy
    with pytest.raises(ValueError):
        decode_numpy({b'nd':True,b'type':'O',b'shape':(1,),b'data':b'not-a-pickle'})


def depth_goal(encoding='16UC1'):
    from sensor_msgs.msg import Image, CameraInfo
    header = Header(frame_id='camera_depth_optical_frame')
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    values = np.full((12,12), 1000 if encoding=='16UC1' else 1.,
                     dtype=np.uint16 if encoding=='16UC1' else np.float32)
    labels = np.ones((12,12), np.uint8)
    labels[:2] = 0
    depth = Image(header=copy.deepcopy(header), height=12, width=12, encoding=encoding,
        step=12*values.dtype.itemsize, data=values.tobytes())
    mask = Image(header=copy.deepcopy(header), height=12, width=12, encoding='mono8', step=12, data=labels.tobytes())
    info = CameraInfo(header=copy.deepcopy(header), height=12, width=12,
        k=[100.,0.,6.,0.,100.,6.,0.,0.,1.])
    return InferGrasps.Goal(input_mode=InferGrasps.Goal.DEPTH_IMAGE,
        depth_image=depth, camera_info=info, object_mask=mask,
        gripper_name='piper_hand', num_grasps=100, top_k=20)


@pytest.mark.parametrize('encoding', ['16UC1','32FC1'])
def test_depth_action_to_official_infer_protocol(bridge, mock, encoding):
    value = depth_goal(encoding)
    outcome = run(bridge, value)
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED
    assert outcome.result.candidates[0].pose.header == value.depth_image.header
    payload = mock.requests[0]
    assert payload['action'] == 'infer'
    xyz = payload['point_cloud']
    assert xyz.shape == (120,3) and xyz.dtype == np.float32
    assert np.allclose(xyz[:,2], 1.)
    assert np.allclose(xyz[0], [-.06,-.04,1.])
    assert 'depth' not in payload  # official named-gripper infer accepts object XYZ


@pytest.mark.parametrize('kind', ['missing_mask','intrinsics','distortion','frame','stamp',
    'shape','step','ambiguous','empty_depth','mode'])
def test_invalid_depth_rejected_before_zmq(bridge,mock,kind):
    value=depth_goal()
    if kind=='missing_mask': value.object_mask.data=[]
    elif kind=='intrinsics': value.camera_info.k=[0.]*9
    elif kind=='distortion': value.camera_info.d=[.1,0.,0.,0.,0.]
    elif kind=='frame': value.object_mask.header.frame_id='rgb_unregistered'
    elif kind=='stamp': value.object_mask.header.stamp.sec=121
    elif kind=='shape': value.camera_info.width=13
    elif kind=='step': value.depth_image.step=1
    elif kind=='ambiguous': value.object_cloud=goal().object_cloud
    elif kind=='empty_depth': value.depth_image.data=bytes(len(value.depth_image.data))
    else: value.input_mode=42
    result=run(bridge,value).result
    assert result.message.startswith('INVALID_INPUT') and not mock.requests


def test_depth_padded_big_endian_downsample_and_invalid_pixels():
    import struct
    from rwh_beyond_grasp_bridge.depth_converter import depth_xyz
    value=depth_goal()
    row=struct.pack('>12H', *([1000]*12))+b'xx'
    value.depth_image.is_bigendian=1
    value.depth_image.step=26
    value.depth_image.data=row*12
    result=depth_xyz(value.depth_image,value.camera_info,value.object_mask,
                     min_points=100,max_points=100)
    assert result.shape==(100,3) and np.allclose(result[:,2],1.)


def test_mock_response_is_labeled(bridge,mock):
    def response(_):
        value=mock.good()
        value['mock']=True
        return value
    mock.handler=response
    result=run(bridge).result
    assert result.success and result.message.startswith('MOCK_ONLY')


@pytest.mark.parametrize('bridge', [False], indirect=True)
def test_point_cloud_grasp_bridge_runs_without_a_sam_ros_server(bridge):
    assert not bridge[0].segment.server_is_ready()
    outcome = run(bridge)
    assert outcome.status == 4 and outcome.result.success
    assert len(outcome.result.candidates) == 2
