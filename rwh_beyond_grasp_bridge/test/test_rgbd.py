"""Protocol tests use synthetic masks, never a substitute for a SAM checkpoint test."""
import copy
import sys
from pathlib import Path
import numpy as np
import pytest
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from rclpy.parameter import Parameter
from rwh_beyond_msgs.action import InferGrasps
from rwh_beyond_grasp_bridge.segmentation_client import rgb_array
from rwh_beyond_grasp_bridge.observed_geometry import observed_bounds
from rwh_beyond_grasp_bridge.depth_converter import depth_xyz
from test_bridge import MockServer, mock, bridge, run


def rgbd_goal():
    header=Header(frame_id='camera_optical');header.stamp.sec=123
    rgb=np.zeros((32,32,3),np.uint8);rgb[:]=[10,100,200]
    z=np.full((32,32),.4,np.float32)
    return InferGrasps.Goal(input_mode=InferGrasps.Goal.RGBD,
        rgb_image=Image(header=header,height=32,width=32,encoding='rgb8',step=96,data=rgb.tobytes()),
        depth_image=Image(header=header,height=32,width=32,encoding='32FC1',step=128,data=z.tobytes()),
        camera_info=CameraInfo(header=header,height=32,width=32,k=[100.,0.,16.,0.,100.,16.,0.,0.,1.]),
        target_text='box',gripper_name='piper_hand',num_grasps=20,top_k=2)


def fake_segmentation(payload):
    mask=np.zeros(payload['rgb'].shape[:2],np.uint8);mask[4:28,4:28]=1
    return dict(mask=mask,score=.85,instance_count=1,backend='mock',mock=True)


def test_rgbd_action_synthetic_mask_to_grasp_wire_and_header(bridge,mock):
    segment=MockServer()
    try:
        segment.handler=fake_segmentation
        bridge[0].set_parameters([Parameter('segmentation_endpoint',value=segment.endpoint),Parameter('allow_mock_segmentation',value=True)])
        result=run(bridge,rgbd_goal()).result
        assert result.success and result.message.startswith('MOCK_ONLY:')
        assert result.segmentation_score==pytest.approx(.85)
        assert result.segmentation_instances==1 and len(result.object_mask.data)==1024
        assert result.candidates[0].pose.header==rgbd_goal().depth_image.header
        assert mock.requests[0]['point_cloud'].shape==(22*22,3)
        assert np.allclose(mock.requests[0]['point_cloud'][:,2],.4)
        assert segment.requests[0]['text']=='box'
    finally:segment.close()


def test_mock_segmentation_rejected_by_default_and_grasp_not_called(bridge,mock):
    segment=MockServer()
    try:
        segment.handler=fake_segmentation
        bridge[0].set_parameters([Parameter('segmentation_endpoint',value=segment.endpoint)])
        result=run(bridge,rgbd_goal()).result
        assert not result.success and result.message.startswith('INVALID_RESPONSE:')
        assert not mock.requests
    finally:segment.close()


def test_rgb_depth_mismatch_fails_before_model(bridge,mock):
    goal=rgbd_goal();goal.rgb_image.header=copy.deepcopy(goal.rgb_image.header);goal.rgb_image.header.frame_id='wrong'
    result=run(bridge,goal).result
    assert not result.success and result.message.startswith('INVALID_INPUT:')
    assert not mock.requests


def test_rgb_strided_bgr_conversion():
    goal=rgbd_goal();rgb=goal.rgb_image
    rgb.encoding='bgr8';rgb.step=98
    rgb.data=b''.join(bytes([1,2,3])*32+b'xx' for _ in range(32))
    assert rgb_array(rgb,goal.depth_image)[0,0].tolist()==[3,2,1]


def test_observed_geometry_measures_height_from_background_support():
    goal=rgbd_goal();z=np.full((32,32),.5,np.float32);z[8:24,8:24]=.45
    goal.depth_image.data=z.tobytes()
    mask=np.zeros((32,32),np.uint8);mask[8:24,8:24]=1
    mask=Image(header=goal.depth_image.header,height=32,width=32,encoding='mono8',step=32,data=mask.tobytes())
    xyz=depth_xyz(goal.depth_image,goal.camera_info,mask)
    transform=np.eye(4);transform[:3,:3]=np.diag([1,-1,-1]);transform[2,3]=.5
    center,dims,support=observed_bounds(xyz,goal.depth_image,goal.camera_info,mask,transform)
    assert support==pytest.approx(0.) and dims[2]==pytest.approx(.056,abs=1e-6)
    assert center[2]==pytest.approx(.025,abs=1e-6)


def test_observed_geometry_rejects_missing_support():
    goal=rgbd_goal();mask=Image(header=goal.depth_image.header,height=32,width=32,encoding='mono8',step=32,data=bytes([1])*1024)
    xyz=depth_xyz(goal.depth_image,goal.camera_info,mask)
    with pytest.raises(ValueError):observed_bounds(xyz,goal.depth_image,goal.camera_info,mask,np.eye(4))


@pytest.mark.parametrize('failure',['malformed','no_object','timeout'])
def test_segmentation_failure_never_reaches_grasp_model(bridge,mock,failure):
    import time
    segment=MockServer()
    def bad(payload):
        if failure=='timeout':time.sleep(.5)
        value=fake_segmentation(payload)
        if failure=='malformed':value['mask']=np.ones((2,2),np.uint8)
        if failure=='no_object':return dict(error='NO_OBJECT: no matching target')
        return value
    try:
        segment.handler=bad
        bridge[0].set_parameters([Parameter('segmentation_endpoint',value=segment.endpoint),Parameter('allow_mock_segmentation',value=True)])
        result=run(bridge,rgbd_goal()).result
        expected={'malformed':'INVALID_RESPONSE:','no_object':'SERVER_ERROR:','timeout':'TIMEOUT:'}[failure]
        assert not result.success and result.message.startswith(expected)
        assert not mock.requests
    finally:segment.close()


def test_local_cloud_stage_requires_segmentation_and_expires():
    import time
    from types import SimpleNamespace
    from rwh_beyond_grasp_bridge.grasp_coordinator import GraspCoordinator
    node=SimpleNamespace(perception=None)
    with pytest.raises(RuntimeError,match='segment_target first'):
        GraspCoordinator.perception_context(node)
    node.perception={'captured_at':time.monotonic()-31}
    with pytest.raises(RuntimeError,match='expired'):
        GraspCoordinator.perception_context(node)
    assert node.perception is None


def test_local_cloud_published_before_grasp_and_uses_original_camera_frame():
    import time
    from types import SimpleNamespace
    from rwh_beyond_grasp_bridge.grasp_coordinator import GraspCoordinator
    goal=rgbd_goal();mask=fake_segmentation({'rgb':np.zeros((32,32,3),np.uint8)})['mask']
    mask=Image(header=goal.depth_image.header,height=32,width=32,encoding='mono8',step=32,data=mask.tobytes())
    context=dict(captured_at=time.monotonic(),frame={'depth':goal.depth_image,'info':goal.camera_info},mask=mask)
    clouds=[];stages=[]
    node=SimpleNamespace(perception_context=lambda:context,points_pub=SimpleNamespace(publish=clouds.append),stage=stages.append)
    GraspCoordinator.build_object_cloud(node)
    assert len(clouds)==1 and clouds[0].header==goal.depth_image.header
    assert clouds[0].width==24*24 and context['xyz'].shape==(24*24,3)
    assert clouds[0] is context['cloud'] and stages[0].startswith('local_object_cloud')


def test_local_snapshot_invalidated_when_arm_moves_but_not_when_gripper_changes():
    import time
    from types import SimpleNamespace
    from sensor_msgs.msg import JointState
    from rwh_beyond_grasp_bridge.grasp_coordinator import GraspCoordinator
    context=dict(captured_at=time.monotonic(),capture_joints=JointState(name=['joint1','gripper'],position=[0.,.1]))
    node=SimpleNamespace(perception=context,joints=JointState(name=['joint1','gripper'],position=[0.,0.]))
    assert GraspCoordinator.perception_context(node) is context
    node.joints.position=[.1,0.]
    with pytest.raises(RuntimeError,match='Arm moved'):
        GraspCoordinator.perception_context(node)
    assert node.perception is None
