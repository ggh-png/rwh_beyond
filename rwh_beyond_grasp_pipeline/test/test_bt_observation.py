"""BT observation -> motion boundary: reject stale/moving/mismatched captures."""
import copy
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from rwh_beyond_msgs.action import RunGraspStage
from rwh_beyond_grasp_pipeline.perception_stages import PerceptionStages


def observation():
    header=Header(frame_id='camera');header.stamp.sec=1
    depth=Image(header=copy.deepcopy(header),width=12,height=12,encoding='16UC1',step=24,data=np.full((12,12),1000,np.uint16).tobytes())
    mask=Image(header=copy.deepcopy(header),width=12,height=12,encoding='mono8',step=12,data=bytes([1])*144)
    info=CameraInfo(header=copy.deepcopy(header),width=12,height=12,k=[100.,0.,6.,0.,100.,6.,0.,0.,1.])
    joints=JointState(name=[f'joint{i}' for i in range(1,7)]+['gripper'],position=[0.]*7)
    request=RunGraspStage.Goal(observation_id='exposure',depth_image=depth,camera_info=info,
        object_mask=mask,object_cloud=create_cloud_xyz32(copy.deepcopy(header),np.ones((144,3)).tolist()),capture_joints=joints)
    transform=TransformStamped();transform.transform.rotation.w=1.
    node=NS(attached=False,scene_active=False,joints=copy.deepcopy(joints),base='base_link',
        get_clock=lambda:NS(now=lambda:Time(seconds=1.1)),
        buffer=NS(lookup_transform=Mock(return_value=transform)))
    node.perception_context=MethodType(PerceptionStages.perception_context,node)
    return node,request


def test_bt_observation_is_used_without_an_internal_segmentation_session():
    node,request=observation()
    result=PerceptionStages.accept_observation(node,request)
    assert result['cloud'] is request.object_cloud
    assert result['xyz'].shape==(144,3)
    assert node.observation_id=='exposure'
    args=node.buffer.lookup_transform.call_args.args
    assert args[:2]==('base_link','camera')
    assert args[2]==Time.from_msg(request.depth_image.header.stamp)


@pytest.mark.parametrize('kind',['stale','future','moved','missing_joints','frame','mask_stamp','id'])
def test_bad_bt_capture_is_rejected_before_planning(kind):
    node,request=observation()
    if kind=='stale':node.get_clock=lambda:NS(now=lambda:Time(seconds=40.))
    if kind=='future':node.get_clock=lambda:NS(now=lambda:Time(seconds=.5))
    if kind=='moved':node.joints.position[0]=.03
    if kind=='missing_joints':request.capture_joints=JointState()
    if kind=='frame':request.object_cloud.header.frame_id='other'
    if kind=='mask_stamp':request.object_mask.header.stamp.sec=2
    if kind=='id':request.observation_id=''
    with pytest.raises((ValueError,RuntimeError)):
        PerceptionStages.accept_observation(node,request)
