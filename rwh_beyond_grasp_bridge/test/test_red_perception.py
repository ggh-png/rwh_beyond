import copy
from types import SimpleNamespace
import numpy as np
import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from rwh_beyond_grasp_bridge.red_perception import red_mask, pose_matrix, matrix_pose, set_pair
from rwh_beyond_grasp_bridge.grasp_coordinator import GraspCoordinator


def test_color_mask_rejects_table_noise_and_erodes_depth_edges():
    rgb=np.zeros((20,20,3),np.uint8)
    rgb[:]=[140,107,76]  # table, never a red target
    rgb[4:16,4:16]=[220,35,20]
    rgb[0,0]=[255,0,0]  # isolated noise
    padded=b''.join(row.tobytes()+b'pad' for row in rgb)
    image=Image(height=20,width=20,encoding='rgb8',step=63,data=padded)
    mask=red_mask(image)
    assert mask.sum()==64
    assert mask[6:14,6:14].all() and not mask[0,0]


def test_pose_roundtrip_and_model_fingertip_offset():
    from scipy.spatial.transform import Rotation
    model=np.eye(4);model[:3,:3]=Rotation.from_euler('x',np.pi).as_matrix()
    model[:3,3]=[.35,0,.15]
    offset=np.eye(4);offset[:3,3]=[0,0,.13]
    offset[:3,:3]=Rotation.from_euler('z',-np.pi/2).as_matrix()
    tcp=model@offset
    assert np.allclose(tcp[:3,3],[.35,0,.02])
    assert np.allclose(offset[:3,:3]@np.array([0,1,0]),[1,0,0])
    assert np.allclose(pose_matrix(matrix_pose(tcp)),tcp)


def test_allowed_contacts_change_only_target_finger_pairs():
    matrix=AllowedCollisionMatrix(entry_names=['arm','table'],entry_values=[
        AllowedCollisionEntry(enabled=[True,False]),AllowedCollisionEntry(enabled=[False,True])])
    original=copy.deepcopy(matrix)
    set_pair(matrix,'finger','target',True)
    assert matrix.entry_values[0].enabled[:2]==original.entry_values[0].enabled
    assert matrix.entry_values[0].enabled[matrix.entry_names.index('target')] is False
    set_pair(matrix,'finger','target',False)
    assert matrix.entry_values[matrix.entry_names.index('finger')].enabled[matrix.entry_names.index('target')] is False


def test_partial_cartesian_path_never_executes():
    coordinator=SimpleNamespace(state=lambda:None,
        cartesian=lambda *_:SimpleNamespace(error_code=SimpleNamespace(val=1),fraction=.8))
    with pytest.raises(RuntimeError,match='Cartesian path rejected'):
        GraspCoordinator.execute_cartesian(coordinator,Pose())
