"""No hardware: symmetric grasp choice and actual MoveIt constraint requests."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState, RobotTrajectory, AllowedCollisionMatrix
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from rwh_beyond_grasp_pipeline.grasp_motion_policy import (
    DEFAULTS, validate_policy, ordered_variants, wrist_constraints,
    within_wrist_limits, trajectory_within_wrist_limits, trajectory_reaches_state)
from rwh_beyond_grasp_pipeline.red_perception import matrix_pose
from rwh_beyond_grasp_pipeline.selection_stages import SelectionStages
from rwh_beyond_grasp_pipeline.motion_stages import MotionStages


def state(q4=0., q6=0.):
    return RobotState(joint_state=JointState(name=[f'joint{i}' for i in range(1,7)],
        position=[0.,0.,0.,q4,0.,q6]))


def trajectory(q4=0., q6=0.):
    result=RobotTrajectory()
    result.joint_trajectory.joint_names=list(state().joint_state.name)
    result.joint_trajectory.points=[JointTrajectoryPoint(positions=list(state(q4,q6).joint_state.position))]
    return result


def candidate(angle):
    pose=PoseStamped()
    q=Rotation.from_euler('z',angle,degrees=True).as_quat()
    pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=q
    return NS(pose=pose,score=1.)


def test_symmetric_grasp_avoids_180_degree_flip_and_keeps_conversion():
    conversion=np.eye(4)
    conversion[:3,:3]=Rotation.from_euler('z',-90,degrees=True).as_matrix()
    choices=ordered_variants([candidate(180)],np.eye(4),conversion,
        Rotation.from_euler('z',-90,degrees=True).as_matrix())
    assert choices[0][3] is True
    assert choices[0][0]==pytest.approx(0,abs=1e-8)
    assert choices[1][0]==pytest.approx(math.pi)


def test_rotation_preference_can_choose_lower_score_candidate():
    choices=ordered_variants([candidate(80),candidate(10)],np.eye(4),np.eye(4),np.eye(3))
    assert choices[0][1]==1
    legacy=ordered_variants([candidate(80),candidate(10)],np.eye(4),np.eye(4))
    assert [(c[1],c[3]) for c in legacy]==[(0,False),(0,True),(1,False),(1,True)]


@pytest.mark.parametrize('values',[
    {'moveit_pregrasp_num_planning_attempts':0},
    {'moveit_pregrasp_num_planning_attempts':True},
    {'moveit_pregrasp_planner_id':''},
    {'moveit_prefer_current_grasp_orientation':1},
    {'moveit_pregrasp_wrist_constraint_enabled':'false'},
    *[{'moveit_pregrasp_wrist_max_deviation_deg':v} for v in [0,180,-1,True,float('nan')]],
])
def test_invalid_policy(values):
    with pytest.raises(ValueError): validate_policy(values)


def test_limits_reject_large_detour_even_when_endpoint_is_close():
    limits=wrist_constraints(state(.2,-.3),100)
    assert within_wrist_limits(state(.3,-.4),limits)
    assert not within_wrist_limits(state(.2,2.),limits)
    path=trajectory(.2,-.3)
    path.joint_trajectory.points.insert(0,JointTrajectoryPoint(positions=[0.,0.,0.,2.5,0.,-.3]))
    assert not trajectory_within_wrist_limits(path,limits)
    assert trajectory_within_wrist_limits(trajectory(.3,-.4),limits)


def test_actual_selection_sends_moveit_path_constraints(monkeypatch):
    import rwh_beyond_grasp_pipeline.selection_stages as module
    monkeypatch.setattr(module,'observed_bounds',lambda *a:(np.zeros(3),np.ones(3),0.))
    monkeypatch.setattr(module,'Path',lambda *a:NS(write_text=lambda text:None))
    params=dict(grasp_top_down_max_angle_deg=35.,grasp_table_clearance_m=.01,
        grasp_target_margin_m=.05,model_to_tcp_rpy=[0.,0.,0.],model_to_tcp_xyz=[0.,0.,0.],
        max_candidates=1,pregrasp_distance=.08,lift_distance=.08,octomap_enabled=False)
    transform=np.eye(4)
    transform[:3,:3]=Rotation.from_euler('x',180,degrees=True).as_matrix()
    transform[2,3]=.2
    pose=matrix_pose(transform)
    planned=[]
    def call(name,request,*args):
        if name=='scene':return NS(scene=NS(allowed_collision_matrix=AllowedCollisionMatrix(),world=NS(collision_objects=[])))
        if name=='fk':return NS(error_code=NS(val=1),pose_stamped=[PoseStamped(pose=pose)])
        if name=='plan':
            planned.append(request.motion_plan_request)
            return NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=trajectory()))
        raise AssertionError(name)
    node=NS(attached=False,scene_active=False,base='base_link',tcp='tcp_link',
        grasp_motion_policy=validate_policy({}),state=state,call=call,stage=Mock(),apply=Mock(),
        get_parameter=lambda name:NS(value=params[name]),ik=Mock(return_value=state()),
        cartesian=Mock(return_value=NS(fraction=1.,error_code=NS(val=1),solution=trajectory())),
        visualization=NS(publish_selection=Mock()))
    node.grasp_filter_settings=lambda:SelectionStages.grasp_filter_settings(node)
    context=dict(frame=dict(depth=NS(header=None),info=None),xyz=np.zeros((100,3)),
        mask=None,base_T_camera=np.eye(4),backend='sam3')
    SelectionStages.select_feasible_grasp(node,context,[NS(pose=PoseStamped(pose=pose),score=1.)])
    assert node.prepared and len(planned)==1
    assert planned[0].num_planning_attempts==3
    assert planned[0].planner_id=='RRTConnect'
    constraints=planned[0].path_constraints.joint_constraints
    assert [c.joint_name for c in constraints]==['joint4','joint6']
    assert constraints[0].tolerance_above==pytest.approx(math.radians(100))
    assert node.selected_wrist_limits is not None


def test_replanned_cartesian_rotation_violation_stops_before_execution():
    node=NS(state=state,cartesian=lambda *a:NS(error_code=NS(val=1),fraction=1.,solution=trajectory(q6=2.)),
        execute_trajectory=Mock())
    with pytest.raises(RuntimeError,match='wrist rotation'):
        MotionStages.execute_cartesian(node,None,wrist_constraints(state(),100))
    node.execute_trajectory.assert_not_called()


def test_planner_success_with_wrong_endpoint_is_not_accepted():
    assert not trajectory_reaches_state(trajectory(q6=.4),state(q6=.2))
    assert trajectory_reaches_state(trajectory(q6=.2),state(q6=.2))
