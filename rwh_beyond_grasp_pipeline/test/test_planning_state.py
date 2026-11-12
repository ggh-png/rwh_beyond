"""Small encoder boundary errors can be planned without changing measured feedback."""
import copy
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterValue
from rwh_beyond_grasp_pipeline.planning_state import arm_position_limits, bounded_planning_state
from rwh_beyond_grasp_pipeline.hardware_coordinator import HardwareGraspCoordinator
from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator


def description():
    return '<robot name="test">'+''.join(
        f'<joint name="joint{i}" type="revolute"><limit lower="{0 if i==2 else -1}" upper="3.14159"/></joint>'
        for i in range(1,7))+'</robot>'


def measured(joint2=-.00792379):
    return RobotState(joint_state=JointState(name=[f'joint{i}' for i in range(1,7)]+['gripper'],
        position=[.1,joint2,.1,.1,.1,.1,.07]),is_diff=True)


def test_reported_negative_joint2_is_corrected_only_in_planning_copy():
    state=measured();original=copy.deepcopy(state)
    planned,corrections=bounded_planning_state(state,arm_position_limits(description()),.01)
    assert planned.joint_state.position[1]==0.
    assert state==original
    assert planned.joint_state.position[0]==.1
    assert planned.joint_state.position[-1]==.07
    assert corrections==['joint2: -0.007924 -> 0.000000']


@pytest.mark.parametrize('value',[-.0101,float('nan'),float('inf')])
def test_large_or_nonfinite_boundary_error_is_rejected(value):
    with pytest.raises((RuntimeError,ValueError)):
        bounded_planning_state(measured(value),arm_position_limits(description()),.01)


def test_upper_boundary_is_also_corrected_and_larger_error_rejected():
    limits=arm_position_limits(description())
    planned,_=bounded_planning_state(measured(3.14659),limits,.01)
    assert planned.joint_state.position[1]==3.14159
    with pytest.raises(RuntimeError):bounded_planning_state(measured(3.16),limits,.01)


@pytest.mark.parametrize('tolerance',[0.,-.01,math.pi+.001,float('nan')])
def test_tolerance_cannot_disable_bound_checks(tolerance):
    with pytest.raises(ValueError):bounded_planning_state(measured(),arm_position_limits(description()),tolerance)


def test_model_limits_must_be_complete_and_finite():
    with pytest.raises(ValueError):arm_position_limits('<robot/>')
    with pytest.raises(ValueError):arm_position_limits(description().replace('upper="3.14159"','upper="nan"'))


def test_hardware_state_loads_moveit_limits_once_and_keeps_feedback_original(monkeypatch):
    state=measured()
    monkeypatch.setattr(GraspCoordinator,'state',lambda self:state)
    class Fake(HardwareGraspCoordinator):
        def __init__(self):
            self.planning_limits=None
            self.feedback=Mock()
            self.call=Mock(return_value=NS(values=[ParameterValue(string_value=description())]))
            self.get_parameter=Mock(return_value=NS(value=math.degrees(.01)))
            self.logger=NS(warning=Mock())
        def get_logger(self):return self.logger
    hardware=Fake()
    assert hardware.state().joint_state.position[1]==0.
    assert hardware.state().joint_state.position[1]==0.
    assert state.joint_state.position[1]==-.00792379
    hardware.call.assert_called_once()
    assert hardware.call.call_args.args[0]=='robot_description'


def test_reported_two_degree_zero_error_is_within_requested_tolerance():
    state=measured(-.038345)
    planned,corrections=bounded_planning_state(state,arm_position_limits(description()),.05)
    assert planned.joint_state.position[1]==0.
    assert state.joint_state.position[1]==-.038345
    assert corrections
    with pytest.raises(RuntimeError):
        bounded_planning_state(measured(-.0501),arm_position_limits(description()),.05)


def test_boundary_recovery_starts_at_actual_joint_and_limits_transition_speed():
    import numpy as np
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from rwh_beyond_grasp_pipeline.physical_trajectory import trajectory_points
    names=[f'joint{i}' for i in range(1,7)]
    current=dict(zip(names,[0.,-.038345,0.,0.,0.,0.]))
    trajectory=JointTrajectory(joint_names=names,points=[
        JointTrajectoryPoint(positions=[0.]*6),
        JointTrajectoryPoint(positions=[.1]*6,time_from_start=Duration(sec=1))])
    _,times,points=trajectory_points(trajectory,current,names,.05)
    np.testing.assert_allclose(points[0],[current[name] for name in names])
    assert points[1,1]==0.
    assert times[1]>=.5
    assert abs(points[1,1]-points[0,1])/times[1]<=.1
    assert times[-1]==times[1]+1.
    current['joint2']=-.0501
    with pytest.raises(ValueError):trajectory_points(trajectory,current,names,.05)


@pytest.mark.parametrize('angle_deg', [9.9, 10.0, 10.1])
def test_ten_degree_start_limit_is_shared_with_physical_transport(angle_deg):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from rwh_beyond_grasp_pipeline.physical_trajectory import trajectory_points
    tolerance = math.radians(10.)
    value = -math.radians(angle_deg)
    state = measured(value)
    original = copy.deepcopy(state)
    names = [f'joint{i}' for i in range(1,7)]
    current = dict(zip(state.joint_state.name, state.joint_state.position))
    trajectory = JointTrajectory(joint_names=names, points=[
        JointTrajectoryPoint(positions=[.1,0.,.1,.1,.1,.1]),
        JointTrajectoryPoint(positions=[.1]*6, time_from_start=Duration(sec=1))])
    if angle_deg > 10.:
        with pytest.raises(RuntimeError):
            bounded_planning_state(state, arm_position_limits(description()), tolerance)
        with pytest.raises(ValueError):
            trajectory_points(trajectory, current, names, tolerance)
    else:
        planned, _ = bounded_planning_state(state, arm_position_limits(description()), tolerance)
        assert planned.joint_state.position[1] == 0.
        _, times, points = trajectory_points(trajectory, current, names, tolerance)
        assert points[0,1] == value
        assert abs(points[1,1]-points[0,1])/times[1] <= .1 + 1e-12
    assert state == original
