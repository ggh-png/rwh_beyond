"""Shared YAML timing reaches actual MoveIt requests without sending motion."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from rwh_beyond_grasp_pipeline.motion_parameters import DEFAULTS, validate_motion_parameters
from rwh_beyond_grasp_pipeline.motion_stages import MotionStages
from rwh_beyond_grasp_pipeline.scenario_motion import ScenarioMotion


@pytest.mark.parametrize('name,value', [
    ('moveit_velocity_scaling_factor',0),
    ('moveit_acceleration_scaling_factor',1.1),
    ('moveit_cartesian_velocity_scaling_factor',float('nan')),
    ('moveit_cartesian_acceleration_scaling_factor',True),
    ('moveit_planning_time',-1.),
    ('moveit_candidate_planning_time',float('inf')),
    ('moveit_ik_timeout',6.),
])
def test_invalid_settings_fail_before_a_request(name,value):
    with pytest.raises(ValueError,match=name):validate_motion_parameters({name:value})


def config(**overrides):return validate_motion_parameters({**DEFAULTS,**overrides})


def test_cartesian_uses_its_own_velocity_and_acceleration_settings():
    state=NS(base='base_link',tcp='tcp_link',call=Mock(),motion_parameters=config(
        moveit_velocity_scaling_factor=.3,moveit_acceleration_scaling_factor=.25,
        moveit_cartesian_velocity_scaling_factor=.2,moveit_cartesian_acceleration_scaling_factor=.15))
    MotionStages.cartesian(state,Pose(),RobotState())
    request=state.call.call_args.args[1]
    assert request.max_velocity_scaling_factor==.2
    assert request.max_acceleration_scaling_factor==.15
    assert request.avoid_collisions and request.max_step==.004
    assert request.revolute_jump_threshold==.35


def test_ik_timeout_reaches_ik_request():
    state=NS(base='base_link',tcp='tcp_link',motion_parameters=config(moveit_ik_timeout=.25),
             call=Mock(return_value=NS(error_code=NS(val=1),solution=RobotState())))
    MotionStages.ik(state,Pose(),RobotState())
    timeout=state.call.call_args.args[1].ik_request.timeout
    assert timeout.sec==0 and timeout.nanosec==250000000


def test_home_plan_uses_shared_scaling_and_matching_service_timeout():
    trajectory=NS(joint_trajectory=NS(points=[NS(time_from_start=NS(sec=2,nanosec=0))]))
    state=NS(motion_parameters=config(moveit_velocity_scaling_factor=.3,
        moveit_acceleration_scaling_factor=.2,moveit_planning_time=25.),
        prepare_observation_motion=Mock(),get_parameter=lambda name:NS(value=.01),
        feedback=lambda:{f'joint{i}':.2 for i in range(1,7)},state=lambda:RobotState(),
        call=Mock(return_value=NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=trajectory))),
        execute_trajectory=Mock(),wait=Mock())
    ScenarioMotion.move_home(state,home=[0.]*6)
    request=state.call.call_args.args[1].motion_plan_request
    assert request.max_velocity_scaling_factor==.3
    assert request.max_acceleration_scaling_factor==.2
    assert request.allowed_planning_time==25.
    assert state.call.call_args.args[2]==30.
