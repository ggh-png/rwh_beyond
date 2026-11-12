"""Validate observation geometry and fail-closed planning without robot commands."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
import yaml

from rwh_beyond_grasp_pipeline.observation_motion import observation_pose, validate_observation_parameters
from rwh_beyond_grasp_pipeline.hardware_coordinator import HardwareGraspCoordinator
from rwh_beyond_grasp_pipeline.motion_stages import MotionStages

ROOT = Path(__file__).resolve().parents[2]


def test_rise_is_vertical_in_base_and_tilt_is_relative_to_home():
    home = Pose()
    home.position.x, home.position.y, home.position.z = .2, -.1, .3
    rotation = Rotation.from_euler('xyz', [20, 40, 60], degrees=True)
    home.orientation.x, home.orientation.y, home.orientation.z, home.orientation.w = rotation.as_quat()
    result = observation_pose(home, .3, [0, 30, 0])
    np.testing.assert_allclose([result.position.x, result.position.y, result.position.z], [.2, -.1, .6])
    q = result.orientation
    relative = rotation.inv() * Rotation.from_quat([q.x, q.y, q.z, q.w])
    np.testing.assert_allclose(relative.as_euler('xyz', degrees=True), [0, 30, 0], atol=1e-10)
    assert home.position.z == .3


@pytest.mark.parametrize('home,rise,tilt,settle,tolerance', [
    ([0.]*5, .3, [0.,30.,0.], .5, .01),
    ([0.]*6, float('nan'), [0.,30.,0.], .5, .01),
    ([0.]*6, .3, [0.,float('inf'),0.], .5, .01),
    ([0.]*6, .3, [0.,30.,0.], -.1, .01),
])
def test_invalid_observation_settings_are_rejected(home, rise, tilt, settle, tolerance):
    with pytest.raises(ValueError):
        validate_observation_parameters(home, rise, tilt, settle, tolerance)


def test_failed_home_plan_never_executes():
    state = NS(prepare_observation_motion=Mock(return_value=False), state=Mock(return_value=RobotState()),
        get_parameter=lambda name: NS(value=([0.]*6 if name=='home_joint_positions' else .01)),
        feedback=lambda: {f'joint{i}':.2 for i in range(1,7)},
        call=Mock(return_value=NS(motion_plan_response=NS(error_code=NS(val=-1)))),
        execute_trajectory=Mock())
    with pytest.raises(RuntimeError, match='Home planning failed'):
        HardwareGraspCoordinator.move_home(state)
    state.execute_trajectory.assert_not_called()


@pytest.mark.parametrize('overrides,expected_rise,expected_tilt', [
    ((), .3, 30.), ((.2, 50.), .2, 50.),
])
def test_observation_requires_home_and_requests_combined_cartesian_target(overrides, expected_rise, expected_tilt):
    params = dict(home_joint_positions=[0.]*6, home_joint_tolerance=.01,
                  observation_z_offset=.3, observation_tilt_rpy_deg=[0.,30.,0.],
                  observation_settle_time=.0)
    home_pose = Pose(); home_pose.orientation.w = 1.; home_pose.position.z = .2
    state = NS(prepare_observation_motion=Mock(return_value=False), base='base_link', tcp='tcp_link',
        feedback=lambda: {f'joint{i}':0. for i in range(1,7)},
        get_parameter=lambda name:NS(value=params[name]),
        state=lambda: RobotState(joint_state=JointState(name=[f'joint{i}' for i in range(1,7)],position=[0.]*6)),
        call=Mock(return_value=NS(error_code=NS(val=1),pose_stamped=[NS(pose=home_pose)])),
        execute_cartesian=Mock(),wait=Mock(),stage=Mock())
    HardwareGraspCoordinator.move_observation(state, *overrides)
    state.execute_cartesian.assert_called_once()
    target = state.execute_cartesian.call_args.args[0]
    assert target.position.z == pytest.approx(.2+expected_rise)
    assert target.orientation.y == pytest.approx(np.sin(np.deg2rad(expected_tilt/2)))
    state.feedback=lambda: {f'joint{i}':.1 for i in range(1,7)}
    state.execute_cartesian.reset_mock()
    with pytest.raises(RuntimeError, match='Move home'):
        HardwareGraspCoordinator.move_observation(state)
    state.execute_cartesian.assert_not_called()


def test_partial_cartesian_path_is_not_executed():
    state=NS(state=Mock(),cartesian=Mock(return_value=NS(fraction=.8,error_code=NS(val=1))),
             execute_trajectory=Mock())
    with pytest.raises(RuntimeError,match='Cartesian path rejected'):
        MotionStages.execute_cartesian(state,Pose())
    state.execute_trajectory.assert_not_called()


def test_shared_tree_orders_home_observation_before_segmentation():
    sequence=ET.parse(ROOT/'rwh_beyond_bt/behavior_trees/demo.xml').find('.//Sequence')
    steps=[node.attrib.get('command',node.tag) for node in sequence]
    assert steps[:7]==['CheckGraspReady','ClearOctomap','move_home','open_gripper',
                       'move_observation','ClearOctomap','SegmentObject']
    # Operators tune these values in XML; validate their contract, not a fixed preset.
    import math
    assert math.isfinite(float(sequence[4].attrib['rise_m']))
    assert float(sequence[4].attrib['rise_m']) > 0
    assert math.isfinite(float(sequence[4].attrib['tilt_deg']))


def test_shared_yaml_is_loaded_and_only_explicit_overrides_replace_it():
    path=ROOT/'rwh_beyond_grasp_pipeline/launch/grasp_pipeline.launch.py'
    spec=importlib.util.spec_from_file_location('pipeline_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    config=ROOT/'rwh_beyond_bringup/config/bt_demo.yaml'
    data=yaml.safe_load(config.read_text())
    assert set(data)=={'bt_navigator','bt_lifecycle_manager','sam3_bridge','graspgenx_bridge','grasp_coordinator'}
    context=LaunchContext()
    context.launch_configurations.update(dict(mode='real',use_sim_time='false',start_coordinator='true',
        params_file=str(config),sam3_params_file=str(config),graspgenx_params_file=str(config),
        segmentation_endpoint='',endpoint='',segmentation_backend='',target_text=''))
    nodes=module.pipeline_nodes(context)
    for node in nodes:
        params=evaluate_parameters(context,node._Node__parameters)
        assert params[0]==config
        assert params[-1]['use_sim_time'] is False
        assert 'endpoint' not in params[-1] and 'target_text' not in params[-1]
    context.launch_configurations.update(target_text='custom object',endpoint='tcp://custom:1234')
    nodes=module.pipeline_nodes(context)
    assert evaluate_parameters(context,nodes[1]._Node__parameters)[-1]['endpoint']=='tcp://custom:1234'
    assert evaluate_parameters(context,nodes[2]._Node__parameters)[-1]['target_text']=='custom object'


def test_home_already_reached_never_plans_or_sends_zero_time_motion():
    state = NS(prepare_observation_motion=Mock(return_value=False),
        get_parameter=lambda name:NS(value=([0.]*6 if name=='home_joint_positions' else .01)),
        feedback=lambda:{f'joint{i}':.005 for i in range(1,7)},
        call=Mock(), execute_trajectory=Mock(), stage=Mock())
    HardwareGraspCoordinator.move_home(state)
    state.call.assert_not_called()
    state.execute_trajectory.assert_not_called()
    state.stage.assert_called_once_with('home_already_reached_skip_motion')


def test_zero_time_home_plan_does_not_hide_unreached_home():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    trajectory=RobotTrajectory()
    trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6)]
    state=NS(prepare_observation_motion=Mock(return_value=False),state=Mock(return_value=RobotState()),
        get_parameter=lambda name:NS(value=([0.]*6 if name=='home_joint_positions' else .01)),
        feedback=lambda:{f'joint{i}':.2 for i in range(1,7)},
        call=Mock(return_value=NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=trajectory))),
        execute_trajectory=Mock())
    with pytest.raises(RuntimeError,match='no positive duration'):
        HardwareGraspCoordinator.move_home(state)
    state.execute_trajectory.assert_not_called()


def test_carried_target_keeps_scene_and_empty_gripper_still_requires_home():
    from rwh_beyond_grasp_pipeline.stage_constants import TARGET
    state=NS(attached=True,ensure_backend=Mock(),holding=Mock(return_value=True),
        call=Mock(return_value=NS(scene=NS(robot_state=NS(attached_collision_objects=[
            NS(object=NS(id=TARGET))])))),cleanup=Mock(),stage=Mock())
    assert HardwareGraspCoordinator.prepare_observation_motion(state) is True
    state.cleanup.assert_not_called()
    state.holding.return_value=False
    assert HardwareGraspCoordinator.prepare_observation_motion(state) is True
    state.holding.assert_not_called()
    state.cleanup.assert_not_called()
    state.call.return_value.scene.robot_state.attached_collision_objects=[]
    with pytest.raises(RuntimeError,match='missing'):
        HardwareGraspCoordinator.prepare_observation_motion(state)
    state.attached=False
    assert HardwareGraspCoordinator.prepare_observation_motion(state) is False
    state.cleanup.assert_called_once()


def test_unknown_payload_cannot_bypass_carry_checks():
    state=NS(attached=False,ensure_backend=Mock(),
        call=Mock(return_value=NS(scene=NS(robot_state=NS(attached_collision_objects=[
            NS(object=NS(id='unknown_payload'))])))),cleanup=Mock())
    with pytest.raises(RuntimeError,match='Unknown attached'):
        HardwareGraspCoordinator.prepare_observation_motion(state)
    state.cleanup.assert_not_called()


def test_carried_observation_returns_directly_to_home_relative_pose():
    params=dict(home_joint_positions=[0.]*6,home_joint_tolerance=.01,
        observation_z_offset=.1,observation_tilt_rpy_deg=[0.,40.,0.],observation_settle_time=0.)
    home_pose=Pose();home_pose.orientation.w=1.;home_pose.position.z=.2
    state=NS(prepare_observation_motion=Mock(return_value=True),base='base_link',tcp='tcp_link',
        feedback=lambda:{f'joint{i}':.4 for i in range(1,7)},
        get_parameter=lambda name:NS(value=params[name]),
        state=lambda:RobotState(joint_state=JointState(name=[f'joint{i}' for i in range(1,7)],position=[.4]*6)),
        call=Mock(return_value=NS(error_code=NS(val=1),pose_stamped=[NS(pose=home_pose)])),
        execute_cartesian=Mock(),wait=Mock(),stage=Mock())
    HardwareGraspCoordinator.move_observation(state,.1,40.)
    assert state.execute_cartesian.call_args.args[0].position.z==pytest.approx(.3)


def test_sim_and_real_use_shared_scenario_motion_and_xml():
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    for name in ('move_home', 'move_observation', 'prepare_observation_motion',
                 'open_gripper', 'command_gripper', 'attach', 'lift'):
        assert getattr(GraspCoordinator, name) is getattr(HardwareGraspCoordinator, name)
    trees = ROOT/'rwh_beyond_bt/behavior_trees'
    assert sorted(path.name for path in trees.iterdir()) == ['demo.xml']
    sequence = ET.parse(trees/'demo.xml').find('.//Sequence')
    assert [n.attrib.get('command', n.tag) for n in sequence][-4:] == [
        'AttachObject', 'LiftObject', 'move_observation', 'move_home']


@pytest.mark.parametrize('closing', [True, False])
def test_sim_gripper_routes_configured_width_and_duration(closing):
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    params = dict(gripper_close_width=.04, gripper_close_duration=4., simulation_only=True)
    handle = NS(accepted=True, get_result_async=Mock(return_value=NS(
        done=lambda: True, result=lambda: NS(status=4, result=NS(error_code=0)))),
        cancel_goal_async=Mock())
    sent = NS(done=lambda: True, result=lambda: handle)
    state = NS(ensure_backend=Mock(), wait=Mock(), motion_handle=None,
        get_parameter=lambda name: NS(value=params[name]),
        gripper_client=NS(server_is_ready=lambda: True, send_goal_async=Mock(return_value=sent)))
    GraspCoordinator.command_gripper(state, .04 if closing else .10)
    goal = state.gripper_client.send_goal_async.call_args.args[0]
    point = goal.trajectory.points[0]
    assert list(point.positions) == [.04 if closing else .10]
    assert point.time_from_start.sec == (4 if closing else 3)
    assert bool(goal.path_tolerance) == closing
    assert state.motion_handle is None
    handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('simulation', [True, False])
def test_shared_gripper_cancels_accepted_goal_after_result_timeout(simulation):
    from rwh_beyond_grasp_pipeline.gripper_motion import GripperMotion
    params = dict(simulation_only=simulation, gripper_close_width=.04, gripper_close_duration=4.)
    handle = NS(accepted=True, get_result_async=lambda: NS(done=lambda: False),
                cancel_goal_async=Mock())
    def wait(predicate, timeout):
        if not predicate():
            raise TimeoutError('result timeout')
    state = NS(ensure_backend=Mock(), wait=wait, motion_handle=None,
        get_parameter=lambda name: NS(value=params[name]),
        gripper_client=NS(server_is_ready=lambda: True, send_goal_async=lambda goal:
                          NS(done=lambda: True, result=lambda: handle)))
    with pytest.raises(TimeoutError, match='result timeout'):
        GripperMotion.command_gripper(state, .04)
    handle.cancel_goal_async.assert_called_once()
    assert state.motion_handle is None
