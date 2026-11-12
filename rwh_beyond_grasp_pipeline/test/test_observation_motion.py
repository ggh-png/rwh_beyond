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
from moveit_msgs.msg import RobotState, CollisionObject
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
    steps=[node.tag for node in sequence.iter() if node.tag != 'Sequence']
    assert steps[:7]==['CheckGraspReady','ClearOctomap','MoveHome','OpenGripper',
                       'MoveObservation','ClearOctomap','SegmentObject']
    # Operators tune these values in XML; validate their contract, not a fixed preset.
    import math
    observation = sequence.find('.//MoveObservation')
    assert all(math.isfinite(float(observation.attrib[axis])) for axis in ('x','y','z','tilt_deg'))


def test_shared_yaml_is_loaded_and_only_explicit_overrides_replace_it():
    path=ROOT/'rwh_beyond_grasp_pipeline/launch/grasp_pipeline.launch.py'
    spec=importlib.util.spec_from_file_location('pipeline_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    config=ROOT/'rwh_beyond_bringup/config/bt_demo.yaml'
    data=yaml.safe_load(config.read_text())
    assert set(data)=={'/**','bt_navigator','bt_lifecycle_manager','sam3_bridge','graspgenx_bridge','grasp_coordinator'}
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
    state=NS(attached=True,ensure_backend=Mock(),apply=Mock(),holding=Mock(return_value=True),
        call=Mock(return_value=NS(scene=NS(world=NS(collision_objects=[NS(id='observed_support_table')]), robot_state=NS(attached_collision_objects=[
            NS(object=NS(id=TARGET))])))),cleanup=Mock(),stage=Mock())
    assert HardwareGraspCoordinator.prepare_observation_motion(state) is True
    removal = state.apply.call_args.args[0]
    assert len(removal.world.collision_objects) == 1
    assert removal.world.collision_objects[0].id == 'observed_support_table'
    assert removal.world.collision_objects[0].operation == CollisionObject.REMOVE
    assert not removal.robot_state.attached_collision_objects
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
    state=NS(attached=False,ensure_backend=Mock(),apply=Mock(),
        call=Mock(return_value=NS(scene=NS(world=NS(collision_objects=[NS(id='observed_support_table')]), robot_state=NS(attached_collision_objects=[
            NS(object=NS(id='unknown_payload'))])))),cleanup=Mock())
    with pytest.raises(RuntimeError,match='Unknown attached'):
        HardwareGraspCoordinator.prepare_observation_motion(state)
    state.cleanup.assert_not_called()


@pytest.mark.parametrize('xyz', [[0.,0.,.1], [.03,-.02,.1], [-.03,.02,0.], [0.,0.,-.05]])
def test_carried_observation_returns_directly_to_home_relative_pose(xyz):
    params=dict(home_joint_positions=[0.]*6,home_joint_tolerance=.01,
        observation_z_offset=.1,observation_tilt_rpy_deg=[0.,40.,0.],observation_settle_time=0.)
    home_pose=Pose();home_pose.orientation.w=1.;home_pose.position.z=.2
    state=NS(prepare_observation_motion=Mock(return_value=True),base='base_link',tcp='tcp_link',
        feedback=lambda:{f'joint{i}':.4 for i in range(1,7)},
        get_parameter=lambda name:NS(value=params[name]),
        state=lambda:RobotState(joint_state=JointState(name=[f'joint{i}' for i in range(1,7)],position=[.4]*6)),
        call=Mock(return_value=NS(error_code=NS(val=1),pose_stamped=[NS(pose=home_pose)])),
        execute_cartesian=Mock(),wait=Mock(),stage=Mock())
    HardwareGraspCoordinator.move_observation(state,tilt_deg=40.,x=xyz[0],y=xyz[1],z=xyz[2])
    pose = state.execute_cartesian.call_args.args[0]
    np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z],
                              [xyz[0], xyz[1], .2+xyz[2]])


def test_sim_and_real_use_shared_scenario_motion_and_xml():
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    for name in ('move_home', 'move_observation', 'prepare_observation_motion',
                 'move_place', 'release_object', 'open_gripper', 'command_gripper', 'attach', 'lift'):
        assert getattr(GraspCoordinator, name) is getattr(HardwareGraspCoordinator, name)
    trees = ROOT/'rwh_beyond_bt/behavior_trees'
    assert sorted(path.name for path in trees.glob('*.xml')
                  if ET.parse(path).find('BehaviorTree') is not None) == ['demo.xml', 'place.xml']
    sequence = ET.parse(trees/'demo.xml').find('.//Sequence')
    assert [n.tag for n in sequence.iter() if n.tag != 'Sequence'][-4:] == [
        'AttachObject', 'LiftObject', 'MoveObservation', 'MoveHome']
    place = ET.parse(trees/'place.xml').find('.//Sequence')
    assert [n.tag for n in place] == ['MoveHome', 'MovePlace', 'ReleaseObject', 'MoveHome']


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


@pytest.mark.parametrize(
    'command', ['move_home', 'open_gripper', 'move_observation', 'move_place', 'release_object']
)
def test_scenario_endpoint_fixes_command_and_preserves_motion_settings(command):
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    from rwh_beyond_msgs.action import RunGraspStep
    request = RunGraspStep.Goal(command='attach', override_observation_z=True, override_observation_tilt=True,
                 observation_z_offset=.1, observation_tilt_deg=40.)
    handle = NS(request=request, is_cancel_requested=False,
                succeed=Mock(), abort=Mock(), canceled=Mock())
    state = NS(scene_lock=Mock(), lock=Mock(), stage=Mock(), ensure_backend=Mock(),
               move_home=Mock(), move_observation=Mock(), move_place=Mock(),
               open_gripper=Mock(), release_object=Mock(),
               cleanup_requested=False, cleanup=Mock(), get_logger=Mock())
    result = GraspCoordinator.execute(state, handle, command)
    assert result.success
    handle.succeed.assert_called_once()
    if command == 'move_observation':
        state.move_observation.assert_called_once_with(None, 40., x=None, y=None, z=.1, home=None, settle_time=None)
    elif command == 'move_home':
        state.move_home.assert_called_once_with(home=None)
    elif command == 'move_place':
        state.move_place.assert_called_once_with(.20, .10, home=None)
    elif command == 'release_object':
        state.release_object.assert_called_once_with(opening=None, duration=None)
    else:
        state.open_gripper.assert_called_once_with(opening=None, duration=None)
    state.cleanup.assert_not_called()
    state.scene_lock.release.assert_called_once()
    state.lock.release.assert_called_once()


@pytest.mark.parametrize('failure', ['abort', 'cancel'])
def test_scenario_failure_or_cancel_cleans_up_and_releases_locks(failure):
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    from rwh_beyond_grasp_pipeline.stage_errors import Canceled
    error = Canceled('canceled') if failure == 'cancel' else RuntimeError('motion failed')
    from rwh_beyond_msgs.action import RunGraspStep
    handle = NS(request=RunGraspStep.Goal(), is_cancel_requested=False,
                succeed=Mock(), abort=Mock(), canceled=Mock())
    state = NS(scene_lock=Mock(), lock=Mock(), stage=Mock(), ensure_backend=Mock(),
               move_home=Mock(side_effect=error), cleanup_requested=False,
               cleanup=Mock(), get_logger=Mock())
    result = GraspCoordinator.execute(state, handle, 'move_home')
    assert not result.success
    (handle.canceled if failure == 'cancel' else handle.abort).assert_called_once()
    state.cleanup.assert_called_once()
    state.scene_lock.release.assert_called_once()
    state.lock.release.assert_called_once()
    assert state.active_handle is None


@pytest.mark.parametrize(
    'command', ['move_home', 'move_observation', 'move_place', 'open_gripper', 'release_object']
)
def test_scenario_xml_overrides_reach_motion_without_changing_yaml(command):
    from rwh_beyond_msgs.action import RunGraspStep
    from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator
    home = [.1, .2, .3, .4, .5, .6]
    request = RunGraspStep.Goal(home_joint_positions=home,
        override_observation_z=True, override_observation_tilt=True, observation_z_offset=.15, observation_tilt_deg=35.,
        override_observation_x=True, observation_x_offset=.03,
        override_observation_y=True, observation_y_offset=-.02,
        override_settle_time=True, observation_settle_time=.2,
        override_gripper_width=True, gripper_open_width=.08,
        override_gripper_duration=True, gripper_open_duration=2.)
    handle = NS(request=request, is_cancel_requested=False,
                succeed=Mock(), abort=Mock(), canceled=Mock())
    state = NS(scene_lock=Mock(), lock=Mock(), stage=Mock(), ensure_backend=Mock(),
               move_home=Mock(), move_observation=Mock(), move_place=Mock(),
               open_gripper=Mock(), release_object=Mock(),
               cleanup_requested=False, cleanup=Mock(), get_logger=Mock())
    assert GraspCoordinator.execute(state, handle, command).success
    if command == 'move_home':
        state.move_home.assert_called_once_with(home=home)
    elif command == 'move_observation':
        state.move_observation.assert_called_once_with(None, 35., x=.03, y=-.02, z=.15, home=home, settle_time=.2)
    elif command == 'move_place':
        state.move_place.assert_called_once_with(.03, .15, home=home)
    elif command == 'release_object':
        state.release_object.assert_called_once_with(opening=.08, duration=2.)
    else:
        state.open_gripper.assert_called_once_with(opening=.08, duration=2.)


def test_place_moves_forward_in_home_tcp_frame_then_releases_attachment():
    from rwh_beyond_grasp_pipeline.stage_constants import TARGET

    home_pose = Pose()
    home_pose.position.x, home_pose.position.y, home_pose.position.z = .3, -.1, .2
    home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w = (
        Rotation.from_euler('z', 90, degrees=True).as_quat()
    )
    params = dict(home_joint_positions=[0.]*6, home_joint_tolerance=.01, gripper_open_width=.10)
    state = NS(
        attached=True, release_ready=False, base='base_link', tcp='tcp_link',
        prepare_observation_motion=Mock(return_value=True),
        feedback=lambda: {f'joint{i}': 0. for i in range(1, 7)} | {'gripper': .10},
        get_parameter=lambda name: NS(value=params[name]),
        state=lambda: RobotState(joint_state=JointState(
            name=[f'joint{i}' for i in range(1, 7)], position=[0.]*6)),
        call=Mock(return_value=NS(error_code=NS(val=1), pose_stamped=[NS(pose=home_pose)])),
        execute_cartesian=Mock(), stage=Mock(), command_gripper=Mock(), wait=Mock(),
        apply=Mock(), cleanup=Mock(),
    )
    HardwareGraspCoordinator.move_place(state, .20, .15)
    target = state.execute_cartesian.call_args.args[0]
    np.testing.assert_allclose(
        [target.position.x, target.position.y, target.position.z], [.3, .1, .35], atol=1e-10
    )
    assert state.release_ready
    HardwareGraspCoordinator.release_object(state)
    detached = state.apply.call_args.args[0].robot_state.attached_collision_objects[0]
    assert detached.object.id == TARGET
    assert detached.object.operation == CollisionObject.REMOVE
    assert not state.attached
    assert not state.release_ready
    state.cleanup.assert_called_once()


def test_release_requires_successful_place_motion():
    state = NS(attached=True, release_ready=False, command_gripper=Mock())
    with pytest.raises(RuntimeError, match='successful place motion'):
        HardwareGraspCoordinator.release_object(state)
    state.command_gripper.assert_not_called()


def test_open_override_sends_width_and_duration_and_keeps_attachment_guard():
    state = NS(attached=False, command_gripper=Mock(), wait=Mock(), feedback=lambda:{'gripper':.08})
    HardwareGraspCoordinator.open_gripper(state, opening=.08, duration=1.5)
    state.command_gripper.assert_called_once_with(.08, duration=1.5, closing=False)
    assert state.wait.call_args.args[0]()
    state.attached = True
    with pytest.raises(RuntimeError, match='automatic release'):
        HardwareGraspCoordinator.open_gripper(state, opening=.08, duration=1.5)
    assert state.command_gripper.call_count == 1


@pytest.mark.parametrize('home', [[0.] * 5, [0.] * 5 + [float('nan')]])
def test_invalid_xml_home_is_rejected_before_motion_preparation(home):
    state = NS(prepare_observation_motion=Mock())
    with pytest.raises(ValueError, match='six finite'):
        HardwareGraspCoordinator.move_home(state, home=home)
    state.prepare_observation_motion.assert_not_called()


@pytest.mark.parametrize('xyz', [[.03, -.02, .1], [-.03, .02, 0.], [0., 0., -.05]])
def test_xyz_offsets_are_base_aligned_and_do_not_accumulate(xyz):
    home = Pose()
    home.position.x, home.position.y, home.position.z = .2, .1, .4
    q = Rotation.from_euler('z', 90, degrees=True).as_quat()
    home.orientation.x, home.orientation.y, home.orientation.z, home.orientation.w = q
    validate_observation_parameters([0.]*6, xyz[2], [0.,30.,0.], 0., .01, offset_xyz=xyz)
    for _ in range(2):
        target = observation_pose(home, xyz[2], [0.,30.,0.], xyz[0], xyz[1])
        np.testing.assert_allclose([target.position.x, target.position.y, target.position.z],
                                  np.array([.2, .1, .4]) + xyz)
        result_q = target.orientation
        relative = Rotation.from_quat(q).inv() * Rotation.from_quat(
            [result_q.x, result_q.y, result_q.z, result_q.w])
        np.testing.assert_allclose(relative.as_euler('xyz', degrees=True), [0.,30.,0.], atol=1e-10)
    assert home.position.z == .4


def test_nonfinite_xyz_is_rejected_before_preparing_motion():
    state = NS(get_parameter=lambda name:NS(value={
        'observation_z_offset':.1, 'observation_tilt_rpy_deg':[0.,40.,0.],
        'home_joint_positions':[0.]*6, 'observation_settle_time':0., 'home_joint_tolerance':.01}[name]),
        prepare_observation_motion=Mock())
    with pytest.raises(ValueError, match='three finite'):
        HardwareGraspCoordinator.move_observation(state, x=float('nan'))
    state.prepare_observation_motion.assert_not_called()


@pytest.mark.parametrize('legacy_table', [False, True])
def test_home_preparation_removes_only_existing_legacy_table(legacy_table):
    scene = NS(world=NS(collision_objects=(
        [NS(id='observed_support_table')] if legacy_table else [])),
        robot_state=NS(attached_collision_objects=[]))
    state = NS(attached=False, ensure_backend=Mock(), apply=Mock(),
               call=Mock(return_value=NS(scene=scene)), cleanup=Mock())
    assert HardwareGraspCoordinator.prepare_observation_motion(state) is False
    if legacy_table:
        removal = state.apply.call_args.args[0].world.collision_objects
        assert [(obj.id, obj.operation) for obj in removal] == [
            ('observed_support_table', CollisionObject.REMOVE)]
    else:
        state.apply.assert_not_called()
    state.cleanup.assert_called_once()
