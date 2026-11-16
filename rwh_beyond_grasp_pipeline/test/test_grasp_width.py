"""Width BT geometry and command binding; no nodes, cameras or hardware."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from rwh_beyond_grasp_pipeline.grasp_width import estimate_aperture
from rwh_beyond_grasp_pipeline.scenario_motion import ScenarioMotion


def cloud(width=.05):
    x = np.linspace(-width/2, width/2, 200)
    return np.column_stack([x, np.zeros(200), np.full(200, .105)])


def test_width_uses_grasp_closing_axis_and_metres():
    transform = np.eye(4)
    transform[:3,:3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    transform[:3,3] = [.2, -.1, .3]
    points = cloud() @ transform[:3,:3].T + transform[:3,3]
    width, target = estimate_aperture(points, transform, .1)
    assert width == pytest.approx(.048)
    assert target == pytest.approx(width-.002)


@pytest.mark.parametrize('points', [cloud(.001), cloud()[:50], cloud()+[.2,0,0], np.full((200,3),np.nan)])
def test_bad_geometry_never_falls_back_to_fixed_width(points):
    with pytest.raises(ValueError): estimate_aperture(points, np.eye(4), .1)


def test_attach_uses_plan_bound_width_and_explicit_close_marker():
    state = NS(prepared=True, selected_gripper_width=.032, command_gripper=Mock(),
               get_parameter=lambda name:NS(value=.1), stage=Mock(), object=Mock(), tcp='tcp_link')
    # Abort immediately after the close to avoid inventing attachment geometry.
    state.command_gripper.side_effect = RuntimeError('close rejected')
    with pytest.raises(RuntimeError, match='close rejected'):
        ScenarioMotion.attach(state)
    state.command_gripper.assert_called_once_with(.032, closing=True)


def test_width_tree_is_separate_and_enables_width_before_approach():
    trees=Path(__file__).resolve().parents[2]/'rwh_beyond_bt/behavior_trees'
    width=ET.parse(trees/'demo_width.xml').find('.//SelectFeasibleGrasp')
    fixed=ET.parse(trees/'demo.xml').find('.//SelectFeasibleGrasp')
    assert width.get('use_estimated_width')=='true'
    assert width.get('grasp_width')=='{grasp_width}'
    assert width.get('close_width')=='{close_width}'
    assert fixed.get('use_estimated_width') is None


def test_variable_width_close_preserves_duration_and_sets_transport_marker():
    from rwh_beyond_grasp_pipeline.gripper_motion import GripperMotion
    params = dict(gripper_close_width=.04, gripper_close_duration=4., simulation_only=False)
    handle=NS(accepted=True, get_result_async=lambda:NS(done=lambda:True,
        result=lambda:NS(status=4,result=NS(error_code=0))))
    send=Mock(return_value=NS(done=lambda:True,result=lambda:handle))
    state=NS(ensure_backend=Mock(),wait=lambda predicate,timeout:predicate(),
             get_parameter=lambda name:NS(value=params[name]),motion_handle=None,
             gripper_client=NS(server_is_ready=lambda:True,send_goal_async=send))
    GripperMotion.command_gripper(state,.032,closing=True)
    goal=send.call_args.args[0]
    assert goal.trajectory.points[0].positions[0]==.032
    assert goal.trajectory.points[0].time_from_start.sec==4
    assert goal.path_tolerance[0].position==-1.


def repeat_pick_state(attached=True, present=True):
    from rwh_beyond_grasp_pipeline.stage_constants import TARGET
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    events=[]
    objects=[AttachedCollisionObject(object=CollisionObject(id=TARGET))] if present else []
    state=NS(attached=attached,release_ready=True,selected_gripper_width=.032,
        selected_observed_width=.034,scene_active=True,stage=Mock(),
        call=Mock(return_value=NS(scene=NS(robot_state=NS(attached_collision_objects=objects)))),
        get_parameter=lambda name:NS(value=.1),
        command_gripper=lambda opening,**kwargs:events.append('open'),
        feedback=lambda:{'gripper':.1},
        wait=lambda predicate,timeout:events.append('confirmed') if predicate() else None,
        apply=lambda scene:events.append('detach'),cleanup=lambda:events.append('cleanup'))
    return state,events


def test_repeat_pick_opens_and_confirms_before_clearing_attachment():
    state,events=repeat_pick_state()
    ScenarioMotion.release_previous_target(state)
    assert events==['open','confirmed','detach','cleanup']
    assert not state.attached and not state.release_ready
    assert state.selected_gripper_width is None


def test_repeat_pick_open_failure_keeps_attachment_and_stops():
    state,events=repeat_pick_state()
    def failed(opening,**kwargs):raise RuntimeError('opening failed')
    state.command_gripper=failed
    with pytest.raises(RuntimeError,match='opening failed'):
        ScenarioMotion.release_previous_target(state)
    assert state.attached and events==[]


def test_repeat_pick_feedback_failure_keeps_attachment_and_stops():
    state,events=repeat_pick_state()
    def timeout(predicate,seconds):raise TimeoutError('not opened')
    state.wait=timeout
    with pytest.raises(TimeoutError):ScenarioMotion.release_previous_target(state)
    assert state.attached and events==['open']


def test_repeat_pick_handles_retained_scene_after_coordinator_restart():
    state,events=repeat_pick_state(attached=False,present=True)
    ScenarioMotion.release_previous_target(state)
    assert events==['open','confirmed','detach','cleanup']
    assert not state.attached


def test_repeat_pick_does_nothing_without_retained_payload():
    state,events=repeat_pick_state(attached=False,present=False)
    ScenarioMotion.release_previous_target(state)
    assert not events


def test_width_tree_explicitly_releases_previous_target_and_fixed_tree_does_not():
    trees=Path(__file__).resolve().parents[2]/'rwh_beyond_bt/behavior_trees'
    assert ET.parse(trees/'demo_width.xml').find('.//CheckGraspReady').get('release_previous')=='true'
    assert ET.parse(trees/'demo.xml').find('.//CheckGraspReady').get('release_previous') is None


@pytest.mark.parametrize('maximum', [.10,.08])
def test_large_estimate_is_retained_but_command_is_capped(maximum):
    observed,target=estimate_aperture(cloud(.15),np.eye(4),maximum)
    assert observed==pytest.approx(.144)
    assert target==maximum


def test_attach_accepts_equal_maximum_and_clamps_oversized_plan():
    for requested,maximum,expected in [(.10,.10,.10),(.15,.10,.10),(.12,.08,.08)]:
        state=NS(prepared=True,selected_gripper_width=requested,
            get_parameter=lambda name:NS(value=maximum),stage=Mock(),command_gripper=Mock())
        state.command_gripper.side_effect=RuntimeError('stop before attachment')
        with pytest.raises(RuntimeError,match='stop before attachment'):
            ScenarioMotion.attach(state)
        state.command_gripper.assert_called_once_with(expected,closing=True)


def test_final_command_transport_caps_aperture_at_physical_limit():
    from rwh_beyond_grasp_pipeline.gripper_motion import GripperMotion
    params=dict(gripper_close_width=.04,gripper_close_duration=4.,simulation_only=False)
    handle=NS(accepted=True,get_result_async=lambda:NS(done=lambda:True,
        result=lambda:NS(status=4,result=NS(error_code=0))))
    send=Mock(return_value=NS(done=lambda:True,result=lambda:handle))
    state=NS(ensure_backend=Mock(),wait=lambda predicate,timeout:predicate(),
        get_parameter=lambda name:NS(value=params[name]),motion_handle=None,
        gripper_client=NS(server_is_ready=lambda:True,send_goal_async=send))
    GripperMotion.command_gripper(state,.15,closing=True)
    assert send.call_args.args[0].trajectory.points[0].positions[0]==.10
