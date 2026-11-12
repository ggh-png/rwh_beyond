"""Exercise configured candidate rejection thresholds without model or robot execution."""
import importlib
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionMatrix, CollisionObject
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from rwh_beyond_grasp_pipeline.selection_stages import SelectionStages

DEFAULTS = dict(grasp_top_down_max_angle_deg=35., grasp_table_clearance_m=.004,
                grasp_target_margin_m=.03)


def select(monkeypatch, overrides, angle=0., x=0., z=.2, legacy_table=False, octomap_enabled=True):
    module = importlib.import_module(SelectionStages.__module__)
    monkeypatch.setattr(module, 'observed_bounds', lambda *args:(np.zeros(3), np.ones(3), 0.))
    monkeypatch.setattr(module, 'Path', lambda *args:NS(write_text=lambda data:None))
    params = dict(DEFAULTS, model_to_tcp_rpy=[0.,0.,0.], model_to_tcp_xyz=[0.,0.,0.],
                  max_candidates=1, pregrasp_distance=.08, octomap_enabled=octomap_enabled, **{})
    params.update(overrides)
    matrix = AllowedCollisionMatrix()
    scene = NS(allowed_collision_matrix=matrix, world=NS(collision_objects=([NS(id='observed_support_table')] if legacy_table else []), octomap=NS(octomap=NS(data=[1]))))
    state = NS(attached=False, scene_active=False, base='base_link',
        get_parameter=lambda key:NS(value=params[key]), stage=Mock(), apply=Mock(),
        call=Mock(return_value=NS(scene=scene,success=True)),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=0)), cloud_seq=0, cloud_stamp=0.,
        wait=Mock(), ik=Mock(return_value=None), state=Mock())
    state.grasp_filter_settings = MethodType(SelectionStages.grasp_filter_settings, state)
    candidate_pose = PoseStamped()
    q = Rotation.from_euler('x', 180.-angle, degrees=True).as_quat()
    (candidate_pose.pose.orientation.x, candidate_pose.pose.orientation.y,
     candidate_pose.pose.orientation.z, candidate_pose.pose.orientation.w) = q
    candidate_pose.pose.position.x, candidate_pose.pose.position.z = x, z
    context = dict(frame=dict(depth=NS(header=Header()),info=None), xyz=np.zeros((1,3)),
                   mask=None, base_T_camera=np.eye(4), backend='sam3')
    with pytest.raises(RuntimeError, match='No feasible model grasp') as error:
        SelectionStages.select_feasible_grasp(state, context, [NS(pose=candidate_pose,score=1.)])
    objects = state.apply.call_args.args[0].world.collision_objects
    expected = [('red_grasp_target', CollisionObject.ADD)]
    if legacy_table:
        expected.append(('observed_support_table', CollisionObject.REMOVE))
    assert [(obj.id, obj.operation) for obj in objects] == expected
    if not octomap_enabled:
        assert not any(call.args[0] in ('region', 'exclude', 'clear')
                       for call in state.call.call_args_list)
        state.wait.assert_not_called()
    return str(error.value), state.ik.call_count


def test_angle_setting_changes_not_top_down_rejection(monkeypatch):
    error, calls = select(monkeypatch, {}, angle=40.)
    assert 'not_top_down' in error and calls == 0
    error, calls = select(monkeypatch, dict(grasp_top_down_max_angle_deg=45.), angle=40.)
    assert 'not_top_down' not in error and calls == 2


def test_table_setting_changes_clearance_rejection(monkeypatch):
    error, calls = select(monkeypatch, {}, z=.003)
    assert 'table_clearance' in error and calls == 0
    error, calls = select(monkeypatch, dict(grasp_table_clearance_m=.002), z=.003)
    assert 'table_clearance' not in error and calls == 2


def test_target_margin_changes_off_target_rejection(monkeypatch):
    error, calls = select(monkeypatch, dict(grasp_target_margin_m=.01), x=.52)
    assert 'off_target' in error and calls == 0
    error, calls = select(monkeypatch, {}, x=.52)
    assert 'off_target' not in error and calls == 2


@pytest.mark.parametrize('key,value', [
    ('grasp_top_down_max_angle_deg', -1.), ('grasp_top_down_max_angle_deg',181.),
    ('grasp_top_down_max_angle_deg',float('nan')), ('grasp_table_clearance_m',-.001),
    ('grasp_table_clearance_m',float('inf')), ('grasp_target_margin_m',-.01),
    ('grasp_target_margin_m',True), ('grasp_target_margin_m','0.03')])
def test_invalid_filter_settings_are_rejected(key, value):
    settings = {**DEFAULTS,key:value}
    state = NS(get_parameter=lambda name:NS(value=settings[name]))
    with pytest.raises(ValueError, match=key):
        SelectionStages.grasp_filter_settings(state)


def test_shared_yaml_exposes_valid_filter_settings():
    root = Path(__file__).resolve().parents[2]
    settings = yaml.safe_load((root/'rwh_beyond_bringup/config/bt_demo.yaml').read_text())
    params = settings['grasp_coordinator']['ros__parameters']
    assert SelectionStages.grasp_filter_settings(NS(get_parameter=lambda name:NS(value=params[name]))) == {
        key:params[key] for key in DEFAULTS}


@pytest.mark.parametrize('legacy_table', [False, True])
def test_selection_removes_only_existing_legacy_table(monkeypatch, legacy_table):
    select(monkeypatch, {}, legacy_table=legacy_table)


def test_octomap_disabled_skips_map_updates_and_waits_but_still_checks_ik(monkeypatch):
    _, calls = select(monkeypatch, {}, octomap_enabled=False)
    assert calls == 2
