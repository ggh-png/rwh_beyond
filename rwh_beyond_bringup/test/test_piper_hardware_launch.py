"""Hardware planning parity without starting a driver or sending robot commands."""
import math
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from launch_ros.parameter_descriptions import ParameterValue
from rwh_beyond_bringup.piper_hardware import (
    DEFAULT_TCP, controller_parameters, hardware_nodes, moveit_parameters,
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(module):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


@pytest.mark.parametrize('octomap', (True, False))
def test_beyond_backends_share_planning_parameters(tmp_path, octomap):
    import yaml

    share = Path(get_package_share_directory('rwh_beyond_bringup'))
    launcher = load(share / 'launch/piper_moveit.launch.py', 'parity_piper')
    simulator = load(Path(get_package_share_directory('moma_bringup')) /
                     'launch/simulation.launch.py', 'parity_sim')
    settings = yaml.safe_load((share / 'config/bt_demo.yaml').read_text())
    settings['/**']['ros__parameters']['octomap_enabled'] = octomap
    settings['bt_navigator']['ros__parameters']['bt_motion']['gripper_close_duration'] = 2.75
    execution = settings['move_group']['ros__parameters']['trajectory_execution']
    execution['allowed_goal_duration_margin'] = 21.0
    path = tmp_path / 'shared.yaml'
    path.write_text(yaml.safe_dump(settings))
    context = defaults(launcher)
    context.launch_configurations.update(mode='sim', gui='false', use_rviz='false',
                                         params_file=str(path), speed_percent='70')
    actions = launcher.piper_nodes(context)
    include = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    launch_arguments = dict(include.launch_arguments)
    baseline = Path(launch_arguments['moveit_baseline_file'])
    try:
        sim_context = defaults(simulator)
        sim_context.launch_configurations.update(launch_arguments)
        nodes = simulator.nodes(sim_context)
        move_group = next(node for node in nodes if node.node_executable == 'move_group')
        actual = evaluate_parameters(sim_context, move_group._Node__parameters)[0]
        bridge = next(node for node in nodes if node.node_executable == 'bridge_node')
        controller = evaluate_parameters(sim_context, bridge._Node__parameters)[0]
    finally:
        baseline.unlink(missing_ok=True)

    real = moveit_parameters(DEFAULT_TCP, camera_model=True, params_file=str(path),
                             speed_percent=70)
    for key in ('robot_description', 'robot_description_semantic'):
        real[key] = ParameterValue(real[key], value_type=str)
    expected = evaluate_parameters(sim_context, normalize_parameters([real]))[0]
    assert actual.pop('use_sim_time') is True
    assert expected.pop('use_sim_time') is False
    assert actual == expected
    assert all(actual[f'robot_description_planning.joint_limits.joint{i}.max_velocity']
               == pytest.approx(2.1) for i in range(1, 7))
    assert actual['trajectory_execution.allowed_goal_duration_margin'] == 21.0
    assert ('sensors' in actual) is octomap
    assert actual['depth_cloud.point_cloud_topic'] == '/camera/depth/color/points'
    assert controller['arm_velocity_limits'] == tuple(
        expected[f'robot_description_planning.joint_limits.joint{i}.max_velocity']
        for i in range(1, 7))
    assert controller['arm_start_tolerance'] == expected['trajectory_execution.allowed_start_tolerance']
    assert controller['camera_rate'] == 5.0
    assert controller['gripper_close_width'] == .04
    assert controller['gripper_close_duration'] == 2.75


def test_camera_mount_is_not_assumed_and_robot_bounds_are_preserved():
    real = moveit_parameters(DEFAULT_TCP)
    sim = moveit_parameters(DEFAULT_TCP, camera_model=True)
    urdf = ET.fromstring(real['robot_description'])
    assert urdf.find("link[@name='d405_link']") is None
    assert urdf.find("joint[@name='tcp_joint']/origin").get('xyz') == '0.0 0.0 0.1425'
    bounds = lambda text: {j.get('name'): j.find('limit').attrib for j in
                           ET.fromstring(text).findall('joint') if j.find('limit') is not None}
    assert bounds(real['robot_description']) == bounds(sim['robot_description'])
    assert real['robot_description_planning'] == sim['robot_description_planning']
    assert real['robot_description_kinematics'] == sim['robot_description_kinematics']


def test_hardware_transport_keeps_closed_gate_and_real_feedback():
    module = load(Path(__file__).resolve().parents[1] /
                  'launch/piper_moveit.launch.py', 'parity_hardware')
    context = defaults(module)
    assert context.launch_configurations['auto_enable'] == 'false'
    driver, group, *sensors = hardware_nodes(context)
    assert len(sensors) == 1
    assert isinstance(driver, IncludeLaunchDescription)
    assert isinstance(group, GroupAction)
    args = dict(driver.launch_arguments)
    assert args['control_enabled'] == 'false'
    assert args['tcp_offset'].perform(context) == DEFAULT_TCP
    nodes = [n for n in group.get_sub_entities() if hasattr(n, 'node_package')]
    assert all(n.node_package != 'moma_mujoco_ros' for n in nodes)
    move_group = next(n for n in nodes if n.node_executable == 'move_group')
    params = evaluate_parameters(context, move_group._Node__parameters)[0]
    assert params['use_sim_time'] is False
    assert params['planning_pipelines'] == ('ompl',)
    assert params['depth_cloud.padding_offset'] == .02


@pytest.mark.parametrize('offset', ['[0,0,0]', '[0,0,1e999,0,0,0]', '123'])
def test_invalid_tcp_rejected_before_hardware_launch(offset):
    with pytest.raises(ValueError):
        moveit_parameters(offset)


def test_hardware_controller_names_and_groups_match_simulator():
    params, names = controller_parameters('arm1', 'agx_gripper')
    assert names == ['arm_controller', 'gripper_controller']
    assert params['/arm1/arm_controller']['ros__parameters']['joints'] == [
        f'joint{i}' for i in range(1, 7)]
    assert params['/arm1/gripper_controller']['ros__parameters']['joints'] == ['gripper']


def test_feedback_transport_replaces_mock_and_external_gate():
    module = load(Path(__file__).resolve().parents[1] /
                  'launch/piper_moveit.launch.py', 'feedback_hardware')
    context = defaults(module)
    context.launch_configurations['execution_transport'] = 'feedback'
    driver, group, *_ = hardware_nodes(context)
    assert dict(driver.launch_arguments)['control_enabled'] == 'false'
    nodes = [n for n in group.get_sub_entities() if hasattr(n, 'node_package')]
    executables = [n.node_executable for n in nodes]
    assert 'physical_trajectory' in executables
    assert 'target_cloud_filter' in executables
    assert 'ros2_control_node' not in executables
    assert 'agx_arm_control_gate' not in executables
    move_group = next(n for n in nodes if n.node_executable == 'move_group')
    params = evaluate_parameters(context, move_group._Node__parameters)[0]
    assert params['depth_cloud.point_cloud_topic'] == '/camera/depth/planning_points'


def test_camera_fixed_mount_exception_does_not_disable_finger_or_environment_checks():
    params = moveit_parameters(DEFAULT_TCP, camera_model=True)
    urdf = ET.fromstring(params['robot_description'])
    srdf = ET.fromstring(params['robot_description_semantic'])
    # Both bodies belong to the same rigid wrist component, not to moving fingers.
    parents = {joint.find('child').get('link'):joint for joint in urdf.findall('joint')}
    for link in ('d405_link','gripper_base'):
        while link != 'link6':
            joint = parents[link]
            assert joint.get('type') == 'fixed'
            link = joint.find('parent').get('link')
    camera_pairs = {frozenset((entry.get('link1'), entry.get('link2')))
                    for entry in srdf.findall('disable_collisions')
                    if 'd405_link' in (entry.get('link1'),entry.get('link2'))}
    assert camera_pairs == {frozenset(('d405_link','link6')),
                            frozenset(('d405_link','gripper_base'))}
    assert urdf.find("link[@name='d405_link']/collision/geometry/box") is not None
    # Requested camera translation is unchanged by the contact exception.
    assert urdf.find("joint[@name='d405_mount_joint']/origin").get('xyz') == '-0.06 0.0 0.065'


def test_execution_and_physical_start_tolerance_share_coordinator_setting():
    import yaml
    settings=yaml.safe_load((Path(get_package_share_directory('rwh_beyond_bringup')) /
        'config/bt_demo.yaml').read_text())
    expected=math.radians(settings['bt_navigator']['ros__parameters']['bt_motion']['planning_start_bounds_tolerance_deg'])
    module=load(Path(__file__).resolve().parents[1]/'launch/piper_moveit.launch.py','tolerance_hardware')
    context=defaults(module);context.launch_configurations['execution_transport']='feedback'
    _,group,*_=hardware_nodes(context)
    nodes=[node for node in group.get_sub_entities() if hasattr(node,'node_executable')]
    transport=next(node for node in nodes if node.node_executable=='physical_trajectory')
    move_group=next(node for node in nodes if node.node_executable=='move_group')
    assert evaluate_parameters(context,transport._Node__parameters)[0]['initial_position_tolerance']==expected
    transport_params=evaluate_parameters(context,transport._Node__parameters)[0]
    assert transport_params['gripper_close_width']==settings['bt_navigator']['ros__parameters']['bt_motion']['gripper_close_width']
    assert transport_params['gripper_close_duration']==settings['bt_navigator']['ros__parameters']['bt_motion']['gripper_close_duration']
    assert evaluate_parameters(context,move_group._Node__parameters)[0]['trajectory_execution.allowed_start_tolerance']==expected


def test_custom_motion_yaml_reaches_moveit_and_feedback(tmp_path):
    import yaml
    data = yaml.safe_load((Path(get_package_share_directory('rwh_beyond_bringup')) /
                           'config/bt_demo.yaml').read_text())
    motion = data['bt_navigator']['ros__parameters']['bt_motion']
    motion['planning_start_bounds_tolerance_deg'] = 7.0
    motion['gripper_close_duration'] = 2.75
    data['move_group']['ros__parameters']['trajectory_execution']['allowed_goal_duration_margin'] = 21.0
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(data))
    module = load(Path(__file__).resolve().parents[1] / 'launch/piper_moveit.launch.py', 'custom_motion')
    context = defaults(module)
    context.launch_configurations.update(execution_transport='feedback', params_file=str(path))
    _, group, sensor, *_ = hardware_nodes(context)
    included = next(action for action in sensor.get_sub_entities()
                    if isinstance(action, IncludeLaunchDescription))
    assert dict(included.launch_arguments)['camera_params_file'] == str(path)
    nodes = [node for node in group.get_sub_entities() if hasattr(node, 'node_executable')]
    transport = next(node for node in nodes if node.node_executable == 'physical_trajectory')
    move_group = next(node for node in nodes if node.node_executable == 'move_group')
    feedback = evaluate_parameters(context, transport._Node__parameters)[0]
    moveit = evaluate_parameters(context, move_group._Node__parameters)[0]
    assert feedback['initial_position_tolerance'] == pytest.approx(math.radians(7.0))
    assert feedback['gripper_close_duration'] == 2.75
    assert moveit['trajectory_execution.allowed_start_tolerance'] == pytest.approx(math.radians(7.0))
    assert moveit['trajectory_execution.allowed_goal_duration_margin'] == 21.0


def test_piper_driver_receives_tunable_parameters():
    module = load(Path(__file__).resolve().parents[1] / 'launch/piper_moveit.launch.py', 'driver_tuning')
    context = defaults(module)
    values = dict(fast_mode='true', pub_rate='150', enable_timeout='8.0',
                  gripper_default_effort='0.5')
    context.launch_configurations.update(values)
    driver, *_ = hardware_nodes(context)
    args = dict(driver.launch_arguments)
    assert {key: args[key].perform(context) for key in values} == values


def test_simulation_move_group_accepts_profile_override(tmp_path):
    from rclpy.parameter import parameter_dict_from_yaml_file
    path = tmp_path / 'simulation.yaml'
    path.write_text('move_group:\n  ros__parameters:\n    trajectory_execution:\n'
                    '      allowed_goal_duration_margin: 4.0\n')
    simulator = load(Path(get_package_share_directory('moma_bringup')) /
                     'launch/simulation.launch.py', 'profile_sim')
    context = defaults(simulator)
    context.launch_configurations.update(moveit='true', gui='false', rviz='false',
                                         moveit_params_file=str(path))
    node = next(n for n in simulator.nodes(context) if n.node_executable == 'move_group')
    parameters = evaluate_parameters(context, node._Node__parameters)
    assert str(parameters[1]) == str(path)
    overrides = parameter_dict_from_yaml_file(str(parameters[1]), target_nodes=['move_group'])
    assert overrides['trajectory_execution.allowed_goal_duration_margin'].value.double_value == 4.0
    context.launch_configurations['moveit_params_file'] = str(tmp_path / 'missing.yaml')
    with pytest.raises(ValueError, match='moveit_params_file'):
        simulator.nodes(context)


@pytest.mark.parametrize('degrees', [5.0, 10.0, 15.0, 0.0, -1.0, float('nan'), 181.0])
def test_start_tolerance_reads_degrees_from_yaml(monkeypatch, tmp_path, degrees):
    import yaml
    import rwh_beyond_bringup.piper_hardware as hardware
    original = hardware.get_package_share_directory
    data = yaml.safe_load((Path(original('rwh_beyond_bringup')) / 'config/bt_demo.yaml').read_text())
    data['bt_navigator']['ros__parameters']['bt_motion']['planning_start_bounds_tolerance_deg'] = degrees
    (tmp_path/'config').mkdir()
    (tmp_path/'config/bt_demo.yaml').write_text(yaml.safe_dump(data))
    monkeypatch.setattr(hardware, 'get_package_share_directory',
        lambda name:str(tmp_path) if name == 'rwh_beyond_bringup' else original(name))
    if not math.isfinite(degrees) or not 0 < degrees <= 180.:
        with pytest.raises(ValueError, match='degrees'):
            hardware.moveit_parameters(DEFAULT_TCP)
    else:
        params = hardware.moveit_parameters(DEFAULT_TCP)
        assert params['trajectory_execution']['allowed_start_tolerance'] == math.radians(degrees)


def test_shared_octomap_switch_disables_hardware_map_updater():
    import yaml
    path = Path(__file__).resolve().parents[1] / 'config/bt_demo.yaml'
    settings = yaml.safe_load(path.read_text())
    enabled = settings['/**']['ros__parameters']['octomap_enabled']
    params = moveit_parameters(DEFAULT_TCP)
    assert ('sensors' in params) is enabled
    assert 'sensors' not in moveit_parameters(DEFAULT_TCP, octomap=False)


@pytest.mark.parametrize('scaling,margin,valid', [
    (3.0, 20.0, True), (4.0, 7.0, True), (1.0, 0.0, True),
    (0.0, 20.0, False), (True, 20.0, False),
    (float('nan'), 20.0, False), (3.0, -1.0, False),
    (3.0, float('inf'), False), (3.0, '20', False),
])
def test_execution_timing_reads_shared_yaml(monkeypatch, tmp_path, scaling, margin, valid):
    import yaml
    import rwh_beyond_bringup.piper_hardware as hardware
    original = hardware.get_package_share_directory
    data = yaml.safe_load((Path(original('rwh_beyond_bringup')) / 'config/bt_demo.yaml').read_text())
    data['move_group']['ros__parameters']['trajectory_execution'] = {
        'allowed_execution_duration_scaling': scaling,
        'allowed_goal_duration_margin': margin,
    }
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/bt_demo.yaml').write_text(yaml.safe_dump(data))
    monkeypatch.setattr(hardware, 'get_package_share_directory',
        lambda name: str(tmp_path) if name == 'rwh_beyond_bringup' else original(name))
    if not valid:
        with pytest.raises(ValueError, match='allowed_'):
            hardware.moveit_parameters(DEFAULT_TCP)
    else:
        execution = hardware.moveit_parameters(DEFAULT_TCP)['trajectory_execution']
        assert execution['allowed_execution_duration_scaling'] == scaling
        assert execution['allowed_goal_duration_margin'] == margin


def test_execution_timeout_budget_covers_feedback_start_blend():
    # A short planned motion can need a 10-degree initial correction at 0.1 rad/s.
    execution = moveit_parameters(DEFAULT_TCP)['trajectory_execution']
    planned = (8.275310 - 2.0) / 3.0
    bound = planned * execution['allowed_execution_duration_scaling'] + execution['allowed_goal_duration_margin']
    assert bound > math.radians(10.0) / 0.1 + planned + 0.5



def test_d435_wrist_geometry_and_rviz_use_same_model_without_d405_or_internal_tf():
    params = moveit_parameters(DEFAULT_TCP, camera_model=True,
                              camera_type='d435', nominal_camera_tf=False)
    robot = ET.fromstring(params['robot_description'])
    semantic = ET.fromstring(params['robot_description_semantic'])
    assert robot.find("link[@name='d405_link']") is None
    assert robot.find("link[@name='d435_link']/collision/geometry/box").get('size') == '0.02505 0.090 0.025'
    assert robot.find("joint[@name='d435_mount_joint']/parent").get('link') == 'link6'
    assert robot.find("joint[@name='d435_mount_joint']/origin").get('xyz') == '-0.06 0.0 0.065'
    assert robot.find("link[@name='d435_color_optical_frame']") is None
    pairs = {frozenset((p.get('link1'), p.get('link2'))) for p in semantic.findall('disable_collisions')
             if 'd435_link' in (p.get('link1'),p.get('link2'))}
    assert pairs == {frozenset(('link6','d435_link')),frozenset(('gripper_base','d435_link'))}
    module=load(Path(__file__).resolve().parents[1]/'launch/moveit_rviz.launch.py','d435_rviz')
    context=defaults(module)
    context.launch_configurations.update(camera_type='d435',use_sim_time='false',tcp_offset=DEFAULT_TCP)
    node,=module.create_rviz(context)
    rviz=evaluate_parameters(context,node._Node__parameters)[0]
    assert rviz['robot_description']==params['robot_description']
    assert rviz['robot_description_semantic']==params['robot_description_semantic']


def test_d435_hardware_starts_pipeline_camera_once_and_disables_d405():
    module=load(Path(__file__).resolve().parents[1]/'launch/piper_moveit.launch.py','d435_hardware')
    context=defaults(module)
    context.launch_configurations.update(camera_type='d435',enable_d435='true',d435_serial='001234')
    _,_,sensor_group,camera=hardware_nodes(context)
    legacy=next(a for a in sensor_group.get_sub_entities() if isinstance(a,IncludeLaunchDescription))
    assert dict(legacy.launch_arguments)['enable_d405']=='false'
    camera=next(a for a in camera.get_sub_entities() if isinstance(a,IncludeLaunchDescription))
    args=dict(camera.launch_arguments)
    assert args['serial_no']=='001234' and args['pipeline_topics']=='true'
    assert args['rviz']=='false'
    context.launch_configurations['start_sensors']='false'
    assert len(hardware_nodes(context))==2



def test_custom_d435_mount_matches_moveit_rviz_and_robot_state_publisher():
    mount=dict(d435_mount_parent='base_link', d435_mount_xyz='[0.12,-0.03,0.8]',
               d435_mount_rpy='[0.1,0.2,-0.3]')
    params=moveit_parameters(DEFAULT_TCP,camera_model=True,camera_type='d435',
                             nominal_camera_tf=False,**mount)
    robot=ET.fromstring(params['robot_description'])
    joint=robot.find("joint[@name='d435_mount_joint']")
    assert joint.find('parent').get('link')=='base_link'
    assert joint.find('origin').get('xyz')=='0.12 -0.03 0.8'
    assert joint.find('origin').get('rpy')=='0.1 0.2 -0.3'
    semantic=ET.fromstring(params['robot_description_semantic'])
    pairs={frozenset((p.get('link1'),p.get('link2'))) for p in semantic.findall('disable_collisions')
           if 'd435_link' in (p.get('link1'),p.get('link2'))}
    assert pairs=={frozenset(('base_link','d435_link'))}
    module=load(Path(__file__).resolve().parents[1]/'launch/piper_moveit.launch.py','custom_mount_hw')
    ctx=defaults(module);ctx.launch_configurations.update(camera_type='d435',**mount)
    _,group,*_=hardware_nodes(ctx)
    nodes=[n for n in group.get_sub_entities() if hasattr(n,'node_package')]
    for executable in ('robot_state_publisher','move_group'):
        node=next(n for n in nodes if n.node_executable==executable)
        actual=evaluate_parameters(ctx,node._Node__parameters)[0]
        assert actual['robot_description']==params['robot_description']
    module=load(Path(__file__).resolve().parents[1]/'launch/moveit_rviz.launch.py','custom_mount_rviz')
    ctx=defaults(module);ctx.launch_configurations.update(camera_type='d435',use_sim_time='false',**mount)
    node,=module.create_rviz(ctx)
    actual=evaluate_parameters(ctx,node._Node__parameters)[0]
    assert actual['robot_description']==params['robot_description']
    assert actual['robot_description_semantic']==params['robot_description_semantic']


@pytest.mark.parametrize('override', [
    {'d435_mount_xyz':'[0,0]'}, {'d435_mount_xyz':'[true,0,0]'},
    {'d435_mount_rpy':'[0,0,1e999]'}, {'d435_mount_parent':'unknown_link'},
    {'d435_mount_parent':'d435_link'}, {'d435_mount_parent':'bad frame'},
])
def test_invalid_mount_is_rejected_before_any_hardware_launch(override):
    with pytest.raises(ValueError,match='d435_mount'):
        moveit_parameters(DEFAULT_TCP,camera_model=True,camera_type='d435',**override)


def test_custom_d405_mount_matches_moveit_rviz_and_robot_state_publisher():
    mount=dict(d405_mount_parent='base_link', d405_mount_xyz='[0.12,-0.03,0.8]',
               d405_mount_rpy='[0.1,0.2,-0.3]')
    params=moveit_parameters(DEFAULT_TCP,camera_model=True,camera_type='d405',
                             nominal_camera_tf=False,**mount)
    robot=ET.fromstring(params['robot_description'])
    joint=robot.find("joint[@name='d405_mount_joint']")
    assert joint.find('parent').get('link')=='base_link'
    assert joint.find('origin').get('xyz')=='0.12 -0.03 0.8'
    assert joint.find('origin').get('rpy')=='0.1 0.2 -0.3'
    semantic=ET.fromstring(params['robot_description_semantic'])
    pairs={frozenset((p.get('link1'),p.get('link2'))) for p in semantic.findall('disable_collisions')
           if 'd405_link' in (p.get('link1'),p.get('link2'))}
    assert pairs=={frozenset(('base_link','d405_link'))}
    module=load(Path(__file__).resolve().parents[1]/'launch/piper_moveit.launch.py','custom_mount_hw')
    ctx=defaults(module);ctx.launch_configurations.update(camera_type='d405',**mount)
    _,group,*_=hardware_nodes(ctx)
    nodes=[n for n in group.get_sub_entities() if hasattr(n,'node_package')]
    for executable in ('robot_state_publisher','move_group'):
        node=next(n for n in nodes if n.node_executable==executable)
        actual=evaluate_parameters(ctx,node._Node__parameters)[0]
        assert actual['robot_description']==params['robot_description']
    module=load(Path(__file__).resolve().parents[1]/'launch/moveit_rviz.launch.py','custom_mount_rviz')
    ctx=defaults(module);ctx.launch_configurations.update(camera_type='d405',use_sim_time='false',**mount)
    node,=module.create_rviz(ctx)
    actual=evaluate_parameters(ctx,node._Node__parameters)[0]
    assert actual['robot_description']==params['robot_description']
    assert actual['robot_description_semantic']==params['robot_description_semantic']


@pytest.mark.parametrize('range_value,default_planner,valid', [
    (0.15, 'RRTstar', True), (0.0, 'RRTConnect', True),
    (-0.1, 'RRTConnect', False), (float('nan'), 'RRTConnect', False),
    (True, 'RRTConnect', False), (0.1, 'missing_planner', False),
])
def test_move_group_planner_overrides_are_applied_and_validated(
        monkeypatch, tmp_path, range_value, default_planner, valid):
    import yaml
    import rwh_beyond_bringup.piper_hardware as hardware
    original = hardware.get_package_share_directory
    root = tmp_path/'share'
    (root/'config').mkdir(parents=True)
    settings = yaml.safe_load((Path(original('rwh_beyond_bringup'))/'config/bt_demo.yaml').read_text())
    ompl = settings['move_group']['ros__parameters']['ompl']
    ompl['planner_configs']['RRTConnect']['range'] = range_value
    ompl['arm']['default_planner_config'] = default_planner
    (root/'config/bt_demo.yaml').write_text(yaml.safe_dump(settings))
    monkeypatch.setattr(hardware, 'get_package_share_directory',
                        lambda name: str(root) if name == 'rwh_beyond_bringup' else original(name))
    if not valid:
        with pytest.raises(ValueError):
            hardware.moveit_parameters(DEFAULT_TCP)
    else:
        params = hardware.moveit_parameters(DEFAULT_TCP)
        assert params['ompl']['planner_configs']['RRTConnect']['range'] == range_value
        assert params['ompl']['arm']['default_planner_config'] == default_planner
