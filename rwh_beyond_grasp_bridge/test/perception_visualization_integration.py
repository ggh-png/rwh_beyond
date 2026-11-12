"""Real SAM observation BT and late-joining visualization checks in private MuJoCo."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rwh_beyond_msgs.action import RunDemo
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py.point_cloud2 import read_points_numpy
import yaml

from rwh_beyond_grasp_bridge.visualization import snapshot_qos

if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
    raise RuntimeError('Use a private nonzero ROS_DOMAIN_ID; this test is MuJoCo-only')

root = Path('/root/ros2_ws/src/rwh_beyond')
prefix = Path(os.environ.get('GRASP_TEST_LOG_PREFIX','/tmp/grasp-rviz-validation'))
plan_debug = os.environ.get('GRASP_DEBUG_PLAN') == '1'
bt_rviz = os.environ.get('GRASP_TEST_RVIZ') == '1' and os.environ.get('GRASP_MOVEIT_RVIZ') == '1'
use_runner = os.environ.get('GRASP_USE_RUNNER') == '1'
log = prefix.with_suffix('.log').open('w')
processes = []
rclpy.init()
node = rclpy.create_node('perception_visualization_check')


def wait(future, timeout=180):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError('Perception preview timed out; see ' + str(prefix))
    return future.result()


def rviz_process_ids():
    matches = []
    domain = ('ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID']).encode()
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            arguments = (proc / 'cmdline').read_bytes().split(b'\0')
            if not arguments or Path(arguments[0].decode()).name != 'rviz2':
                continue
            if domain in (proc / 'environ').read_bytes().split(b'\0'):
                matches.append(int(proc.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return matches


def start(command):
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    processes.append(process)
    return process


try:
    params = yaml.safe_load((root / 'rwh_beyond_bringup/config/bt_demo.yaml').read_text())
    params['bt_navigator']['ros__parameters']['run_demo']['enable_groot_monitoring'] = False
    params_file = Path('/tmp/grasp-rviz-bt-params.yaml')
    params_file.write_text(yaml.safe_dump(params))
    if use_runner:
        os.environ.update(RWH_PIPER_SIM_GUI='false', RWH_PIPER_SIM_RVIZ='true',
                          RWH_BT_RVIZ='true' if bt_rviz else 'false',
                          RWH_SEGMENTATION_BACKEND='sam3', RWH_GRASP_TARGET_TEXT='red box')
        config = yaml.safe_load(Path('/root/ros2_ws/src/ros_runner/config/runners.yaml').read_text())
        for runner in config['runners']:
            runner['on_launch'] = False
            if runner['name'] == 'bt_demo_sim':
                runner['activate_script'][0] += ' params_file:=' + str(params_file)
        config_file = Path('/tmp/bt-rviz-owner-runners.yaml')
        config_file.write_text(yaml.safe_dump(config))
        bt_process = start(['ros2', 'launch', 'ros_runner', 'runner.launch.py',
                            'node_name:=bt_rviz_owner_test', 'config_yaml:=' + str(config_file)])
        activate = node.create_client(Empty, 'bt_rviz_owner_test/activate_bt_demo_sim')
        deactivate = node.create_client(Empty, 'bt_rviz_owner_test/deactivate_bt_demo_sim')
        assert activate.wait_for_service(timeout_sec=30)
        wait(activate.call_async(Empty.Request()), 30)
    else:
        start(['ros2', 'launch', 'moma_bringup', 'simulation.launch.py',
               'moveit:=true', 'gui:=false', 'rviz:=false'])
        bt_process = start(['ros2', 'launch', 'rwh_beyond_bringup', 'bt_demo.launch.py',
                            'rviz:=' + ('true' if bt_rviz else 'false'),
                            'params_file:=' + str(params_file)])
    client = ActionClient(node, RunDemo, 'run_demo')
    assert client.wait_for_server(timeout_sec=45)
    state = node.create_client(GetState, 'bt_navigator/get_state')
    assert state.wait_for_service(timeout_sec=20)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        if wait(state.call_async(GetState.Request()), 5).current_state.id == 3:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise TimeoutError('BT navigator did not activate')
    goal = RunDemo.Goal(behavior_tree=str(
        root / 'rwh_beyond_bt/behavior_trees' / ('grasp_debug.xml' if plan_debug else 'perception_preview.xml')))
    handle = wait(client.send_goal_async(goal), 15)
    assert handle.accepted
    outcome = wait(handle.get_result_async())
    assert outcome.status == 4 and outcome.result.error_code == 0, outcome
    # Deliberately subscribe after the complete BT result, including navigator cleanup.
    messages = {}
    subscriptions = []
    topics = [('object_mask', Image), ('object_mask_preview', Image),
              ('segmentation_overlay', Image), ('object_cloud', PointCloud2),
              ('object_cloud_base', PointCloud2)]
    if plan_debug:
        topics += [('grasp_candidates', MarkerArray), ('selected_grasp', MarkerArray),
                   ('selected_tcp', PoseStamped), ('selected_pregrasp', PoseStamped)]
    for name, message_type in topics:
        subscriptions.append(node.create_subscription(
            message_type, 'grasp/' + name,
            lambda message, key=name: messages.__setitem__(key, message), snapshot_qos()))
    deadline = time.monotonic() + 10
    while len(messages) < len(topics) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(messages) == len(topics), list(messages)
    cloud = messages['object_cloud']
    base_cloud = messages['object_cloud_base']
    assert cloud.width >= 100 and cloud.width == base_cloud.width
    assert base_cloud.header.frame_id == 'base_link'
    assert cloud.header.stamp == base_cloud.header.stamp
    assert set(messages['object_mask_preview'].data) == {0, 255}
    assert messages['segmentation_overlay'].encoding == 'rgb8'
    base_xyz = read_points_numpy(base_cloud)
    assert np.isfinite(base_xyz).all()
    report = dict(status='PASS',point_count=cloud.width,cloud_frame=cloud.header.frame_id,
                  display_frame=base_cloud.header.frame_id,late_subscription_after_bt=True,
                  stamp=dict(sec=cloud.header.stamp.sec,nanosec=cloud.header.stamp.nanosec),
                  base_bounds=dict(lower=base_xyz.min(axis=0).tolist(),
                                   upper=base_xyz.max(axis=0).tolist()))
    if plan_debug:
        candidates = messages['grasp_candidates'].markers
        selected = messages['selected_grasp'].markers
        assert any(m.action == Marker.ADD and m.ns == 'model_candidates' for m in candidates)
        assert any(m.action == Marker.ADD and m.ns == 'selected_tcp_axes' for m in selected)
        report['candidate_markers'] = sum(m.ns == 'model_candidates' for m in candidates)
        report['selected_label'] = next(m.text for m in selected if m.ns == 'selected_score')
        report['pregrasp_frame'] = messages['selected_pregrasp'].header.frame_id
        assert report['pregrasp_frame'] == 'base_link'
    if os.environ.get('GRASP_TEST_RVIZ') == '1':
        before_move_group = node.get_node_names().count('move_group')
        if bt_rviz:
            rviz = bt_process
            report['rviz_owner'] = 'bt_demo.launch.py'
            report['rviz_process_count'] = len(rviz_process_ids())
            assert report['rviz_process_count'] == 1, report
        else:
            rviz = start(['ros2', 'launch', 'rwh_beyond_grasp_bridge',
                          'grasp_visualization.launch.py'])
        deadline = time.monotonic() + 30
        expected = ['object_cloud_base', 'object_mask_preview', 'segmentation_overlay']
        if plan_debug:
            expected += ['grasp_candidates', 'selected_grasp']
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(node.count_subscribers('grasp/' + topic) >= 2 for topic in expected):
                break
        report['rviz_subscribers'] = {
            topic: node.count_subscribers('grasp/' + topic) for topic in expected}
        assert rviz.poll() is None
        assert all(count >= 2 for count in report['rviz_subscribers'].values()), report
        report['move_group_counts'] = [before_move_group, node.get_node_names().count('move_group')]
        assert report['move_group_counts'] == [1, 1], report
        # Leave a short interval for the actual retained images/point cloud to render.
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        snapshot = os.environ.get('GRASP_RVIZ_SCREENSHOT_SCRIPT')
        if snapshot:
            subprocess.run(['python3', snapshot], check=True)
    if use_runner:
        wait(deactivate.call_async(Empty.Request()), 30)
        deadline = time.monotonic() + 30
        while rviz_process_ids() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report['rviz_stopped_on_bt_deactivate'] = not rviz_process_ids()
        assert report['rviz_stopped_on_bt_deactivate'], report
    prefix.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
    log.close()
