"""Standalone D435 composition without opening USB or starting robot processes."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('d435_launch', ROOT / 'launch/d435.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def context(**overrides):
    result = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(result)
    result.launch_configurations.update(overrides)
    return result


def test_standalone_d435_uses_rgb_sensor_and_no_robot_or_moveit():
    ctx = context(serial_no='001234', depth_profile='848x480x15')
    nodes = module.camera_nodes(ctx)
    assert [(n.node_package, n.node_executable) for n in nodes] == [
        ('realsense2_camera', 'realsense2_camera_node'), ('rviz2', 'rviz2')]
    params = evaluate_parameters(ctx, nodes[0]._Node__parameters)[0]
    assert params['device_type'] == 'd435'
    assert params['serial_no'] == '001234'
    assert params['rgb_camera.color_profile'] == '640x480x15'
    assert params['depth_module.depth_profile'] == '848x480x15'
    assert 'depth_module.color_profile' not in params
    assert params['align_depth.enable'] is params['pointcloud.enable'] is True
    assert params['publish_tf'] is True and params['use_sim_time'] is False
    assert len(module.camera_nodes(context(rviz='false'))) == 1


def test_rviz_topics_match_private_camera_namespace_and_sensor_qos():
    data = yaml.safe_load((ROOT / 'rviz/d435.rviz').read_text())['Visualization Manager']
    assert data['Global Options']['Fixed Frame'] == 'd435_link'
    topics = [d['Topic'] for d in data['Displays'] if 'Topic' in d]
    assert {t['Value'] for t in topics} == {
        '/camera/d435/color/image_raw', '/camera/d435/aligned_depth_to_color/image_raw',
        '/camera/d435/depth/color/points'}
    assert all(t['Reliability Policy'] == 'Best Effort' for t in topics)


@pytest.mark.parametrize('profile', ['0x480x15', '640x480', '-1x480x15'])
def test_invalid_profile_rejected_before_launch(profile):
    with pytest.raises(ValueError, match='depth_profile'):
        module.camera_nodes(context(depth_profile=profile))



def test_pipeline_mode_aliases_aligned_rgbd_and_keeps_sdk_tf_frames():
    ctx=context(pipeline_topics='true',rviz='false')
    node,=module.camera_nodes(ctx)
    from launch.utilities import perform_substitutions
    remaps={perform_substitutions(ctx,a):perform_substitutions(ctx,b)
            for a,b in node._Node__remappings}
    assert remaps['~/color/image_raw']=='/camera/color/image_raw'
    assert remaps['~/aligned_depth_to_color/image_raw']=='/camera/depth/image_rect_raw'
    assert remaps['~/aligned_depth_to_color/camera_info']=='/camera/depth/camera_info'
    assert remaps['~/depth/color/points']=='/camera/depth/color/points'
    assert remaps['~/depth/image_rect_raw']=='/camera/depth/image_raw'
    params=evaluate_parameters(ctx,node._Node__parameters)[0]
    assert params['camera_name']=='d435' and params['publish_tf'] is True
