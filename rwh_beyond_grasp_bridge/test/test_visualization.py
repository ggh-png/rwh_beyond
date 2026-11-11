"""Verify captured-frame displays, visible mask intensities and late-joining viewers."""
import copy
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header

from rwh_beyond_grasp_bridge.visualization import (
    GraspVisualization, cloud_in_base, segmentation_images, snapshot_qos,
)


def images():
    header = Header(frame_id='camera')
    header.stamp.sec = 42
    # RGB input is BGR with row padding, while the mask is the processing 0/1 mask.
    rgb = Image(header=header, height=1, width=2, encoding='bgr8', step=8,
                data=bytes([10, 20, 100, 30, 40, 50, 99, 99]))
    depth = Image(header=header, height=1, width=2, encoding='32FC1')
    mask = Image(header=header, height=1, width=2, encoding='mono8', step=3,
                 data=bytes([1, 0, 99]))
    return rgb, depth, mask


def test_overlay_uses_registered_rgb_and_does_not_modify_processing_mask():
    rgb, depth, mask = images()
    original = copy.deepcopy(mask)
    preview, overlay = segmentation_images(rgb, depth, mask)
    assert list(preview.data) == [255, 0]
    assert overlay.encoding == 'rgb8' and overlay.step == 6
    pixels = np.frombuffer(bytes(overlay.data), np.uint8).reshape(1, 2, 3)
    assert pixels[0, 1].tolist() == [50, 40, 30]
    assert pixels[0, 0, 1] > 100
    assert mask == original and preview.header == mask.header == overlay.header


def test_overlay_rejects_mismatched_capture():
    rgb, depth, mask = images()
    mask.header = copy.deepcopy(mask.header)
    mask.header.stamp.sec += 1
    with pytest.raises(ValueError, match='captured'):
        segmentation_images(rgb, depth, mask)


def test_base_cloud_uses_capture_transform_not_current_camera_pose():
    header = Header(frame_id='camera')
    header.stamp.sec = 42
    xyz = np.array([[1, 0, 2]], dtype=np.float32)
    transform = np.array([[0, -1, 0, 3], [1, 0, 0, 4], [0, 0, 1, 5], [0, 0, 0, 1]])
    cloud = cloud_in_base(header, xyz, transform, 'base_link')
    np.testing.assert_allclose(read_points_numpy(cloud), [[3, 5, 7]])
    assert cloud.header.frame_id == 'base_link' and cloud.header.stamp.sec == 42
    assert header.frame_id == 'camera'
    np.testing.assert_array_equal(xyz, [[1, 0, 2]])


def test_late_subscriber_receives_last_snapshot_and_new_mask_clears_displayed_cloud():
    rclpy.init()
    publisher = rclpy.create_node('visualization_snapshot_publisher')
    subscriber = rclpy.create_node('visualization_late_viewer')
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)
    try:
        visualization = GraspVisualization(publisher, 'base_link')
        rgb, depth, mask = images()
        xyz = np.array([[0.1, 0.2, 0.3]], dtype=np.float32)
        cloud = create_cloud_xyz32(depth.header, xyz.tolist())
        visualization.publish_segmentation(rgb, depth, mask)
        visualization.publish_cloud(cloud, xyz, np.eye(4))
        messages = {}
        subscriptions = []
        for topic, message_type in [('object_mask', Image), ('object_mask_preview', Image),
                                    ('segmentation_overlay', Image), ('object_cloud', PointCloud2),
                                    ('object_cloud_base', PointCloud2)]:
            subscriptions.append(subscriber.create_subscription(
                message_type, 'grasp/' + topic,
                lambda message, key=topic: messages.__setitem__(key, message), snapshot_qos()))
        deadline = time.monotonic() + 8
        while len(messages) < 5 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert len(messages) == 5
        assert messages['object_cloud_base'].header.frame_id == 'base_link'
        assert messages['object_cloud_base'].width == 1
        assert list(messages['object_mask'].data) == [1, 0, 99]
        assert list(messages['object_mask_preview'].data) == [255, 0]
        visualization.publish_segmentation(rgb, depth, mask)
        deadline = time.monotonic() + 5
        while messages['object_cloud_base'].width and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert messages['object_cloud_base'].width == 0
    finally:
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.try_shutdown()


def test_grasp_markers_use_model_capture_frame_and_selected_tcp_with_scores():
    from types import SimpleNamespace
    from geometry_msgs.msg import Pose
    from rwh_beyond_msgs.msg import GraspCandidate
    from visualization_msgs.msg import Marker

    messages = {}
    def publisher(message_type, topic, qos):
        messages[topic] = []
        return SimpleNamespace(publish=messages[topic].append)
    visualization = GraspVisualization(SimpleNamespace(create_publisher=publisher), 'base_link', 1)
    header = Header(frame_id='camera')
    header.stamp.sec = 42
    candidate = GraspCandidate(score=0.8)
    candidate.pose.header = header
    candidate.pose.pose.orientation.w = 1.0
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 3]
    candidates = [candidate, copy.deepcopy(candidate)]
    visualization.publish_candidates(candidates, header, transform)
    markers = messages['grasp/grasp_candidates'][-1].markers
    assert markers[0].action == Marker.DELETEALL
    arrows = [m for m in markers if m.type == Marker.ARROW and m.action == Marker.ADD]
    assert len(arrows) == 1  # Display limit does not truncate the input candidates.
    assert len(candidates) == 2 and candidates[0].pose.header.frame_id == 'camera'
    assert arrows[0].header.frame_id == 'base_link' and arrows[0].header.stamp.sec == 42
    np.testing.assert_allclose([arrows[0].points[0].x, arrows[0].points[0].y,
                                arrows[0].points[0].z], [1, 2, 3])
    assert any('#0: 0.800' in m.text for m in markers)
    pre, tcp = Pose(), Pose()
    pre.orientation.w = tcp.orientation.w = 1.0
    pre.position.z, tcp.position.z = 0.18, 0.1
    visualization.publish_selection(pre, tcp, header, 0.8, 12, True)
    selected = messages['grasp/selected_grasp'][-1].markers
    assert selected[0].action == Marker.DELETEALL
    assert any('Selected TCP #12' in m.text and 'flipped' in m.text for m in selected)
    approach = next(m for m in selected if m.ns == 'selected_approach')
    assert approach.points[0].z == 0.18 and approach.points[1].z == 0.1
    assert messages['grasp/selected_tcp'][-1].pose == tcp
    assert messages['grasp/selected_pregrasp'][-1].pose == pre
    assert messages['grasp/selected_tcp'][-1].header.stamp.sec == 42
    rgb, depth, mask = images()
    visualization.publish_segmentation(rgb, depth, mask)
    assert messages['grasp/grasp_candidates'][-1].markers[0].action == Marker.DELETEALL
    assert len(messages['grasp/grasp_candidates'][-1].markers) == 1
    assert len(messages['grasp/selected_grasp'][-1].markers) == 1
