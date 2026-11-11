"""Retained RGB-D observation displays; never used as planning or inference input."""

import copy

import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

from rwh_beyond_model_utils.depth_converter import image_array
from rwh_beyond_model_utils.rgb_converter import rgb_array
from .red_perception import pose_matrix


def snapshot_qos():
    """Keep one result for viewers that join after a BT perception step finishes."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def segmentation_images(rgb, depth, mask):
    """Return a visible 0/255 mask and RGB overlay without changing the 0/1 mask."""
    pixels = rgb_array(rgb, depth)
    labels = image_array(mask, {'mono8': 'u1', '8UC1': 'u1'}, 2073600)
    if labels.shape != pixels.shape[:2] or mask.header != depth.header:
        raise ValueError('Visualization mask must match the captured RGB-D observation')
    selected = labels != 0
    preview = Image(
        header=copy.deepcopy(mask.header), height=mask.height, width=mask.width,
        encoding='mono8', step=mask.width,
        data=(selected.astype(np.uint8) * 255).tobytes(),
    )
    overlay = pixels.copy()
    overlay[selected] = np.rint(
        pixels[selected].astype(np.float32) * 0.45 + np.array([0, 255, 0]) * 0.55
    ).astype(np.uint8)
    image = Image(
        header=copy.deepcopy(rgb.header), height=rgb.height, width=rgb.width,
        encoding='rgb8', step=rgb.width * 3, data=overlay.tobytes(),
    )
    return preview, image


def cloud_in_base(header, xyz, base_t_camera, base_frame):
    """Freeze a visualization cloud at its acquisition-time TF, preserving its stamp."""
    points = np.asarray(xyz)
    transform = np.asarray(base_t_camera)
    if points.ndim != 2 or points.shape[1] != 3 or transform.shape != (4, 4):
        raise ValueError('Expected N x 3 points and a 4 x 4 capture transform')
    if not np.isfinite(points).all() or not np.isfinite(transform).all() or not base_frame:
        raise ValueError('Visualization points/transform must be finite with a base frame')
    base_points = points @ transform[:3, :3].T + transform[:3, 3]
    base_header = copy.deepcopy(header)
    base_header.frame_id = base_frame
    return create_cloud_xyz32(base_header, base_points.tolist())


def point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def display_marker(header, base_frame, namespace, marker_id, marker_type):
    base_header = copy.deepcopy(header)
    base_header.frame_id = base_frame
    marker = Marker(header=base_header, ns=namespace, id=marker_id,
                    type=marker_type, action=Marker.ADD)
    marker.pose.orientation.w = 1.0
    return marker


def cleared_markers(header, base_frame):
    marker = display_marker(header, base_frame, '', 0, Marker.ARROW)
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


class GraspVisualization:
    """Publish processing outputs plus display-ready retained snapshots."""

    def __init__(self, node, base_frame, max_candidates=10):
        if type(max_candidates) is not int or not 0 <= max_candidates <= 100:
            raise ValueError('debug_max_candidates must be an integer in [0, 100]')
        self.base_frame = base_frame
        self.max_candidates = max_candidates
        qos = snapshot_qos()
        self.mask_publisher = node.create_publisher(Image, 'grasp/object_mask', qos)
        self.mask_preview_publisher = node.create_publisher(
            Image, 'grasp/object_mask_preview', qos)
        self.overlay_publisher = node.create_publisher(
            Image, 'grasp/segmentation_overlay', qos)
        self.cloud_publisher = node.create_publisher(PointCloud2, 'grasp/object_cloud', qos)
        self.base_cloud_publisher = node.create_publisher(
            PointCloud2, 'grasp/object_cloud_base', qos)
        self.candidates_publisher = node.create_publisher(
            MarkerArray, 'grasp/grasp_candidates', qos)
        self.selected_markers_publisher = node.create_publisher(
            MarkerArray, 'grasp/selected_grasp', qos)
        self.selected_tcp_publisher = node.create_publisher(
            PoseStamped, 'grasp/selected_tcp', qos)
        self.pregrasp_publisher = node.create_publisher(
            PoseStamped, 'grasp/selected_pregrasp', qos)

    def publish_segmentation(self, rgb, depth, mask):
        preview, overlay = segmentation_images(rgb, depth, mask)
        # A new mask invalidates the previous displayed 3D observation until cloud build.
        header = copy.deepcopy(depth.header)
        header.frame_id = self.base_frame
        self.base_cloud_publisher.publish(create_cloud_xyz32(header, []))
        self.candidates_publisher.publish(cleared_markers(depth.header, self.base_frame))
        self.selected_markers_publisher.publish(cleared_markers(depth.header, self.base_frame))
        self.mask_publisher.publish(mask)
        self.mask_preview_publisher.publish(preview)
        self.overlay_publisher.publish(overlay)

    def publish_cloud(self, cloud, xyz, base_t_camera):
        base_cloud = cloud_in_base(cloud.header, xyz, base_t_camera, self.base_frame)
        self.cloud_publisher.publish(cloud)
        self.base_cloud_publisher.publish(base_cloud)

    def publish_candidates(self, candidates, header, base_t_camera):
        """Show bounded raw model-base poses and scores, before IK/collision selection."""
        markers = cleared_markers(header, self.base_frame)
        transform = np.asarray(base_t_camera)
        for index, candidate in enumerate(candidates[:self.max_candidates]):
            if candidate.pose.header != header:
                raise ValueError('Candidate visualization must use the captured cloud frame/stamp')
            model = transform @ pose_matrix(candidate.pose.pose)
            marker = display_marker(header, self.base_frame, 'model_candidates', index, Marker.ARROW)
            marker.points = [point(model[:3, 3]), point(model[:3, 3] + model[:3, 2] * 0.06)]
            marker.scale.x, marker.scale.y, marker.scale.z = 0.002, 0.006, 0.009
            marker.color = ColorRGBA(r=0.15, g=0.65, b=1.0, a=0.65)
            markers.markers.append(marker)
            if index < 3:
                text = display_marker(header, self.base_frame, 'model_scores', index, Marker.TEXT_VIEW_FACING)
                text.pose.position = point(model[:3, 3] + [0, 0, 0.012])
                text.scale.z = 0.012
                text.text = f'Model #{index}: {candidate.score:.3f}'
                text.color = ColorRGBA(r=0.5, g=0.85, b=1.0, a=1.0)
                markers.markers.append(text)
        self.candidates_publisher.publish(markers)
        self.selected_markers_publisher.publish(cleared_markers(header, self.base_frame))

    def publish_selection(self, pregrasp, tcp, header, score, index, flipped):
        """Show the selected calibrated TCP, pregrasp and approach after feasibility checks."""
        base_header = copy.deepcopy(header)
        base_header.frame_id = self.base_frame
        self.selected_tcp_publisher.publish(PoseStamped(header=base_header, pose=tcp))
        self.pregrasp_publisher.publish(PoseStamped(header=base_header, pose=pregrasp))
        markers = cleared_markers(header, self.base_frame)
        matrix = pose_matrix(tcp)
        for axis, color in enumerate([(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.3, 1.0)]):
            marker = display_marker(header, self.base_frame, 'selected_tcp_axes', axis, Marker.ARROW)
            marker.points = [point(matrix[:3, 3]), point(matrix[:3, 3] + matrix[:3, axis] * 0.04)]
            marker.scale.x, marker.scale.y, marker.scale.z = 0.003, 0.007, 0.01
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            markers.markers.append(marker)
        pre = display_marker(header, self.base_frame, 'pregrasp', 0, Marker.SPHERE)
        pre.pose.position = copy.deepcopy(pregrasp.position)
        pre.scale.x = pre.scale.y = pre.scale.z = 0.012
        pre.color = ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0)
        markers.markers.append(pre)
        line = display_marker(header, self.base_frame, 'selected_approach', 0, Marker.LINE_STRIP)
        line.points = [copy.deepcopy(pregrasp.position), copy.deepcopy(tcp.position)]
        line.scale.x = 0.003
        line.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0)
        markers.markers.append(line)
        label = display_marker(header, self.base_frame, 'selected_score', 0, Marker.TEXT_VIEW_FACING)
        label.pose.position = point(matrix[:3, 3] + [0, 0, 0.025])
        label.scale.z = 0.014
        label.text = f'Selected TCP #{index}: {score:.3f}' + (' (flipped)' if flipped else '')
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        markers.markers.append(label)
        self.selected_markers_publisher.publish(markers)
