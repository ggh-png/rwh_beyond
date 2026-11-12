"""Exclude only a selected observed target AABB before MoveIt's robot self-filter."""
import copy
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from rwh_beyond_msgs.srv import SetGraspRegion


def validate_region(request):
    lower = np.array([request.lower.x, request.lower.y, request.lower.z])
    upper = np.array([request.upper.x, request.upper.y, request.upper.z])
    if request.frame_id != 'base_link' or not np.isfinite([lower, upper]).all() or np.any(upper<=lower) or np.any(upper-lower>.35):
        raise ValueError('Expected finite base_link bounds with dimensions in (0, .35] m')
    return lower, upper


class TargetCloudFilter(Node):
    def __init__(self):
        super().__init__('grasp_target_cloud_filter')
        self.region = None
        self.lock = threading.Lock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, 'camera/depth/planning_points', qos_profile_sensor_data)
        self.subscription = self.create_subscription(PointCloud2, 'camera/depth/color/points', self.cloud, qos_profile_sensor_data)
        self.service = self.create_service(SetGraspRegion, 'grasp/set_grasp_region', self.set_region)

    def set_region(self, request, response):
        try:
            region = validate_region(request) if request.enabled else None
            with self.lock:
                self.region = region
            response.success = True
            response.message = 'Observed target region updated'
        except ValueError as error:
            response.success = False
            response.message = str(error)
        return response

    def cloud(self, msg):
        with self.lock:
            region = self.region
        if region is None:
            self.publisher.publish(msg)
            return
        try:
            xyz = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
            transform = self.buffer.lookup_transform('base_link', msg.header.frame_id,
                Time.from_msg(msg.header.stamp)).transform
            q = transform.rotation
            rotation = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            t = transform.translation
            base_xyz = xyz@rotation.T+np.array([t.x,t.y,t.z])
            lower, upper = region
            keep = ~np.all((base_xyz>=lower)&(base_xyz<=upper), axis=1)
            # Preserve every field and endianness; flatten row padding explicitly.
            rows = np.ndarray((msg.height, msg.width, msg.point_step), dtype=np.uint8,
                buffer=bytes(msg.data), strides=(msg.row_step,msg.point_step,1))
            data = rows.reshape(-1,msg.point_step)[keep].tobytes()
            result = copy.deepcopy(msg)
            result.height, result.width = 1, int(keep.sum())
            result.row_step = result.width*result.point_step
            result.data = data
            self.publisher.publish(result)
        except Exception as error:
            # Do not forward an unfiltered frame after target exclusion is enabled.
            self.get_logger().warning('Target cloud withheld: '+str(error), throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = TargetCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
