"""Validate BT-owned observation artifacts at the MoveIt motion boundary."""
import copy
import time
import numpy as np
from geometry_msgs.msg import Pose, Point
from rclpy.time import Time
from rclpy.duration import Duration
from rwh_beyond_model_utils.cloud_converter import cloud_xyz
from .red_perception import pose_matrix


class PerceptionStages:
    def accept_observation(self, request):
        if self.attached or self.scene_active:
            raise RuntimeError('Previous target still active; cleanup/release before selecting')
        depth, info, mask, cloud = (request.depth_image, request.camera_info,
                                   request.object_mask, request.object_cloud)
        if not request.observation_id or not depth.width or not depth.height:
            raise ValueError('BT must supply an observation ID and registered RGB-D capture')
        if cloud.header != depth.header or mask.header != depth.header:
            raise ValueError('Cloud/mask frame and stamp must match captured depth')
        if info.header.frame_id != depth.header.frame_id or (info.width, info.height) != (depth.width, depth.height):
            raise ValueError('CameraInfo must match captured depth frame and resolution')
        age = (self.get_clock().now().nanoseconds - Time.from_msg(depth.header.stamp).nanoseconds) * 1e-9
        if not 0 <= age <= 30:
            raise ValueError('BT observation expired or timestamp is in the future')
        captured = dict(zip(request.capture_joints.name, request.capture_joints.position))
        current = dict(zip(self.joints.name, self.joints.position)) if self.joints else {}
        for name in [f'joint{i}' for i in range(1, 7)]:
            if name not in captured or name not in current or not np.isfinite(captured[name]) or abs(current[name] - captured[name]) > .02:
                raise ValueError('Arm moved after BT capture or capture joint state is incomplete')
        transform = self.buffer.lookup_transform(self.base, depth.header.frame_id,
            Time.from_msg(depth.header.stamp), timeout=Duration(seconds=2.0)).transform
        camera = Pose(position=Point(x=transform.translation.x, y=transform.translation.y,
                                    z=transform.translation.z), orientation=transform.rotation)
        self.perception = dict(frame={'rgb': request.rgb_image, 'depth': depth, 'info': info}, mask=mask,
            xyz=cloud_xyz(cloud), cloud=cloud, base_T_camera=pose_matrix(camera),
            backend='sam3', captured_at=time.monotonic() - age,
            capture_joints=copy.deepcopy(request.capture_joints))
        if hasattr(self, 'visualization'):
            self.visualization.publish_segmentation(request.rgb_image, depth, mask)
            self.visualization.publish_cloud(cloud, self.perception['xyz'], self.perception['base_T_camera'])
            self.visualization.publish_candidates(request.candidates, cloud.header, self.perception['base_T_camera'])
        self.observation_id = request.observation_id
        self.plan_id = ''
        return self.perception_context()

    def perception_context(self):
        if self.perception is None:
            raise RuntimeError('No BT observation supplied to motion selection')
        if time.monotonic() - self.perception['captured_at'] > 30:
            self.perception = None
            raise RuntimeError('Perception snapshot expired; reobserve target')
        previous = self.perception.get('capture_joints')
        current = getattr(self, 'joints', None)
        if previous is not None and current is not None:
            old = dict(zip(previous.name, previous.position))
            new = dict(zip(current.name, current.position))
            if any(name in old and name in new and abs(old[name] - new[name]) > .02
                   for name in [f'joint{i}' for i in range(1, 7)]):
                self.perception = None
                raise RuntimeError('Arm moved after capture; reobserve target')
        return self.perception
