"""PerceptionStages operations, independent of BT command dispatch."""

import copy, time, uuid
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from rclpy.time import Time
from rclpy.duration import Duration
from rwh_beyond_model_utils.depth_converter import depth_xyz
from .red_perception import red_mask, pose_matrix
from rwh_beyond_sam3_bridge.action_client import segment_via_action
from rwh_beyond_model_utils.zmq_client import RequestCanceled
from .stage_errors import Canceled


class PerceptionStages:
    def snapshot(self):
        after = self.get_clock().now().nanoseconds

        def ready():
            return any(
                set(v) == {'rgb', 'depth', 'info'} and k[0] * 10**9 + k[1] > after
                for k, v in list(self.frames.items())
            )

        try:
            self.wait(ready, 10)
        except TimeoutError as error:
            samples = [(stamp, sorted(frame)) for stamp, frame in list(self.frames.items())]
            raise TimeoutError(
                f'RGB-D snapshot unavailable: use_sim_time={self.get_parameter("use_sim_time").value}, '
                f'after_ns={after}, now_ns={self.get_clock().now().nanoseconds}, '
                f'buffered_frames={samples}'
            ) from error
        keys = [
            k
            for k, v in list(self.frames.items())
            if set(v) == {'rgb', 'depth', 'info'} and k[0] * 10**9 + k[1] > after
        ]
        return copy.deepcopy(self.frames[max(keys)])

    def segment_target(self, target_text=None):
        requested_text = (target_text or '').strip()
        backend = self.get_parameter('segmentation_backend').value
        if requested_text and backend != 'sam3':
            raise ValueError('Text target selection requires segmentation_backend=sam3')
        target_text = requested_text or self.get_parameter('target_text').value
        if self.attached:
            raise RuntimeError('Already holding object')
        self.cleanup()
        self.stage('acquire_object_rgbd')
        frame = self.snapshot()
        captured_at = time.monotonic()
        rgb, depth = frame['rgb'], frame['depth']
        transform = self.buffer.lookup_transform(
            self.base,
            depth.header.frame_id,
            Time.from_msg(depth.header.stamp),
            timeout=Duration(seconds=2.0),
        )
        camera = Pose()
        camera.position.x = transform.transform.translation.x
        camera.position.y = transform.transform.translation.y
        camera.position.z = transform.transform.translation.z
        camera.orientation = transform.transform.rotation
        capture_joints = copy.deepcopy(self.joints)
        self.stage('segment_target_on_server')
        if backend == 'sam3':
            box = list(self.get_parameter('target_box').value)
            point = list(self.get_parameter('selection_point').value)
            try:
                mask, score, count, _ = segment_via_action(
                    self.segment,
                    rgb,
                    depth,
                    target_text,
                    box if any(box) else [],
                    point if min(point) >= 0 else [],
                    self.get_parameter('selection_policy').value,
                    timeout=60.0,
                    canceled=lambda: self.active_handle is not None
                    and self.active_handle.is_cancel_requested,
                )
            except RequestCanceled as error:
                raise Canceled(str(error)) from error
            self.stage(f'segmented_target score={score:.4f} instances={count}')
        elif backend == 'color_demo':
            labels = red_mask(rgb)
            mask = Image(
                header=depth.header,
                height=depth.height,
                width=depth.width,
                encoding='mono8',
                step=depth.width,
                data=labels.astype(np.uint8).tobytes(),
            )
        else:
            raise ValueError('segmentation_backend must be sam3 or explicit color_demo')
        self.perception = dict(
            target_text=target_text,
            frame=frame,
            mask=mask,
            base_T_camera=pose_matrix(camera),
            backend=backend,
            captured_at=captured_at,
            capture_joints=capture_joints,
        )
        self.observation_id = uuid.uuid4().hex
        self.plan_id = ""
        self.visualization.publish_segmentation(rgb, depth, mask)

    def perception_context(self):
        if self.perception is None:
            raise RuntimeError('No segmented target; run segment_target first')
        if time.monotonic() - self.perception['captured_at'] > 30:
            self.perception = None
            raise RuntimeError('Perception snapshot expired; reobserve target')
        previous = self.perception.get('capture_joints')
        current = getattr(self, 'joints', None)
        if previous is not None and current is not None:
            old = dict(zip(previous.name, previous.position))
            new = dict(zip(current.name, current.position))
            if any(
                name in old and name in new and abs(old[name] - new[name]) > 0.02
                for name in [f'joint{i}' for i in range(1, 7)]
            ):
                self.perception = None
                raise RuntimeError('Arm moved after capture; reobserve target')
        return self.perception

    def build_object_cloud(self):
        context = self.perception_context()
        frame = context['frame']
        xyz = depth_xyz(frame['depth'], frame['info'], context['mask'])
        cloud = create_cloud_xyz32(frame['depth'].header, xyz.tolist())
        context.update(xyz=xyz, cloud=cloud)
        self.visualization.publish_cloud(cloud, xyz, context['base_T_camera'])
        self.stage(f'local_object_cloud points={len(xyz)} frame={cloud.header.frame_id}')
