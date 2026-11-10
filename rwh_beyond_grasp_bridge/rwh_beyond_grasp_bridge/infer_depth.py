"""Send stored depth and a target binary mask as a ROS Action goal."""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from rwh_beyond_msgs.action import InferGrasps
from .infer_cloud import send_goal_and_print


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('depth', help='2D NPY: uint16 millimeters or float32 meters')
    parser.add_argument('mask', help='same-size binary target mask NPY, nonzero=target')
    parser.add_argument('--frame', required=True, help='calibrated depth optical frame')
    for name in ['fx','fy','cx','cy']:
        parser.add_argument('--'+name, required=True, type=float)
    parser.add_argument('--gripper', default='piper_hand')
    parser.add_argument('--num-grasps', type=int, default=100)
    parser.add_argument('--top-k', type=int, default=20)
    args = parser.parse_args()
    values = np.load(args.depth, allow_pickle=False)
    labels = np.load(args.mask, allow_pickle=False)
    if values.ndim != 2 or labels.shape != values.shape or values.dtype not in [np.dtype('uint16'),np.dtype('float32')]:
        parser.error('expected 2D uint16/float32 depth and equal-size mask')
    rclpy.init()
    node = Node('graspgenx_infer_depth')
    try:
        header = Header(frame_id=args.frame, stamp=node.get_clock().now().to_msg())
        height, width = values.shape
        depth = Image(header=header, height=height, width=width,
            encoding='16UC1' if values.dtype==np.uint16 else '32FC1',
            is_bigendian=0, step=width*values.dtype.itemsize,
            data=values.astype(values.dtype.newbyteorder('<')).tobytes())
        mask = Image(header=header, height=height, width=width, encoding='mono8',
            is_bigendian=0, step=width, data=(labels!=0).astype(np.uint8).tobytes())
        info = CameraInfo(header=header, height=height, width=width,
            k=[args.fx,0.,args.cx,0.,args.fy,args.cy,0.,0.,1.])
        send_goal_and_print(node, InferGrasps.Goal(input_mode=InferGrasps.Goal.DEPTH_IMAGE,
            depth_image=depth, object_mask=mask, camera_info=info,
            gripper_name=args.gripper, num_grasps=args.num_grasps, top_k=args.top_k))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
