"""Send stored registered RGB-D and target prompt, without robot motion."""
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
    parser.add_argument('rgb', help='registered uint8 HxWx3 RGB NPY')
    parser.add_argument('--text', default='')
    parser.add_argument('--save-mask', help='write the selected mask to NPY')
    parser.add_argument('--box', type=float, nargs=4, default=[0.,0.,0.,0.], metavar=('X0','Y0','X1','Y1'))
    parser.add_argument('--point', type=int, nargs=2, metavar=('U','V'))
    parser.add_argument('--selection-policy', choices=['unique','highest_score'], default='unique')
    parser.add_argument('--frame', required=True, help='calibrated depth optical frame')
    for name in ['fx','fy','cx','cy']:
        parser.add_argument('--'+name, required=True, type=float)
    parser.add_argument('--gripper', default='piper_hand')
    parser.add_argument('--num-grasps', type=int, default=100)
    parser.add_argument('--top-k', type=int, default=20)
    args = parser.parse_args()
    values = np.load(args.depth, allow_pickle=False)
    rgb = np.load(args.rgb, allow_pickle=False)
    if values.ndim != 2 or rgb.shape != (*values.shape,3) or rgb.dtype != np.uint8 or values.dtype not in [np.dtype('uint16'),np.dtype('float32')]:
        parser.error('expected 2D uint16/float32 depth and registered uint8 RGB')
    if not args.text and not any(args.box):parser.error('--text or --box is required')
    rclpy.init()
    node = Node('graspgenx_infer_rgbd')
    try:
        header = Header(frame_id=args.frame, stamp=node.get_clock().now().to_msg())
        height, width = values.shape
        depth = Image(header=header, height=height, width=width,
            encoding='16UC1' if values.dtype==np.uint16 else '32FC1',
            is_bigendian=0, step=width*values.dtype.itemsize,
            data=values.astype(values.dtype.newbyteorder('<')).tobytes())
        image = Image(header=header, height=height, width=width, encoding='rgb8',
            is_bigendian=0, step=width*3, data=np.ascontiguousarray(rgb).tobytes())
        info = CameraInfo(header=header, height=height, width=width,
            k=[args.fx,0.,args.cx,0.,args.fy,args.cy,0.,0.,1.])
        result = send_goal_and_print(node, InferGrasps.Goal(input_mode=InferGrasps.Goal.RGBD,
            depth_image=depth, rgb_image=image, camera_info=info, target_text=args.text, target_box=args.box,
            use_selection_point=args.point is not None, selection_u=args.point[0] if args.point else 0,
            selection_v=args.point[1] if args.point else 0, selection_policy=args.selection_policy,
            gripper_name=args.gripper, num_grasps=args.num_grasps, top_k=args.top_k))
        if args.save_mask:
            np.save(args.save_mask,np.frombuffer(bytes(result.object_mask.data),np.uint8).reshape(height,width))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
