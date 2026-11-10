"""Send an already segmented NPY cloud; never invokes robot actions."""
import argparse
import json
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from rwh_beyond_msgs.action import InferGrasps

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('cloud', help='segmented (N,3) .npy in meters')
    parser.add_argument('--frame', required=True)
    parser.add_argument('--gripper', default='piper_hand')
    parser.add_argument('--num-grasps', type=int, default=100)
    parser.add_argument('--top-k', type=int, default=20)
    args = parser.parse_args()
    xyz = np.load(args.cloud, allow_pickle=False)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        parser.error('cloud must have shape (N,3)')
    rclpy.init()
    node = Node('graspgenx_infer_cloud')
    try:
        cloud = create_cloud_xyz32(Header(frame_id=args.frame, stamp=node.get_clock().now().to_msg()), xyz.tolist())
        send_goal_and_print(node, InferGrasps.Goal(object_cloud=cloud,
            gripper_name=args.gripper, num_grasps=args.num_grasps, top_k=args.top_k))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def send_goal_and_print(node, goal):
    client = ActionClient(node, InferGrasps, 'infer_grasps')
    if not client.wait_for_server(timeout_sec=10):
        raise RuntimeError('infer_grasps Action server unavailable')
    future = client.send_goal_async(goal,
        feedback_callback=lambda f: print(f.feedback.stage, flush=True))
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError('goal rejected: bridge busy')
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future)
    result = future.result().result
    print(json.dumps(dict(success=result.success, message=result.message, segmentation_score=result.segmentation_score,
        segmentation_instances=result.segmentation_instances, candidates=[
        dict(id=c.id, frame=c.pose.header.frame_id,
             stamp=dict(sec=c.pose.header.stamp.sec, nanosec=c.pose.header.stamp.nanosec), score=c.score,
             position=[c.pose.pose.position.x,c.pose.pose.position.y,c.pose.pose.position.z],
             quaternion=[c.pose.pose.orientation.x,c.pose.pose.orientation.y,c.pose.pose.orientation.z,c.pose.pose.orientation.w])
        for c in result.candidates]), indent=2))
    if not result.success:
        raise SystemExit(1)

    return result
