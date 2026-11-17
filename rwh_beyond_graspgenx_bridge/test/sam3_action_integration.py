"""Real RGB-D -> SAM 3 -> GraspGenX Action test. Never sends a motion goal."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from rwh_beyond_msgs.action import InferGrasps

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture',help='NPZ containing rgb uint8, depth float32 meters and k 3x3')
parser.add_argument('--target-text',default='red box')
parser.add_argument('--report',default='/tmp/sam3-action-validation.json')
parser.add_argument('--mask',default='/tmp/sam3-selected-mask.npy')
args=parser.parse_args()
rclpy.init();node=Node('sam3_actual_validation')
try:
    c=np.load(args.capture)
    rgb=c['rgb'];depth=c['depth'];h,w=depth.shape
    header=Header(frame_id='d405_color_optical_frame',stamp=node.get_clock().now().to_msg())
    goal=InferGrasps.Goal(input_mode=InferGrasps.Goal.RGBD,
        rgb_image=Image(header=header,height=h,width=w,encoding='rgb8',step=w*3,data=rgb.tobytes()),
        depth_image=Image(header=header,height=h,width=w,encoding='32FC1',step=w*4,data=depth.astype('<f4').tobytes()),
        camera_info=CameraInfo(header=header,height=h,width=w,k=c['k'].reshape(-1).tolist()),
        target_text=args.target_text,gripper_name='piper_hand',num_grasps=100,top_k=20)
    client=ActionClient(node,InferGrasps,'infer_grasps')
    assert client.wait_for_server(timeout_sec=20)
    start=time.monotonic()
    f=client.send_goal_async(goal,feedback_callback=lambda f:print(f.feedback.stage,flush=True))
    rclpy.spin_until_future_complete(node,f,timeout_sec=20);handle=f.result();assert handle.accepted
    f=handle.get_result_async();rclpy.spin_until_future_complete(node,f,timeout_sec=150)
    outcome=f.result();result=outcome.result
    report=dict(status=outcome.status,success=result.success,message=result.message,elapsed=time.monotonic()-start,
        segmentation_score=result.segmentation_score,segmentation_instances=result.segmentation_instances,
        candidates=len(result.candidates),scores=[c.score for c in result.candidates],
        frame=result.candidates[0].pose.header.frame_id if result.candidates else None,
        mask_pixels=int(np.count_nonzero(np.frombuffer(bytes(result.object_mask.data),np.uint8))))
    Path(args.report).write_text(json.dumps(report,indent=2))
    if result.object_mask.data:
        np.save(args.mask,np.frombuffer(bytes(result.object_mask.data),np.uint8).reshape(h,w))
    print(json.dumps(report,indent=2),flush=True)
    assert result.success and result.message.startswith('OK:') and len(result.candidates)==20
    assert result.segmentation_instances>=1 and report['mask_pixels']>=100
finally:
    node.destroy_node();rclpy.try_shutdown()
