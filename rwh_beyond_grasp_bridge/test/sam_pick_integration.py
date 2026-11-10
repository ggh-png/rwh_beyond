"""Private-domain MuJoCo baseline pick with real SAM 3 and GraspGenX. Never hardware."""
import json,os,signal,subprocess,time
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
import rclpy
from rclpy.action import ActionClient
from rwh_beyond_msgs.action import RunDemo
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data

if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
 raise RuntimeError('Use a private nonzero ROS_DOMAIN_ID; never run this test in the hardware graph')

rclpy.init();node=rclpy.create_node('red_pick_check')
logpath=Path('/tmp/sam3-pick-full.log');log=logpath.open('w');processes=[]
def wait(future,timeout=240):
 end=time.monotonic()+timeout
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():raise TimeoutError('pick test timed out; '+str(logpath))
 return future.result()
clouds=[];stages=[]
cloud_sub=node.create_subscription(PointCloud2,'grasp/object_cloud',lambda msg:clouds.append(msg),qos_profile_sensor_data)
last=['']
def feedback(f):
 stages.append(f.feedback.current_stage)
 stage=f.feedback.current_stage+' / '+f.feedback.error_msg
 if stage!=last[0]:print(stage,flush=True);last[0]=stage
try:
 # Keep the user's demo.xml unchanged. This baseline explicitly observes and stops after lift.
 tree=ET.parse('/root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/behavior_trees/demo.xml')
 sequence=tree.find('.//Sequence')
 if sequence.find('MoveObservation') is None:
  index=next(i for i,n in enumerate(sequence) if n.tag=='GraspStep' and n.get('command') in ('segment_target','prepare'))
  sequence.insert(index,ET.Element('MoveObservation'))
 full_tree=os.environ.get('SAM_PICK_FULL_TREE')=='1'
 if not full_tree and sequence[-1].tag=='MoveHome':sequence.remove(sequence[-1])
 tree_path=Path('/tmp/sam3-baseline-pick.xml');tree.write(tree_path,encoding='unicode')
 params=yaml.safe_load(Path('/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/params/bt_demo_params.yaml').read_text())
 params['bt_navigator']['ros__parameters']['run_demo']['enable_groot_monitoring']=False
 file=Path('/tmp/sam3-pick-test-params.yaml');file.write_text(yaml.safe_dump(params))
 for cmd in [
  ['ros2','launch','moma_bringup','simulation.launch.py','moveit:=true','gui:=false','rviz:=false'],
  ['ros2','launch','rwh_beyond_grasp_bridge','grasp_pipeline.launch.py','segmentation_backend:=sam3','target_text:=red box'],
  ['ros2','launch','rwh_beyond_bringup','bt_demo.launch.py',f'params_file:={file}']]:
  processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
 client=ActionClient(node,RunDemo,'run_demo');physics=node.create_client(Trigger,'simulation/grasp_state')
 assert client.wait_for_server(timeout_sec=45)
 assert physics.wait_for_service(timeout_sec=15)
 initial=json.loads(wait(physics.call_async(Trigger.Request())).message)
 handle=wait(client.send_goal_async(RunDemo.Goal(behavior_tree=str(tree_path)),feedback_callback=feedback),15)
 assert handle.accepted
 outcome=wait(handle.get_result_async())
 until_hold=time.monotonic()+2
 while time.monotonic()<until_hold:rclpy.spin_once(node,timeout_sec=.05)
 final_response=wait(physics.call_async(Trigger.Request()))
 final=json.loads(final_response.message)
 report=dict(full_default_tree=full_tree,local_cloud_points=clouds[-1].width if clouds else 0,local_cloud_frame=clouds[-1].header.frame_id if clouds else None,stages=stages,status=outcome.status,error_code=outcome.result.error_code,error_name=outcome.result.error_name,
    message=outcome.result.error_msg,stage=outcome.result.failed_stage,initial=initial,final=final)
 Path('/tmp/sam3-pick-full-result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,indent=2),flush=True)
 assert clouds and clouds[-1].width>=100
 local_index=next(i for i,s in enumerate(stages) if s.startswith('local_object_cloud'))
 infer_index=stages.index('infer_grasps_from_local_cloud')
 assert local_index<infer_index,stages
 assert outcome.status==4 and outcome.result.error_code==0,report
 if full_tree:
  assert any(s.startswith('physical_lift_verified') for s in stages),report
 else:
  assert final_response.success and final['object_position'][2]>initial['object_position'][2]+.06,report
 print('PASS: BT lifted the physical red box with bilateral contacts, real SAM 3 mask and GraspGenX grasp',flush=True)
finally:
 node.destroy_node();rclpy.try_shutdown()
 for p in reversed(processes):
  if p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 log.close()
