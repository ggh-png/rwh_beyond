"""Private-domain MuJoCo pick test with real GraspGenX. Never hardware."""
import json,os,signal,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.action import ActionClient
from rwh_beyond_msgs.action import RunDemo
from std_srvs.srv import Trigger

if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
 raise RuntimeError('Use a private nonzero ROS_DOMAIN_ID; never run this test in the hardware graph')

rclpy.init();node=rclpy.create_node('red_pick_check')
logpath=Path('/tmp/red-pick-full.log');log=logpath.open('w');processes=[]
def wait(future,timeout=240):
 end=time.monotonic()+timeout
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():raise TimeoutError('pick test timed out; '+str(logpath))
 return future.result()
last=['']
def feedback(f):
 stage=f.feedback.current_stage+' / '+f.feedback.error_msg
 if stage!=last[0]:print(stage,flush=True);last[0]=stage
try:
 params=yaml.safe_load(Path('/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/params/bt_demo_params.yaml').read_text())
 params['bt_navigator']['ros__parameters']['run_demo']['enable_groot_monitoring']=False
 file=Path('/tmp/red-pick-test-params.yaml');file.write_text(yaml.safe_dump(params))
 for cmd in [
  ['ros2','launch','moma_bringup','simulation.launch.py','moveit:=true','gui:=false','rviz:=false'],
  ['ros2','launch','rwh_beyond_grasp_bridge','grasp_pipeline.launch.py'],
  ['ros2','launch','rwh_beyond_bringup','bt_demo.launch.py',f'params_file:={file}']]:
  processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
 client=ActionClient(node,RunDemo,'run_demo');physics=node.create_client(Trigger,'simulation/grasp_state')
 assert client.wait_for_server(timeout_sec=45)
 assert physics.wait_for_service(timeout_sec=15)
 initial=json.loads(wait(physics.call_async(Trigger.Request())).message)
 handle=wait(client.send_goal_async(RunDemo.Goal(),feedback_callback=feedback),15)
 assert handle.accepted
 outcome=wait(handle.get_result_async())
 until_hold=time.monotonic()+5
 while time.monotonic()<until_hold:rclpy.spin_once(node,timeout_sec=.05)
 final_response=wait(physics.call_async(Trigger.Request()))
 final=json.loads(final_response.message)
 report=dict(status=outcome.status,error_code=outcome.result.error_code,error_name=outcome.result.error_name,
    message=outcome.result.error_msg,stage=outcome.result.failed_stage,initial=initial,final=final)
 Path('/tmp/red-pick-full-result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,indent=2),flush=True)
 assert outcome.status==4 and outcome.result.error_code==0,report
 assert final_response.success and final['object_position'][2]>initial['object_position'][2]+.06,report
 print('PASS: BT lifted the physical red box with bilateral contacts and a real model grasp',flush=True)
finally:
 node.destroy_node();rclpy.try_shutdown()
 for p in reversed(processes):
  if p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 log.close()
