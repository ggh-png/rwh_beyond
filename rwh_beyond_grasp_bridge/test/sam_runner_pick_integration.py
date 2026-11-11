"""Private-domain runner -> BT -> real SAM 3 / GraspGenX -> physical MuJoCo pick test."""
import json,os,signal,subprocess,time
os.environ["RWH_SEGMENTATION_BACKEND"]="sam3"
os.environ["RWH_GRASP_TARGET_TEXT"]="red box"
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Empty,Trigger
from lifecycle_msgs.srv import GetState
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents as PC
from rwh_beyond_msgs.action import RunDemo

if os.environ.get('ROS_DOMAIN_ID','0')=='0':raise RuntimeError('A private nonzero ROS_DOMAIN_ID is mandatory')
tree=ET.parse('/root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/behavior_trees/demo.xml')
sequence=tree.find('.//Sequence')
if sequence.find('MoveObservation') is None:
 index=next(i for i,n in enumerate(sequence) if n.tag=='SegmentObject' or (n.tag=='GraspStep' and n.get('command') in ('segment_target','prepare')))
 sequence.insert(index,ET.Element('MoveObservation'))
if sequence[-1].tag=='MoveHome':sequence.remove(sequence[-1])
tree_path=Path('/tmp/sam3-runner-baseline.xml');tree.write(tree_path,encoding='unicode')
rclpy.init();node=rclpy.create_node('runner_sam3_pick_test')
root=Path('/root/ros2_ws/src/rwh_beyond')
params=yaml.safe_load((root/'rwh_beyond_bringup/params/bt_demo_params.yaml').read_text())
params['bt_navigator']['ros__parameters']['run_demo']['enable_groot_monitoring']=False
params_file=Path('/tmp/runner-sam3-pick-bt.yaml');params_file.write_text(yaml.safe_dump(params))
config=yaml.safe_load(Path('/root/ros2_ws/src/ros_runner/config/runners.yaml').read_text())
for runner in config['runners']:
 runner['on_launch']=False
 if runner['name']=='bt_demo':runner['activate_script']=[runner['activate_script'][0]+f' params_file:={params_file}']
config_file=Path('/tmp/runner-sam3-pick-config.yaml');config_file.write_text(yaml.safe_dump(config))
env=dict(os.environ,RWH_PIPER_SIM_GUI='false',RWH_PIPER_SIM_RVIZ='false',MUJOCO_GL='osmesa',ROS_RUNNER_LOG_DIR='/tmp/sam3-pick-runner-logs')
log_path=Path('/tmp/sam3-pick-runner.log');log=log_path.open('w')
process=subprocess.Popen(['ros2','run','ros_runner','ros_runner','--ros-args','-r','__node:=grasp_runner_test',
 '-p',f'config_yaml:={config_file}','-p','auto_start:=true'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
activate=node.create_client(Empty,'grasp_runner_test/activate_bt_demo')
deactivate=node.create_client(Empty,'grasp_runner_test/deactivate_bt_demo')
state=node.create_client(GetState,'bt_navigator/get_state')
physics=node.create_client(Trigger,'simulation/grasp_state')
scene=node.create_client(GetPlanningScene,'get_planning_scene')
action=ActionClient(node,RunDemo,'run_demo')

def until(predicate,timeout=30):
 end=time.monotonic()+timeout
 while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 assert predicate(),'timeout: '+str(log_path)

def result(future,timeout=30):until(future.done,timeout);return future.result()
def physical():return result(physics.call_async(Trigger.Request()))
traces=[];trace=[]
def feedback(msg):
 value=msg.feedback.current_stage
 if not trace or trace[-1]!=value:trace.append(value);print(value,flush=True)

def goal():
 handle=result(action.send_goal_async(RunDemo.Goal(behavior_tree=str(tree_path)),feedback_callback=feedback),15)
 assert handle.accepted
 return handle

try:
 until(activate.service_is_ready)
 result(activate.call_async(Empty.Request()))
 until(lambda:state.service_is_ready() and action.server_is_ready() and physics.service_is_ready(),45)
 until(lambda:result(state.call_async(GetState.Request())).current_state.id==3,30)
 initial=json.loads(physical().message)
 # Cancel during candidate preparation; must restore target-specific scene changes.
 handle=goal();future=handle.get_result_async()
 until(lambda:any('select_collision_free_grasp' in stage for stage in trace) or future.done(),120)
 assert not future.done(),'goal ended before cancellation test reached selection'
 assert result(handle.cancel_goal_async()).goals_canceling
 canceled=result(future,30);assert canceled.status==5,canceled
 traces.append(list(trace));trace.clear()
 until(lambda:not json.loads(physical().message)['exclude_grasp_region'],10)
 query=GetPlanningScene.Request();query.components.components=PC.WORLD_OBJECT_NAMES|PC.ALLOWED_COLLISION_MATRIX|PC.ROBOT_STATE_ATTACHED_OBJECTS
 recovered=result(scene.call_async(query)).scene
 assert 'red_grasp_target' not in {o.id for o in recovered.world.collision_objects}
 assert not recovered.robot_state.attached_collision_objects
 matrix=recovered.allowed_collision_matrix
 for finger in ['gripper_link1','gripper_link2']:
  if 'red_grasp_target' in matrix.entry_names:
   assert not matrix.entry_values[matrix.entry_names.index(finger)].enabled[matrix.entry_names.index('red_grasp_target')]
 print('PASS: cancellation restored selected target region exclusion and finger contact rules',flush=True)
 if os.environ.get('SAM_RUNNER_CANCEL_ONLY')=='1':
  print('PASS: cancellation-only integration completed',flush=True)
  raise SystemExit(0)
 handle=goal();outcome=result(handle.get_result_async(),240)
 hold_until=time.monotonic()+5
 while time.monotonic()<hold_until:rclpy.spin_once(node,timeout_sec=.05)
 final_response=physical();final=json.loads(final_response.message)
 assert outcome.status==4 and outcome.result.error_code==0,outcome
 assert final_response.success and final['object_position'][2]>initial['object_position'][2]+.06,final
 attached=result(scene.call_async(query)).scene.robot_state.attached_collision_objects
 assert any(o.object.id=='red_grasp_target' and o.link_name=='tcp_link' for o in attached)
 traces.append(list(trace));trace.clear()
 # An already-held target must not be opened/dropped by another demo goal.
 repeated=result(goal().get_result_async(),15)
 assert repeated.status==6 and 'Already holding' in repeated.result.error_msg,repeated
 assert not any(x in trace for x in ['move_home','open_gripper','move_pregrasp'])
 traces.append(list(trace))
 report=dict(status='PASS',mock=False,cancellation_recovery=True,repeat_goal_guard=True,
   initial=initial,final=final,stages=traces,attached_object='red_grasp_target')
 Path('/tmp/sam3-pick-runner-result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,indent=2),flush=True)
 result(deactivate.call_async(Empty.Request()))
 until(lambda:not physics.service_is_ready() and not state.service_is_ready(),30)
 print('PASS: runner stopped navigator, bridge and unused simulation dependencies',flush=True)
finally:
 if deactivate.service_is_ready():
  try:result(deactivate.call_async(Empty.Request()),20)
  except Exception:pass
 node.destroy_node();rclpy.try_shutdown()
 if process.poll() is None:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
 log.close()
