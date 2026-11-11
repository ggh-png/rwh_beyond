"""Verify runner activation/deactivation owns exactly one RViz, without a motion goal."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
import yaml

assert os.environ.get('ROS_DOMAIN_ID', '0') != '0'
os.environ.update(RWH_PIPER_SIM_GUI='false',RWH_PIPER_SIM_RVIZ='true',RWH_BT_RVIZ='true')
root=Path('/root/ros2_ws/src')
params=yaml.safe_load((root/'rwh_beyond/rwh_beyond_bringup/params/bt_demo_params.yaml').read_text())
params['bt_navigator']['ros__parameters']['run_demo']['enable_groot_monitoring']=False
Path('/tmp/bt-rviz-lifecycle-params.yaml').write_text(yaml.safe_dump(params))
config=yaml.safe_load((root/'ros_runner/config/runners.yaml').read_text())
for runner in config['runners']:
    runner['on_launch']=False
    if runner['name']=='bt_demo_sim':
        runner['activate_script'][0]+=' params_file:=/tmp/bt-rviz-lifecycle-params.yaml'
Path('/tmp/bt-rviz-lifecycle-runners.yaml').write_text(yaml.safe_dump(config))
log=open('/tmp/bt-rviz-lifecycle.log','w')
process=subprocess.Popen(['ros2','launch','ros_runner','runner.launch.py',
    'node_name:=bt_rviz_lifecycle_test','config_yaml:=/tmp/bt-rviz-lifecycle-runners.yaml'],
    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=rclpy.create_node('bt_rviz_lifecycle_check')

def until(predicate, timeout=45):
    deadline=time.monotonic()+timeout
    while not predicate() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    assert predicate(),'Timed out; see /tmp/bt-rviz-lifecycle.log'

def result(future):
    until(future.done)
    return future.result()

def viewers():
    found=[];domain=('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode()
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            command=(proc/'cmdline').read_bytes().split(b'\0')
            if command and Path(command[0].decode()).name=='rviz2' and domain in (proc/'environ').read_bytes().split(b'\0'):
                found.append(int(proc.name))
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return found

activate=node.create_client(Empty,'bt_rviz_lifecycle_test/activate_bt_demo_sim')
deactivate=node.create_client(Empty,'bt_rviz_lifecycle_test/deactivate_bt_demo_sim')
state=node.create_client(GetState,'bt_navigator/get_state')
try:
    until(activate.service_is_ready)
    result(activate.call_async(Empty.Request()))
    until(lambda:state.service_is_ready() and len(viewers())==1)
    until(lambda:result(state.call_async(GetState.Request())).current_state.id==3)
    # Verify the normal simulator did not create a second viewer despite its old env flag.
    until(lambda:node.get_node_names().count('move_group')==1)
    report=dict(status='PASS',rviz_owner='bt_demo.launch.py',rviz_process_count=len(viewers()),
                move_group_count=node.get_node_names().count('move_group'),motion_goal_sent=False)
    result(deactivate.call_async(Empty.Request()))
    until(lambda:not viewers())
    report['rviz_stopped_on_bt_deactivate']=True
    Path('/tmp/bt-rviz-lifecycle-result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
finally:
    node.destroy_node();rclpy.try_shutdown()
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
    log.close()
