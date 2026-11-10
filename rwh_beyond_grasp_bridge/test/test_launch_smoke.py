"""Exercise installed ros2 launch/run entrypoints with synthetic data, CPU only."""
import os
import signal
import socket
import subprocess
import numpy as np


def test_launch_cloud_and_depth_cli(tmp_path):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        endpoint = f'tcp://127.0.0.1:{probe.getsockname()[1]}'
    np.save(tmp_path/'cloud.npy', np.tile([.1,.2,.3],(120,1)).astype(np.float32))
    np.save(tmp_path/'depth.npy', np.full((12,12),1000,np.uint16))
    np.save(tmp_path/'mask.npy', np.ones((12,12),np.uint8))
    environment = dict(os.environ, ROS_DOMAIN_ID='94')
    with (tmp_path/'launch.log').open('w+') as log:
        process = subprocess.Popen(['ros2','launch','rwh_beyond_grasp_bridge','inference.launch.py',
            'mock:=true',f'endpoint:={endpoint}'], env=environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            for command in [
                ['ros2','run','rwh_beyond_grasp_bridge','infer_cloud',str(tmp_path/'cloud.npy'),
                 '--frame','camera_depth_optical_frame'],
                ['ros2','run','rwh_beyond_grasp_bridge','infer_depth',str(tmp_path/'depth.npy'),str(tmp_path/'mask.npy'),
                 '--frame','camera_depth_optical_frame','--fx','100','--fy','100','--cx','6','--cy','6']]:
                result = subprocess.run(command, env=environment, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, timeout=20)
                log.flush()
                assert result.returncode == 0, result.stdout + (tmp_path/'launch.log').read_text()
                assert 'MOCK_ONLY' in result.stdout and '"success": true' in result.stdout
                assert '"stamp":' in result.stdout
        finally:
            if process.poll() is None:
                # Signal only launch; it forwards the signal to its children once.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            log.flush()
        assert 'process has died' not in (tmp_path/'launch.log').read_text()
