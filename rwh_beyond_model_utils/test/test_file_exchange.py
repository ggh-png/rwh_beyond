import json
import sys
import threading
import time

import numpy as np
import pytest

from rwh_beyond_model_utils.file_exchange import FileClient, GRASP, SAM, read_pcd, pcd_bytes
from rwh_beyond_model_utils.file_exchange_worker import process_stage
from rwh_beyond_model_utils.zmq_client import ProtocolError, RequestCanceled, ServerError

JOB = '20261125143230'
HEADER = {'frame_id': 'camera_optical', 'stamp': {'sec': 7, 'nanosec': 22}}


def client(root):
    return FileClient('shared', str(root), str(root), '', 22, 0.1)


def run(client, view, payload, timeout=3, canceled=lambda: False):
    return client.run(JOB, view, HEADER, payload, timeout, canceled)


def sam_request():
    return dict(action='segment', rgb=np.zeros((8, 10, 3), dtype=np.uint8),
                text='red can', box=[], selection_point=[], selection_policy='unique')


def fake_model(endpoint, payload, timeout):
    if endpoint == 'sam':
        assert payload['text'] == 'red can'
        return dict(mask=np.ones((8, 10), dtype=np.uint8), backend='sam3', mock=False,
                    score=0.9, instance_count=1)
    np.testing.assert_array_equal(payload['point_cloud'], np.ones((150, 3)))
    return dict(grasps=np.eye(4)[None, :, :], confidences=np.array([0.8]), gripper_name='piper_hand')


def complete(root, stage, view, rpc=fake_model):
    directory = root / JOB
    def watch():
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if process_stage(directory, view, stage, stage, 1, rpc):
                return
            time.sleep(0.01)
    worker = threading.Thread(target=watch)
    worker.start()
    return worker


def test_two_stages_and_multiple_views(tmp_path):
    for view in ('front', 'left', 'right'):
        worker = complete(tmp_path, SAM, view)
        response = run(client(tmp_path), view, sam_request())
        worker.join()
        assert response['mask'].sum() == 80
    worker = complete(tmp_path, GRASP, 'fused')
    response = run(client(tmp_path), 'fused', dict(
        action='infer', point_cloud=np.ones((150, 3)), gripper_name='piper_hand',
        num_grasps=10, topk_num_grasps=1, grasp_threshold=-1))
    worker.join()
    assert response['header'] == HEADER
    assert response['confidences'] == [0.8]
    assert len(list(tmp_path.iterdir())) == 1
    assert all(path.is_file() for path in (tmp_path / JOB).iterdir())
    for view in ('left', 'right'):
        assert (tmp_path / JOB / f'{view}_rx_mask.png').is_file()
    assert (tmp_path / JOB / 'rx_mask.png').is_file()
    assert (tmp_path / JOB / 'fused_rx_grasp_data.json').is_file()


def test_incomplete_upload_is_ignored_and_canceled_job_keeps_inputs(tmp_path):
    directory = tmp_path / JOB
    directory.mkdir(parents=True)
    (directory / 'tx_text.json').write_text('{}')
    assert not process_stage(directory, 'front', SAM, 'sam', 1, fake_model)
    with pytest.raises(RequestCanceled):
        run(client(tmp_path), 'front', sam_request(),
            canceled=lambda: (directory / 'sam.ready.json').exists())
    assert json.loads((directory / 'tx_text.json').read_bytes()) == {'text': 'red can'}
    assert not (directory / 'rx_mask.png').exists()


def test_model_errors_and_timeout_are_distinct(tmp_path):
    def fail(*args):
        raise RuntimeError('model offline')
    worker = complete(tmp_path, SAM, 'front', fail)
    with pytest.raises(ServerError, match='model offline'):
        run(client(tmp_path), 'front', sam_request())
    worker.join()
    with pytest.raises(TimeoutError):
        run(client(tmp_path), 'right', sam_request(), timeout=0.2)


def test_invalid_pcd_and_request_path(tmp_path):
    encoded = pcd_bytes(np.ones((3, 3), dtype=np.float32))
    with pytest.raises(ProtocolError):
        read_pcd(encoded[:-1])
    with pytest.raises(ValueError):
        run(client(tmp_path), '../escape', sam_request())


def test_ssh_upload_survives_multiple_poll_intervals(tmp_path):
    transport = FileClient('ssh', str(tmp_path), '~/exchange', 'example.org', 22, 0.1)
    transport.ssh = [sys.executable, '-c',
                     'import sys,time; time.sleep(0.45); sys.stdout.buffer.write(sys.stdin.buffer.read())']
    payload = bytes(range(256)) * 8192
    assert transport._ssh('unused', time.monotonic() + 3, lambda: False, payload) == payload
