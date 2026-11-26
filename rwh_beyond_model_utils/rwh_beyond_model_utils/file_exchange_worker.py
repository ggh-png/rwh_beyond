"""Watch an exchange root and call the model endpoints on the inference host."""
import argparse
import fcntl
import json
import logging
from pathlib import Path
import time

import numpy as np

from .file_exchange import (
    GRASP, SAM, VERSION, atomic_write, exchange_file, json_bytes, png_bytes, read_file,
    read_pcd, read_png,
)
from .pose_converter import quaternion_xyzw, validate_candidates
from .zmq_client import ProtocolError, request

log = logging.getLogger(__name__)


def process_stage(directory, view, stage, endpoint, timeout, rpc=request):
    ready = exchange_file(directory, view, f'{stage}.ready.json')
    receipt = exchange_file(directory, view, f'{stage}.result.json')
    if not ready.is_file() or receipt.exists():
        return False
    result = dict(version=VERSION, request_id=directory.name, view=view)
    try:
        meta = json.loads(read_file(ready))
        if any(meta.get(key) != value for key, value in result.items()):
            raise ProtocolError('Ready metadata does not match directory')
        result['header'] = meta['header']
        if stage == SAM:
            image = read_png(read_file(exchange_file(directory, view, 'tx_image.png')), 'RGB')
            prompt = json.loads(read_file(exchange_file(directory, view, 'tx_text.json')))['text']
            payload = dict(action='segment', rgb=image, text=prompt)
            payload.update({key: meta[key] for key in ('box', 'selection_point', 'selection_policy')})
            response = rpc(endpoint, payload, timeout)
            mask = np.asarray(response['mask'])
            if mask.dtype != np.uint8 or mask.shape != image.shape[:2] or not np.isin(mask, [0, 1]).all():
                raise ProtocolError('Invalid SAM binary mask')
            atomic_write(exchange_file(directory, view, 'rx_mask.png'), png_bytes(mask * 255))
            result['model'] = dict(backend=response['backend'], mock=response['mock'],
                                   score=float(response['score']),
                                   instance_count=response['instance_count'])
        elif stage == GRASP:
            cloud = read_pcd(read_file(exchange_file(directory, view, 'tx_object.pcd')))
            payload = dict(action='infer', point_cloud=cloud)
            payload.update({key: meta[key] for key in (
                'gripper_name', 'num_grasps', 'grasp_threshold', 'topk_num_grasps')})
            response = rpc(endpoint, payload, timeout)
            if response.get('gripper_name') != meta['gripper_name']:
                raise ProtocolError('Grasp response does not match requested gripper')
            matrices, scores = validate_candidates(response, meta['topk_num_grasps'])
            atomic_write(exchange_file(directory, view, 'rx_grasp_data.json'), json_bytes(dict(
                header=meta['header'], gripper_name=response['gripper_name'],
                mock=response.get('mock') is True, grasps=matrices.tolist(),
                confidences=scores.tolist(), poses=[
                    {'position': matrix[:3, 3].tolist(),
                     'quaternion_xyzw': quaternion_xyzw(matrix[:3, :3]).tolist(),
                     'score': float(score)}
                    for matrix, score in zip(matrices, scores)
                ])))
        result['success'] = True
    except Exception as error:
        log.warning('%s/%s stage %s failed: %s', directory.name, view, stage, error)
        result.update(success=False, error=str(error))
    atomic_write(receipt, json_bytes(result))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default='~/beyond_demo/com_msgs')
    parser.add_argument('--sam-endpoint', default='tcp://127.0.0.1:5567')
    parser.add_argument('--grasp-endpoint', default='tcp://127.0.0.1:5566')
    parser.add_argument('--timeout', type=float, default=55)
    parser.add_argument('--poll-seconds', type=float, default=0.5)
    args = parser.parse_args()
    if not 0 < args.timeout <= 3600 or not 0.1 <= args.poll_seconds <= 10:
        parser.error('Invalid timeout or polling interval')
    logging.basicConfig(level=logging.INFO)
    root = Path(args.root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    with (root / '.worker.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(1, 'A worker already owns this exchange root\n')
        log.info('Watching %s for model requests', root)
        endpoints = {SAM: args.sam_endpoint, GRASP: args.grasp_endpoint}
        try:
            while True:
                for ready in sorted(root.glob('*/*.ready.json')):
                    view, _, stage = ready.name.removesuffix('.ready.json').rpartition('_')
                    if stage in endpoints:
                        try:
                            process_stage(ready.parent, view or 'front', stage, endpoints[stage], args.timeout)
                        except ValueError as error:
                            log.warning('Ignoring %s: %s', ready, error)
                time.sleep(args.poll_seconds)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
