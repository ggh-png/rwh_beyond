"""CPU transport demo ONLY. Output is synthetic, never a valid grasp prediction."""
import argparse
import signal
import msgpack
import numpy as np
import zmq
from rwh_beyond_model_utils.zmq_client import decode_numpy
import msgpack_numpy
from rclpy.utilities import remove_ros_args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5556')
    args = parser.parse_args(remove_ros_args()[1:])
    if not args.endpoint.startswith('tcp://127.0.0.1:'):
        parser.error('mock server must bind to loopback')
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.MAXMSGSIZE, 32*1024*1024)
    socket.bind(args.endpoint)
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f'MOCK ONLY: synthetic grasps at {args.endpoint}; not for robot execution', flush=True)
    try:
        while running:
            if not socket.poll(100):
                continue
            try:
                payload = msgpack.unpackb(socket.recv(), raw=False, object_hook=decode_numpy)
                if payload.get('action') == 'health':
                    response = {'status':'ok', 'mock':True}
                elif payload.get('action') == 'infer':
                    cloud = np.asarray(payload['point_cloud'])
                    if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud) or not np.isfinite(cloud).all():
                        raise ValueError('expected finite nonempty (N,3) point cloud')
                    count = min(int(payload['topk_num_grasps']), int(payload['num_grasps']), 3)
                    if not 1 <= count <= 3:
                        raise ValueError('invalid grasp counts')
                    grasps = np.repeat(np.eye(4,dtype=np.float32)[None], count, axis=0)
                    grasps[:,:3,3] = cloud.mean(axis=0)
                    response = dict(grasps=grasps, confidences=np.linspace(.9,.7,count,dtype=np.float32),
                        gripper_name=payload['gripper_name'], mock=True, timing={'infer_ms':0.0})
                else:
                    raise ValueError('mock supports health and infer only')
            except Exception as error:
                response = {'error':f'{type(error).__name__}: {error}'}
            socket.send(msgpack.packb(response, use_bin_type=True, default=msgpack_numpy.encode))
    finally:
        socket.close(linger=0)
        context.term()
