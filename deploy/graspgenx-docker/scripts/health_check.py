#!/usr/bin/env python3
"""CPU-only official health request; never imports GraspGenX or torch."""
import argparse
import msgpack
import zmq

parser = argparse.ArgumentParser()
parser.add_argument('--endpoint', default='tcp://127.0.0.1:5556')
args = parser.parse_args()
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.setsockopt(zmq.LINGER, 0)
socket.setsockopt(zmq.SNDTIMEO, 4000)
socket.setsockopt(zmq.RCVTIMEO, 4000)
try:
    socket.connect(args.endpoint)
    socket.send(msgpack.packb({'action': 'health'}, use_bin_type=True))
    response = msgpack.unpackb(socket.recv(), raw=False)
    if response.get('status') != 'ok':
        raise SystemExit('health response was not ok')
    print('GraspGenX healthy')
finally:
    socket.close(linger=0)
    context.term()
