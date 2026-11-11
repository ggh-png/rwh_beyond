"""Official msgpack-numpy wire format; each request owns a fresh REQ socket.

No model imports, GPU dependencies, shared sockets, or automatic infer retries.
Cancel closes the client socket only: it cannot stop remote GPU computation.
"""
import time
import math
import numpy as np
import msgpack
import msgpack_numpy
import zmq

class RequestCanceled(Exception):
    pass

class ServerError(Exception):
    pass

class ProtocolError(Exception):
    pass

def decode_numpy(value):
    # msgpack_numpy's object-dtype path uses pickle; never accept that on the wire.
    if b'nd' in value:
        dtype = np.dtype(value[b'type'])
        if dtype.hasobject or dtype.kind not in 'fiu':
            raise ValueError('numeric arrays only')
        shape = value.get(b'shape', ())
        if len(shape) > 4 or any(type(n) is not int or n < 0 for n in shape):
            raise ValueError('invalid array shape')
        size = math.prod(shape) * dtype.itemsize if value[b'nd'] else dtype.itemsize
        if size > 32 * 1024 * 1024 or size != len(value[b'data']):
            raise ValueError('invalid array byte length')
    return msgpack_numpy.decode(value)

def request(endpoint, payload, timeout=60.0, canceled=lambda: False):
    if not 0 < timeout <= 3600:
        raise ValueError('timeout must be in (0, 3600] seconds')
    packed = msgpack.packb(payload, default=msgpack_numpy.encode, use_bin_type=True)
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.IMMEDIATE, 1)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.MAXMSGSIZE, 32 * 1024 * 1024)
    deadline = time.monotonic() + timeout
    try:
        socket.connect(endpoint)
        for event in (zmq.POLLOUT, zmq.POLLIN):
            while True:
                if canceled():
                    raise RequestCanceled('client canceled; remote inference may still be running')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('GraspGenX request deadline exceeded')
                if socket.poll(min(50, max(1, int(remaining * 1000))), event):
                    if event == zmq.POLLOUT:
                        socket.send(packed, flags=zmq.NOBLOCK)
                    else:
                        try:
                            response = msgpack.unpackb(socket.recv(), raw=False,
                                object_hook=decode_numpy)
                        except Exception as error:
                            raise ProtocolError('invalid msgpack response') from error
                        if not isinstance(response, dict):
                            raise ProtocolError('response must be a map')
                        if 'error' in response:
                            raise ServerError(str(response['error']))
                        return response
                    break
    finally:
        socket.close(linger=0)
        context.term()
