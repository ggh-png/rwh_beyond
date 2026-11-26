"""Two-stage inference exchange. A ready file is published after all its inputs."""
from contextlib import nullcontext
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
from tempfile import TemporaryFile
import time
from uuid import uuid4

import numpy as np
from PIL import Image

from .zmq_client import ProtocolError, RequestCanceled, ServerError

VERSION = 2
REQUEST_ID = re.compile(r'[0-9]{14}|[0-9]{8}T[0-9]{6}Z-[0-9a-f]{16,32}')
VIEW = re.compile(r'[a-zA-Z0-9_-]{1,32}')
MAX_FILE = 32 * 1024 * 1024
SAM = 'sam'
GRASP = 'grasp'


def json_bytes(data):
    return json.dumps(data, ensure_ascii=False, allow_nan=False).encode()


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{uuid4().hex}.part')
    try:
        temporary.write_bytes(data)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def png_bytes(array):
    target = io.BytesIO()
    Image.fromarray(array).save(target, format='PNG')
    return target.getvalue()


def read_png(data, mode):
    with Image.open(io.BytesIO(data), formats=['PNG']) as img:
        if img.mode != mode or img.width * img.height > 2073600:
            raise ProtocolError(f'Expected a {mode} PNG of at most 2073600 pixels')
        return np.asarray(img).copy()


def pcd_bytes(xyz):
    points = np.asarray(xyz, dtype='<f4')
    if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 200000 or not np.isfinite(points).all():
        raise ProtocolError('Expected finite XYZ points with shape (1..200000, 3)')
    count = len(points)
    return (f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n'
            ).encode() + points.tobytes()


def read_pcd(data):
    header, marker, body = data.partition(b'DATA binary\n')
    if not marker or len(header) > 1024:
        raise ProtocolError('Expected binary PCD')
    fields = dict(line.split(maxsplit=1) for line in header.decode('ascii').splitlines())
    count = int(fields['POINTS'])
    if (fields['FIELDS'] != 'x y z' or fields['SIZE'] != '4 4 4' or fields['TYPE'] != 'F F F' or
            fields['COUNT'] != '1 1 1' or fields['HEIGHT'] != '1' or int(fields['WIDTH']) != count or
            not 1 <= count <= 200000 or len(body) != count * 12):
        raise ProtocolError('Invalid binary XYZ PCD layout')
    points = np.frombuffer(body, dtype='<f4').reshape(count, 3)
    if not np.isfinite(points).all():
        raise ProtocolError('Nonfinite XYZ in PCD')
    return points


def read_file(path):
    with path.open('rb') as stream:
        data = stream.read(MAX_FILE + 1)
    if len(data) > MAX_FILE:
        raise ProtocolError(f'Exchange file too large: {path.name}')
    return data


def exchange_file(directory, view, name):
    if not REQUEST_ID.fullmatch(directory.name) or not VIEW.fullmatch(view):
        raise ValueError('Invalid file exchange request ID or view')
    prefix = '' if view == 'front' else f'{view}_'
    return directory / (prefix + name)


class Exchange:
    def __init__(self, node):
        settings = {
            'file_transport': 'ssh',
            'file_local_root': '/root/wsyoon_data/beyond_demo/com_msgs',
            'file_remote_root': '~/beyond_demo/com_msgs',
            'file_ssh_target': '',
            'file_ssh_port': 0,
            'file_poll_seconds': 0.5,
        }
        for key, value in settings.items():
            node.declare_parameter(key, value)
        self.node = node

    def request(self, request_id, view, header):
        def exchange(_endpoint, payload, timeout, canceled):
            settings = {key: self.node.get_parameter('file_' + key).value for key in (
                'transport', 'local_root', 'remote_root', 'ssh_target', 'ssh_port', 'poll_seconds')}
            client = FileClient(**settings)
            return client.run(request_id, view, header, payload, timeout, canceled)
        return exchange


class FileClient:
    def __init__(self, transport, local_root, remote_root, ssh_target, ssh_port, poll_seconds):
        if transport not in ('ssh', 'shared'):
            raise ValueError('file_transport must be ssh or shared')
        if not 0.1 <= poll_seconds <= 10:
            raise ValueError('file_poll_seconds must be in [0.1, 10]')
        self.local = Path(local_root).expanduser()
        self.poll_seconds = poll_seconds
        self.transport = transport
        self.remote = PurePosixPath(remote_root)
        if transport == 'ssh':
            ssh_target = ssh_target or os.environ.get('RWH_FILE_SSH_TARGET', '')
            ssh_port = ssh_port or int(os.environ.get('RWH_FILE_SSH_PORT', '22'))
            if (not re.fullmatch(r'[A-Za-z0-9_.@:-]+', ssh_target) or
                    ssh_target.startswith('-') or not 1 <= ssh_port <= 65535):
                raise ValueError('Set file_ssh_target (SSH alias or user@host) and file_ssh_port')
            if not (self.remote.is_absolute() or str(self.remote).startswith('~/')) or '..' in self.remote.parts:
                raise ValueError('file_remote_root must be absolute or start with ~/')
            options = ['-o', 'BatchMode=yes']
            self.ssh_env = None
            if (os.environ.get('RWH_FILE_ASKPASS_CONFIG') and
                    f'{ssh_target}:{ssh_port}' == os.environ.get('RWH_FILE_ASKPASS_TARGET')):
                options = ['-o', 'BatchMode=no', '-o', 'PreferredAuthentications=password',
                           '-o', 'NumberOfPasswordPrompts=1']
                self.ssh_env = {
                    **os.environ,
                    'SSH_ASKPASS': os.environ['RWH_FILE_ASKPASS'],
                    'SSH_ASKPASS_REQUIRE': 'force',
                    'ROS_RUNNER_ASKPASS_CONFIG': os.environ['RWH_FILE_ASKPASS_CONFIG'],
                    'ROS_RUNNER_ASKPASS_TARGET': os.environ['RWH_FILE_ASKPASS_TARGET'],
                }
            self.ssh = ['ssh', '-T', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5',
                        *options, '-p', str(ssh_port), ssh_target]

    def _remote_file(self, relative):
        path = str(self.remote / relative.as_posix())
        return '"$HOME"/' + shlex.quote(path[2:]) if path.startswith('~/') else shlex.quote(path)

    @staticmethod
    def _remaining(deadline, canceled):
        if canceled():
            raise RequestCanceled('File exchange canceled')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('File exchange response deadline exceeded')
        return remaining

    def _ssh(self, command, deadline, canceled, content=None, missing_ok=False):
        self._remaining(deadline, canceled)
        stdin = TemporaryFile() if content is not None else nullcontext(subprocess.DEVNULL)
        with stdin as source:
            if content is not None:
                source.write(content)
                source.seek(0)
            with subprocess.Popen([*self.ssh, command], stdin=source,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=self.ssh_env) as process:
                try:
                    while True:
                        timeout = min(0.2, self._remaining(deadline, canceled))
                        try:
                            out, err = process.communicate(timeout=timeout)
                            break
                        except subprocess.TimeoutExpired:
                            continue
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
        if process.returncode == 44 and missing_ok:
            return None
        if process.returncode != 0:
            raise ServerError(f'SSH exchange failed ({process.returncode}): {err.decode(errors="replace").strip()}')
        if len(out) > MAX_FILE:
            raise ProtocolError('SSH exchange file is too large')
        return out

    def _publish(self, relative, data, deadline, canceled):
        atomic_write(self.local / relative, data)
        if self.transport == 'ssh':
            parent = self._remote_file(relative.parent)
            path = self._remote_file(relative)
            partial = self._remote_file(relative.with_name(f'.{relative.name}.{uuid4().hex}.part'))
            self._ssh(f'umask 077; mkdir -p -- {parent} && cat > {partial} && mv -- {partial} {path}',
                      deadline, canceled, data)

    def _read(self, relative, deadline, canceled, missing_ok=False):
        self._remaining(deadline, canceled)
        if self.transport == 'shared':
            try:
                return read_file(self.local / relative)
            except FileNotFoundError:
                if missing_ok:
                    return None
                raise
        path = self._remote_file(relative)
        data = self._ssh(f'if test -f {path}; then head -c {MAX_FILE + 1} -- {path}; else exit 44; fi',
                         deadline, canceled, missing_ok=missing_ok)
        if data is not None:
            atomic_write(self.local / relative, data)
        return data

    def run(self, request_id, view, header, payload, timeout, canceled):
        directory = Path(request_id)
        deadline = time.monotonic() + timeout
        meta = dict(version=VERSION, request_id=request_id, view=view, header=header)
        if payload['action'] == 'segment':
            stage = SAM
            image = payload['rgb']
            files = {
                'tx_text.json': json_bytes({'text': payload['text']}),
                'tx_image.png': png_bytes(image),
            }
            meta.update({key: payload[key] for key in ('box', 'selection_point', 'selection_policy')})
        elif payload['action'] == 'infer':
            stage = GRASP
            files = {'tx_object.pcd': pcd_bytes(payload['point_cloud'])}
            meta.update({key: payload[key] for key in (
                'gripper_name', 'num_grasps', 'grasp_threshold', 'topk_num_grasps')})
        else:
            raise ValueError('Unsupported file exchange action')
        ready = exchange_file(directory, view, f'{stage}.ready.json')
        if (self.local / ready).exists():
            raise ProtocolError(f'Exchange stage already submitted: {ready}')
        for name, data in files.items():
            self._publish(exchange_file(directory, view, name), data, deadline, canceled)
        self._publish(ready, json_bytes(meta), deadline, canceled)
        receipt = exchange_file(directory, view, f'{stage}.result.json')
        data = self._read(receipt, deadline, canceled, missing_ok=True)
        while data is None:
            time.sleep(min(self.poll_seconds, self._remaining(deadline, canceled)))
            data = self._read(receipt, deadline, canceled, missing_ok=True)
        result = json.loads(data)
        if any(result.get(key) != meta[key] for key in ('version', 'request_id', 'view', 'header')):
            raise ProtocolError('Response metadata does not match the request')
        if not result.get('success'):
            raise ServerError(str(result.get('error', 'Remote inference failed')))
        if stage == SAM:
            mask = self._read(exchange_file(directory, view, 'rx_mask.png'), deadline, canceled)
            labels = read_png(mask, 'L')
            if not np.isin(labels, [0, 255]).all():
                raise ProtocolError('Invalid binary mask PNG')
            return dict(result['model'], mask=labels // 255)
        response = json.loads(self._read(exchange_file(directory, view, 'rx_grasp_data.json'), deadline, canceled))
        if response.get('header') != header:
            raise ProtocolError('Grasp result frame or timestamp does not match the cloud')
        return response


def frame_header(header):
    return dict(frame_id=header.frame_id, stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec))
