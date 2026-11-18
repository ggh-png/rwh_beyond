"""Load camera node defaults from a shared ROS parameter file."""
from pathlib import Path

import yaml


def camera_parameters(path, node):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or not isinstance(data.get(node), dict):
        raise ValueError(f'{path} must contain camera node {node}')
    params = data[node].get('ros__parameters')
    if not isinstance(params, dict):
        raise ValueError(f'{path}: {node}.ros__parameters must be a mapping')
    return dict(params)
