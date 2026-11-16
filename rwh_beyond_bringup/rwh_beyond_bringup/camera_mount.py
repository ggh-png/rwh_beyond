"""Validated RealSense mount arguments shared by MoveIt and its RViz model."""
import ast
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory


def mount_mappings(parent='', xyz='', rpy='', camera_type='d435'):
    if camera_type not in ('d405', 'd435'):
        raise ValueError('camera_type must be d405 or d435')
    path = (Path(get_package_share_directory('moma_mujoco_ros')) / 'project/config/cameras/d405.yaml'
            if camera_type == 'd405' else Path(get_package_share_directory('rwh_beyond_bringup')) / 'config/d435_mount.yaml')
    defaults = yaml.safe_load(path.read_text())
    prefix = camera_type + '_mount'

    parent = parent or defaults['parent_frame']
    if not isinstance(parent, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*', parent):
        raise ValueError(prefix + '_parent must be an existing URDF link name')
    result = {prefix + '_parent': parent}
    for name, value, fallback in [(prefix + '_xyz', xyz, defaults['translation_m']),
                                   (prefix + '_rpy', rpy, defaults['body_rpy_rad'])]:
        try:
            parsed = ast.literal_eval(value) if value else fallback
        except (SyntaxError, ValueError) as error:
            raise ValueError(name + ' requires three finite numbers') from error
        if (not isinstance(parsed, (list, tuple)) or len(parsed) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in parsed)):
            raise ValueError(name + ' requires three finite numbers')
        result[name] = ' '.join(str(float(v)) for v in parsed)
    return result


def validate_mount_parent(description, mappings, camera_type='d435'):
    parent = mappings[camera_type + '_mount_parent']
    links = {link.get('name') for link in ET.fromstring(description).findall('link')}
    if parent.startswith(camera_type + '_') or parent not in links:
        raise ValueError(camera_type + '_mount_parent is not a robot link: ' + parent)
