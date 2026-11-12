"""Home-relative observation geometry; no hardware or ROS side effects."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation


def validate_observation_parameters(home, rise, tilt, settle, tolerance):
    for name, values, size in [('home_joint_positions', home, 6),
                               ('observation_tilt_rpy_deg', tilt, 3)]:
        array = np.asarray(values, dtype=float)
        if array.shape != (size,) or not np.isfinite(array).all():
            raise ValueError(f'{name} requires {size} finite values')
    if not math.isfinite(rise) or rise <= 0:
        raise ValueError('observation_z_offset must be finite and positive')
    if not math.isfinite(settle) or settle < 0:
        raise ValueError('observation_settle_time must be finite and nonnegative')
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('home_joint_tolerance must be finite and positive')


def observation_pose(home_pose, rise, tilt):
    """Translate in base Z, rotate about home-local XYZ axes (degrees)."""
    target = copy.deepcopy(home_pose)
    target.position.z += rise
    q = home_pose.orientation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    q_target = (rotation * Rotation.from_euler('xyz', tilt, degrees=True)).as_quat()
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = map(float, q_target)
    return target
