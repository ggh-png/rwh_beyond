"""RGB-D perception for the one-red-box simulation task, without simulator poses."""

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_erosion
from geometry_msgs.msg import Pose


def red_mask(image):
    if (
        image.encoding not in ('rgb8', 'bgr8')
        or image.step < image.width * 3
        or len(image.data) != image.step * image.height
    ):
        raise ValueError('expected valid rgb8/bgr8 image')
    rgb = np.ndarray(
        (image.height, image.width, 3),
        np.uint8,
        buffer=bytes(image.data),
        strides=(image.step, 3, 1),
    ).astype(float)
    if image.encoding == 'bgr8':
        rgb = rgb[:, :, ::-1]
    mask = (
        (rgb[:, :, 0] > 80)
        & (rgb[:, :, 0] > 1.8 * rgb[:, :, 1])
        & (rgb[:, :, 0] > 1.8 * rgb[:, :, 2])
    )
    return binary_erosion(mask, structure=np.ones((3, 3)), iterations=2)


def pose_matrix(pose):
    q = pose.orientation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    p = pose.position
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def matrix_pose(matrix):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(
        float, q
    )
    return result


def set_pair(matrix, first, second, allowed):
    if len(matrix.entry_names) != len(matrix.entry_values) or any(
        len(r.enabled) != len(matrix.entry_names) for r in matrix.entry_values
    ):
        raise ValueError('invalid ACM')
    from moveit_msgs.msg import AllowedCollisionEntry

    for name in (first, second):
        if name not in matrix.entry_names:
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_names.append(name)
            matrix.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names))
            )
    i, j = matrix.entry_names.index(first), matrix.entry_names.index(second)
    matrix.entry_values[i].enabled[j] = matrix.entry_values[j].enabled[i] = bool(allowed)
