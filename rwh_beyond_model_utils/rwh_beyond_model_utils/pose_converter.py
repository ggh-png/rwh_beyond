"""Validate SE(3) and convert to xyzw; do not apply TF or a guessed TCP offset."""
import numpy as np

def validate_candidates(response, limit):
    try:
        matrices = np.asarray(response['grasps'], dtype=np.float64)
        scores = np.asarray(response['confidences'], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('missing or nonnumeric grasps/confidences') from error
    if matrices.shape == (0,) and scores.shape == (0,):
        matrices = matrices.reshape(0, 4, 4)
    if matrices.ndim != 3 or matrices.shape[1:] != (4, 4) or scores.shape != (len(matrices),):
        raise ValueError('expected grasps (K,4,4), confidences (K,)')
    if len(matrices) > 10000 or not np.isfinite(matrices).all() or not np.isfinite(scores).all():
        raise ValueError('nonfinite or oversized response')
    if np.any((scores < 0) | (scores > 1)):
        raise ValueError('confidence must be in [0,1]')
    for matrix in matrices:
        rotation = matrix[:3, :3]
        if not np.allclose(matrix[3], [0,0,0,1], atol=1e-5, rtol=0):
            raise ValueError('invalid homogeneous last row')
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3, rtol=0) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-3):
            raise ValueError('rotation must be orthonormal and proper')
    order = np.argsort(-scores, kind='stable')[:limit]
    return matrices[order], scores[order]

def quaternion_xyzw(rotation):
    # Symmetric eigenvector method remains stable at rotations of pi.
    r = np.asarray(rotation)
    k = np.array([
      [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
      [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
      [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
      [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q
