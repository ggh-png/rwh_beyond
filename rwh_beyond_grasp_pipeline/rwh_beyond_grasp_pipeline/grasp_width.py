"""Observed aperture estimates; these are not neural-model width predictions."""
import math
import numpy as np


def estimate_aperture(xyz, camera_T_grasp, max_opening, inset=0.002):
    points = np.asarray(xyz, dtype=float)
    transform = np.asarray(camera_T_grasp, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 100
            or not np.isfinite(points).all() or transform.shape != (4, 4)
            or not np.isfinite(transform).all()):
        raise ValueError('Insufficient finite geometry for grasp width')
    if (not math.isfinite(max_opening) or not 0 < max_opening <= .10
            or not math.isfinite(inset) or not 0 <= inset <= .005):
        raise ValueError('Invalid aperture limit or closing inset')
    # GraspGen convention: local X is finger closing, local Z is approach.
    # Use the whole segmented object, accounting for off-centre grasp axes.
    local_x = (points - transform[:3, 3]) @ transform[:3, 0]
    lower, upper = np.quantile(local_x, [.02, .98])
    if upper - lower < .008 or lower >= 0 or upper <= 0:
        raise ValueError('Object width is degenerate or does not straddle the grasp axis')
    width = float(2 * max(abs(lower), abs(upper)))
    if width < .008:
        raise ValueError('Observed grasp width is too small')
    # Preserve the observation for diagnostics; saturate the actuator target.
    return width, min(max_opening, width - inset)
