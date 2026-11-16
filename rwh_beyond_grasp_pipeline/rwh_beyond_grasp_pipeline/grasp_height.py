"""Filter final grasp TCP height above the observed target top, in base Z."""
import math
import numpy as np

DEFAULTS = {
    'grasp_height_filter_enabled': False,
    'grasp_height_above_object_min_m': .05,
    'grasp_height_above_object_max_m': .10,
}


def validate_height_parameters(values):
    settings = {**DEFAULTS, **values}
    enabled = settings['grasp_height_filter_enabled']
    lower = settings['grasp_height_above_object_min_m']
    upper = settings['grasp_height_above_object_max_m']
    if type(enabled) is not bool:
        raise ValueError('grasp_height_filter_enabled must be boolean')
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (lower, upper))
            or lower < 0 or upper < lower):
        raise ValueError('Grasp height range requires finite 0 <= min <= max')
    return (float(lower), float(upper)) if enabled else None


def observed_top_z(xyz, base_T_camera):
    points = np.asarray(xyz, dtype=float)
    transform = np.asarray(base_T_camera, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 100
            or not np.isfinite(points).all() or transform.shape != (4, 4)
            or not np.isfinite(transform).all()):
        raise ValueError('Insufficient finite geometry for object top height')
    base_z = points @ transform[2, :3] + transform[2, 3]
    return float(np.quantile(base_z, .99))


def in_height_band(tcp_z, object_top_z, band):
    if not math.isfinite(tcp_z) or not math.isfinite(object_top_z):
        return False
    height = tcp_z - object_top_z
    return band[0] - 1e-9 <= height <= band[1] + 1e-9
