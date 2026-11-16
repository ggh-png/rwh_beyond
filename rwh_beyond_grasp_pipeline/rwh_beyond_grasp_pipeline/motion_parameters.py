"""Validated startup settings for all coordinator-generated MoveIt requests."""
import math

DEFAULTS = {
    'moveit_velocity_scaling_factor': 0.1,
    'moveit_acceleration_scaling_factor': 0.1,
    'moveit_cartesian_velocity_scaling_factor': 0.1,
    'moveit_cartesian_acceleration_scaling_factor': 0.1,
    'moveit_planning_time': 10.0,
    'moveit_candidate_planning_time': 2.0,
    'moveit_ik_timeout': 0.1,
}


def validate_motion_parameters(values):
    result = {}
    for name, default in DEFAULTS.items():
        value = values.get(name, default)
        upper = 1.0 if 'scaling_factor' in name else (5.0 if name.endswith('ik_timeout') else 60.0)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f'{name} must be finite in (0, {upper}]')
        result[name] = float(value)
    return result


def motion_value(node, name):
    # Bare mixin consumers retain the established defaults; ROS coordinators
    # populate the validated settings once at startup from the shared YAML.
    return getattr(node, 'motion_parameters', DEFAULTS)[name]
