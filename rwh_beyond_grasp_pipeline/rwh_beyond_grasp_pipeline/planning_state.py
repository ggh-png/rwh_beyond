"""Bounded planning-state correction for small physical encoder zero errors."""
import copy
import math
import xml.etree.ElementTree as ET

ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))


def arm_position_limits(robot_description):
    robot = ET.fromstring(robot_description)
    limits = {}
    for name in ARM_JOINTS:
        joint = robot.find(f"joint[@name='{name}']")
        limit = joint.find('limit') if joint is not None else None
        if limit is None:
            raise ValueError(f'MoveIt robot_description is missing limits for {name}')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f'Invalid MoveIt position limits for {name}')
        limits[name] = (lower, upper)
    return limits


def bounded_planning_state(state, limits, tolerance):
    if not math.isfinite(tolerance) or not 0 < tolerance <= .05:
        raise ValueError('planning_start_bounds_tolerance must be in (0, 0.05] rad')
    result = copy.deepcopy(state)
    names = list(result.joint_state.name)
    positions = list(result.joint_state.position)
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError('Invalid physical joint state names/positions')
    corrections = []
    for name in ARM_JOINTS:
        if name not in names:
            raise ValueError(f'Missing physical joint {name}')
        index = names.index(name)
        value = positions[index]
        if not math.isfinite(value):
            raise ValueError(f'Nonfinite physical position for {name}')
        lower, upper = limits[name]
        bounded = min(max(value, lower), upper)
        if abs(bounded-value) > tolerance:
            raise RuntimeError(f'{name} physical position {value:.6f} outside [{lower:.6f}, {upper:.6f}] '
                               f'by more than planning tolerance {tolerance:.6f} rad')
        if bounded != value:
            positions[index] = bounded
            corrections.append(f'{name}: {value:.6f} -> {bounded:.6f}')
    result.joint_state.position = positions
    return result, corrections
