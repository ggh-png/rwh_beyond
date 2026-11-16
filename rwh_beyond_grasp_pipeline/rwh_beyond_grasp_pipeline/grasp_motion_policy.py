"""Grasp symmetry preference and MoveIt wrist path constraints."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import Constraints, JointConstraint
from .red_perception import pose_matrix

DEFAULTS = {
    'moveit_pregrasp_planner_id': 'RRTConnect',
    'moveit_pregrasp_num_planning_attempts': 3,
    'moveit_prefer_current_grasp_orientation': True,
    'moveit_pregrasp_wrist_constraint_enabled': True,
    'moveit_pregrasp_wrist_max_deviation_deg': 100.0,
}


def validate_policy(values):
    result = {**DEFAULTS, **values}
    for name in ('moveit_prefer_current_grasp_orientation',
                 'moveit_pregrasp_wrist_constraint_enabled'):
        if type(result[name]) is not bool:
            raise ValueError(name + ' must be a YAML boolean')
    name = 'moveit_pregrasp_planner_id'
    if not isinstance(result[name], str) or not result[name].strip():
        raise ValueError(name + ' must be a nonempty MoveIt planner ID')
    name = 'moveit_pregrasp_num_planning_attempts'
    if type(result[name]) is not int or not 1 <= result[name] <= 10:
        raise ValueError(name + ' must be an integer in [1, 10]')
    name = 'moveit_pregrasp_wrist_max_deviation_deg'
    value = result[name]
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value < 180:
        raise ValueError(name + ' must be finite in (0, 180) degrees')
    return result


def ordered_variants(candidates, base_T_camera, model_T_tcp, current_rotation=None):
    variants = []
    for index, candidate in enumerate(candidates):
        grasp = base_T_camera @ pose_matrix(candidate.pose.pose)
        for flipped in (False, True):
            g = grasp.copy()
            if flipped:
                g[:3, :3] = g[:3, :3] @ Rotation.from_euler('z', math.pi).as_matrix()
            angle = (0.0 if current_rotation is None else
                     float(Rotation.from_matrix(current_rotation.T @
                           (g @ model_T_tcp)[:3, :3]).magnitude()))
            variants.append((angle, index, candidate, flipped, g))
    # Stable ordering retains model score order for equally oriented grasps.
    return sorted(variants, key=lambda entry: entry[0])


def wrist_constraints(start, deviation_deg):
    positions = dict(zip(start.joint_state.name, start.joint_state.position))
    tolerance = math.radians(deviation_deg)
    return Constraints(name='pregrasp_wrist_rotation_limit', joint_constraints=[
        JointConstraint(joint_name=name, position=positions[name],
                        tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0)
        for name in ('joint4', 'joint6')])


def within_wrist_limits(state, constraints):
    positions = dict(zip(state.joint_state.name, state.joint_state.position))
    return all(c.position - c.tolerance_below <= positions[c.joint_name] <=
               c.position + c.tolerance_above for c in constraints.joint_constraints)


def trajectory_within_wrist_limits(trajectory, constraints):
    names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    if not points:
        return False
    for c in constraints.joint_constraints:
        if c.joint_name not in names:
            return False
        i = names.index(c.joint_name)
        if any(len(p.positions) != len(names) or not math.isfinite(p.positions[i]) or
               not c.position - c.tolerance_below <= p.positions[i] <=
               c.position + c.tolerance_above for p in points):
            return False
    return True


def trajectory_reaches_state(trajectory, goal, tolerance=0.002):
    names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    if not points or len(points[-1].positions) != len(names):
        return False
    final = dict(zip(names, points[-1].positions))
    desired = dict(zip(goal.joint_state.name, goal.joint_state.position))
    return all(name in final and math.isfinite(final[name]) and
               abs(final[name] - desired[name]) <= tolerance
               for name in (f'joint{i}' for i in range(1, 7)))
