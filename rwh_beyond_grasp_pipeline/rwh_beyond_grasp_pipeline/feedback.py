"""Fresh, finite feedback contract for the physical transport."""
import math


def feedback_values(joints, received_at, now, max_age=1.0):
    if joints is None or now - received_at > max_age:
        raise RuntimeError('Physical joint feedback is missing or stale')
    values = dict(zip(joints.name, joints.position))
    if not all(
        name in values and math.isfinite(values[name])
        for name in [*[f'joint{i}' for i in range(1, 7)], 'gripper']
    ):
        raise RuntimeError('Physical arm/gripper positions are missing or nonfinite')
    return values

