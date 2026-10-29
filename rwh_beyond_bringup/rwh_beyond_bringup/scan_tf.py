"""Optional static transform for the frame carried by LaserScan messages."""
import math


def transform_arguments(robot):
    config = robot.get('scan_static_tf', {})
    enabled = config.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('scan_static_tf.enabled must be a YAML boolean')
    if not enabled:
        return None
    parent = config.get('parent_frame', '')
    child = robot.get('scan_frame', '')
    for frame in (parent, child):
        if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
            raise ValueError('Scan TF frames must be nonempty IDs without leading slash or whitespace')
    if parent == child:
        raise ValueError('Scan TF parent and child must differ')
    translation = config.get('translation')
    rotation = config.get('rotation_rpy')
    for name, values in (('translation', translation), ('rotation_rpy', rotation)):
        if not isinstance(values, list) or len(values) != 3 or any(
                not isinstance(v, (float, int)) or isinstance(v, bool) or not math.isfinite(v) for v in values):
            raise ValueError('Scan TF ' + name + ' must have three finite numbers')
    return [item for flag, value in zip(
        ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), translation + rotation)
        for item in (flag, str(value))] + ['--frame-id', parent, '--child-frame-id', child]


def make_scan_tf_node(robot, namespace, use_sim_time):
    arguments = transform_arguments(robot)
    if arguments is None:
        return []
    from launch_ros.actions import Node
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='rwh_scan_static_tf', namespace=namespace, output='screen',
                 arguments=arguments, parameters=[{'use_sim_time': use_sim_time}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])]
