"""Render namespace/time overrides while preserving the checked-in Nav2 tuning."""
from copy import deepcopy


def topic(namespace, name):
    if not isinstance(name, str) or not name.strip():
        raise ValueError('Topic names must be nonempty strings')
    return name if name.startswith('/') else '/' + '/'.join(
        part for part in (namespace.strip('/'), name) if part)


def render_parameters(source, namespace, use_sim_time=False, cmd_vel_type=''):
    if cmd_vel_type not in ('', 'twist', 'twist_stamped'):
        raise ValueError('cmd_vel_type must be twist or twist_stamped')
    if not isinstance(source, dict):
        raise ValueError('params_file must contain a node-keyed ROS parameter dictionary')
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''

    def render(value):
        if isinstance(value, str):
            return value.replace('<robot_namespace>', prefix)
        if isinstance(value, list):
            return [render(item) for item in value]
        if not isinstance(value, dict):
            return value
        result = {key: render(item) for key, item in value.items()}
        if 'ros__parameters' in result:
            result['ros__parameters']['use_sim_time'] = bool(use_sim_time)
        if cmd_vel_type and 'enable_stamped_cmd_vel' in result:
            result['enable_stamped_cmd_vel'] = cmd_vel_type == 'twist_stamped'
        return result

    return render(deepcopy(source))
