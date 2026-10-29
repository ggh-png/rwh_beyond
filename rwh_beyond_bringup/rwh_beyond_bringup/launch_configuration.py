"""Resolve arguments shared by standalone and nested navigation launches."""
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def prepare_navigation(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    actions = [SetLaunchConfiguration('namespace', namespace),
               SetLaunchConfiguration('_nav_topic_prefix', '/' + namespace if namespace else '')]
    for name in ('use_composition', 'use_respawn'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        actions.append(SetLaunchConfiguration(name, value.title()))
    return actions
