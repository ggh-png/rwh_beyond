#!/usr/bin/env python3
"""Validate lifecycle transitions and the independently generated custom service."""
import os
os.environ['ROS_DOMAIN_ID'] = '180'
import signal
import subprocess
import tempfile
import rclpy
from lifecycle_msgs.srv import ChangeState
from composition_interfaces.srv import LoadNode, UnloadNode
from rwh_beyond_msgs.srv import GetStatus


def main():
    rclpy.init()
    node = rclpy.create_node('rwh_service_verifier')

    def call(kind, endpoint, request):
        client = node.create_client(kind, endpoint)
        try:
            assert client.wait_for_service(timeout_sec=10), endpoint
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), endpoint
            return future.result()
        finally:
            node.destroy_client(client)

    def exercise(prefix):
        for transition, expected in [(None, 'unconfigured'), (1, 'inactive'),
                                     (3, 'active'), (4, 'inactive'), (2, 'unconfigured')]:
            if transition is not None:
                request = ChangeState.Request()
                request.transition.id = transition
                assert call(ChangeState, prefix + '/change_state', request).success
            status = call(GetStatus, prefix + '/get_status', GetStatus.Request())
            assert status.state_label == expected, status
            assert status.active == (expected == 'active'), status
            assert status.node_namespace == '/service_test', status

    commands = [
        ['ros2', 'run', 'rwh_beyond', 'rwh_beyond_server', '--ros-args',
         '-r', '__ns:=/service_test', '-r', '__node:=beyond_server'],
        ['ros2', 'run', 'rclcpp_components', 'component_container',
         '--ros-args', '-r', '__node:=rwh_component_container'],
    ]
    try:
        for index, command in enumerate(commands):
            with tempfile.TemporaryFile(mode='w+') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    if index == 0:
                        exercise('/service_test/beyond_server')
                        print('PASS standalone lifecycle and GetStatus service', flush=True)
                    else:
                        request = LoadNode.Request()
                        request.package_name = 'rwh_beyond'
                        request.plugin_name = 'rwh_beyond::BeyondServer'
                        request.node_name = 'composed_server'
                        request.node_namespace = '/service_test'
                        loaded = call(LoadNode, '/rwh_component_container/_container/load_node', request)
                        assert loaded.success, loaded.error_message
                        exercise('/service_test/composed_server')
                        request = UnloadNode.Request()
                        request.unique_id = loaded.unique_id
                        assert call(UnloadNode, '/rwh_component_container/_container/unload_node', request).success
                        print('PASS component loading, lifecycle service and unloading', flush=True)
                except Exception:
                    log.seek(0)
                    print(log.read())
                    raise
                finally:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
