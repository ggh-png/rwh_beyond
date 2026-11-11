from glob import glob
from setuptools import setup

package_name = 'rwh_beyond_graspgenx_bridge'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='Woosung Yoon', maintainer_email='71277820+ggh-png@users.noreply.github.com',
    description='Independent ROS Action to GraspGenX ZMQ bridge', license='Apache-2.0',
    entry_points={'console_scripts': ['inference_action_server = rwh_beyond_graspgenx_bridge.inference_action_server:main', 'mock_zmq_server = rwh_beyond_graspgenx_bridge.mock_zmq_server:main', 'infer_cloud = rwh_beyond_graspgenx_bridge.infer_cloud:main', 'infer_depth = rwh_beyond_graspgenx_bridge.infer_depth:main', 'infer_rgbd = rwh_beyond_graspgenx_bridge.infer_rgbd:main']},
)
