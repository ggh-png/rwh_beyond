from glob import glob
from setuptools import setup
setup(name='rwh_beyond_grasp_bridge', version='0.1.0',
 packages=['rwh_beyond_grasp_bridge'],
 data_files=[('share/ament_index/resource_index/packages', ['resource/rwh_beyond_grasp_bridge']),
 ('share/rwh_beyond_grasp_bridge', ['package.xml', 'README.md']),
 ('share/rwh_beyond_grasp_bridge/launch', glob('launch/*.launch.py')),
 ('share/rwh_beyond_grasp_bridge/config', glob('config/*.yaml'))],
 extras_require={'test': ['pytest']},
 install_requires=['setuptools'], zip_safe=True,
 maintainer='Woosung Yoon', maintainer_email='71277820+ggh-png@users.noreply.github.com',
 description='CPU-only GraspGenX ZMQ ROS 2 bridge', license='Apache-2.0',
 entry_points={'console_scripts': [
 'grasp_coordinator = rwh_beyond_grasp_bridge.grasp_coordinator:main',
 'mock_zmq_server = rwh_beyond_grasp_bridge.mock_zmq_server:main',
 'infer_depth = rwh_beyond_grasp_bridge.infer_depth:main',
 'inference_action_server = rwh_beyond_grasp_bridge.inference_action_server:main',
 'infer_cloud = rwh_beyond_grasp_bridge.infer_cloud:main']})
