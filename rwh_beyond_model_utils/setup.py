from glob import glob
from setuptools import setup

package_name = 'rwh_beyond_model_utils'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='Woosung Yoon', maintainer_email='71277820+ggh-png@users.noreply.github.com',
    description='Shared CPU data conversion and model transport utilities', license='Apache-2.0',
    entry_points={'console_scripts': []},
)
