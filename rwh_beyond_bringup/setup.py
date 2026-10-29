from glob import glob
from setuptools import find_packages, setup

setup(name='rwh_beyond_bringup', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/rwh_beyond_bringup']),
                  ('share/rwh_beyond_bringup', ['package.xml']),
                  ('share/rwh_beyond_bringup/launch', glob('launch/*.launch.py')),
                  ('share/rwh_beyond_bringup/params', glob('params/*.yaml')),
                  ('share/rwh_beyond_bringup/params/upstream', glob('params/upstream/*.yaml'))],
      install_requires=['setuptools'], license='Apache-2.0',
      description='RWH Nav2 bringup and complete editable configuration',
      maintainer='Woosung Yoon', maintainer_email='71277820+ggh-png@users.noreply.github.com',
      entry_points={'console_scripts': ['inspect_robot = rwh_beyond_bringup.inspect_robot:main']})
