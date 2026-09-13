from glob import glob
from setuptools import setup

setup(name='delivery_ros', version='0.1.0', packages=['delivery_ros'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/delivery_ros']),
                  ('share/delivery_ros', ['package.xml']),
                  ('share/delivery_ros/launch', glob('launch/*.py')),
                  ('share/delivery_ros/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='UAV team', maintainer_email='maintainer@example.com',
      description='ROS 2 Humble PX4 companion', license='UNLICENSED',
      entry_points={'console_scripts': [
          'vision = delivery_ros.vision:main',
          'px4_adapter = delivery_ros.px4:main',
          'payload = delivery_ros.payload:main',
          'mission = delivery_ros.mission:main']})
