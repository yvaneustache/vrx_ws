import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vrx_apf_dock_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yvan Eustache',
    maintainer_email='yvan.eustache@gmail.com',
    description='Joystick teleop for tuning vrx_apf_dock PID gains directly',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_setpoint_node = vrx_apf_dock_teleop.joystick_setpoint_node:main',
        ],
    },
)
