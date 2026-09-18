import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vrx_apf_dock'

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
    description='Docking controller for a VRX WAM-V using artificial potential fields',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_estimator_node = vrx_apf_dock.state_estimator_node:main',
            'apf_controller_node = vrx_apf_dock.apf_controller_node:main',
            'thrust_mixer_node = vrx_apf_dock.thrust_mixer_node:main',
            'vector_marker_node = vrx_apf_dock.vector_marker_node:main',
            'apf_field_marker_node = vrx_apf_dock.apf_field_marker_node:main',
        ],
    },
)
