import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'buggy_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/kct_osm_network.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VJ',
    maintainer_email='vj@example.com',
    description='KCT Campus Multi-Rider Navigation, Kinematics, and Web Integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager_node = buggy_fleet.fleet_manager_node:main',
            'car_motion_node = buggy_fleet.car_motion_node:main',
            'web_server = buggy_fleet.web_server:main',
            'health_monitor_node = buggy_fleet.health_monitor_node:main',
        ],
    },
)
