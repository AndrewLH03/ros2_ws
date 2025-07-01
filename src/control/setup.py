from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Control modules for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_command_node = control.pose_to_command_node:main',
            'motion_planner_node = control.motion_planner_node:main',
            'trajectory_executor_node = control.trajectory_executor_node:main',
            'teleop_node = control.teleop_node:main',
        ],
    },
)
