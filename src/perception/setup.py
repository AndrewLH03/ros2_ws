from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'perception'

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
    description='Perception modules for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_pose_node = perception.hand_pose_node:main',
            'body_pose_node = perception.body_pose_node:main',
            'pose_filter_node = perception.pose_filter_node:main',
            'coordinate_transform_node = perception.coordinate_transform_node:main',
        ],
    },
)
