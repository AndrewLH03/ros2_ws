from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'camera_interface'

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
    maintainer='Andrew Holland',
    maintainer_email='AndrewLloydHolland@gmail.com',
    description='Camera interface for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_interface.camera_node:main',
            'camera_info_node = camera_interface.camera_info_node:main',
        ],
    },
)
