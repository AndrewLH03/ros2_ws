from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cr3_interface'

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
    description='CR3 robot interface modules',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr3_controller_node = cr3_interface.cr3_controller_node:main',
            'hand_controller_node = cr3_interface.hand_controller_node:main',
            'joint_state_publisher_node = cr3_interface.joint_state_publisher_node:main',
            'tf_broadcaster_node = cr3_interface.tf_broadcaster_node:main',
        ],
    },
)
