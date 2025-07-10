from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'servo_control'

setup(
    name='servo_control',  # Use underscores consistently
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 Developer',
    maintainer_email='user@example.com',
    description='Servo control package for Dynamixel XL330-288 servos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_interface_node = servo_control.servo_interface_node:main',
            'manual_servo_control_node = servo_control.manual_servo_control_node:main',
            'direct_servo_tester_node = servo_control.direct_servo_tester_node:main',
        ],
    },
)
