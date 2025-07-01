from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sim_interface'

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
    description='Simulation interface for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = sim_interface.simulator_node:main',
            'sim_world_interface_node = sim_interface.sim_world_interface_node:main',
        ],
    },
)
