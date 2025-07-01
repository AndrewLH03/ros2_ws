from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ui'

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
    description='UI modules for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_switcher_node = ui.mode_switcher_node:main',
            'ui_dashboard_node = ui.ui_dashboard_node_modular:main',
        ],
    },
)
