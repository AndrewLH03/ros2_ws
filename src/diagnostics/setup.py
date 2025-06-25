from setuptools import setup
import os
from glob import glob

package_name = 'diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Diagnostics modules for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_stop_node = diagnostics.emergency_stop_node:main',
            'error_handler_node = diagnostics.error_handler_node:main',
            'health_monitor_node = diagnostics.health_monitor_node:main',
            'logger_node = diagnostics.logger_node:main',
            'network_monitor_node = diagnostics.network_monitor_node:main',
            'resource_monitor_node = diagnostics.resource_monitor_node:main',
            'system_metrics_node = diagnostics.system_metrics_node:main',
            'watchdog_node = diagnostics.watchdog_node:main',
        ],
    },
)
