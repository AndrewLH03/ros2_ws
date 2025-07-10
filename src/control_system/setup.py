from setuptools import setup, find_packages

package_name = 'control_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'lifecycle_msgs',
    ],
    zip_safe=True,
    maintainer='andrewlh',
    maintainer_email='andrew@example.com',
    description='Unified control system for all control types (manual, perception, future modes)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_control_node = control_system.unified_control_node:main',
            'manual_control_node = control_system.manual_control_node:main',
            'perception_control_node = control_system.perception_control_node:main',
            'servo_interface_node = control_system.servo_interface_node:main',
            'servo_interface_lifecycle_node = control_system.servo_interface_lifecycle_node:main',
            'mode_manager_node = control_system.mode_manager_node:main',
        ],
    },
)
