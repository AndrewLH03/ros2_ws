from setuptools import setup, find_packages

package_name = 'action_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rclpy_action',
        'lifecycle_msgs',
        'action_interfaces',
    ],
    zip_safe=True,
    maintainer='andrewlh',
    maintainer_email='andrew@example.com',
    description='Action servers for CR3 robot control system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_action_server = action_server.lifecycle_action_server:main',
        ],
    },
)
