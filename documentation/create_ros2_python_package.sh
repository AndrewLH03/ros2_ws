#!/bin/bash

# ROS 2 Python Package Template Generator
# Creates a new ROS 2 Python package with all required files and correct structure
# Usage: ./create_ros2_python_package.sh <package_name> [maintainer_name] [maintainer_email]

set -e

# Check if package name is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 <package_name> [maintainer_name] [maintainer_email]"
    echo "Example: $0 my_robot_package \"John Doe\" \"john.doe@example.com\""
    exit 1
fi

PACKAGE_NAME=$1
MAINTAINER_NAME=${2:-"Your Name"}
MAINTAINER_EMAIL=${3:-"your.email@example.com"}

echo "🚀 Creating ROS 2 Python package: $PACKAGE_NAME"
echo "📧 Maintainer: $MAINTAINER_NAME <$MAINTAINER_EMAIL>"

# Check if already in src/ directory
if [ ! -d "src" ]; then
    echo "❌ Error: Please run this script from your ROS 2 workspace root (where src/ directory exists)"
    exit 1
fi

# Check if package already exists
if [ -d "src/$PACKAGE_NAME" ]; then
    echo "❌ Error: Package $PACKAGE_NAME already exists!"
    exit 1
fi

echo "📁 Creating directory structure..."

# Create directory structure
mkdir -p "src/$PACKAGE_NAME/$PACKAGE_NAME"
mkdir -p "src/$PACKAGE_NAME/resource"
mkdir -p "src/$PACKAGE_NAME/test"
mkdir -p "src/$PACKAGE_NAME/launch"
mkdir -p "src/$PACKAGE_NAME/config"

# Create __init__.py files
touch "src/$PACKAGE_NAME/$PACKAGE_NAME/__init__.py"
touch "src/$PACKAGE_NAME/test/__init__.py"

# Create resource marker file
touch "src/$PACKAGE_NAME/resource/$PACKAGE_NAME"

echo "📝 Creating configuration files..."

# Create setup.cfg (THE CRITICAL FILE!)
cat > "src/$PACKAGE_NAME/setup.cfg" << EOF
[develop]
script_dir = src/$PACKAGE_NAME
[install]
install_scripts = \$base/lib/$PACKAGE_NAME
EOF

# Create setup.py
cat > "src/$PACKAGE_NAME/setup.py" << EOF
from setuptools import setup, find_packages
import os
from glob import glob

package_name = '$PACKAGE_NAME'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch*.[pP][yY]'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='$MAINTAINER_NAME',
    maintainer_email='$MAINTAINER_EMAIL',
    description='$PACKAGE_NAME package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your nodes here:
            # 'example_node = $PACKAGE_NAME.example_node:main',
        ],
    },
)
EOF

# Create package.xml
cat > "src/$PACKAGE_NAME/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$PACKAGE_NAME</name>
  <version>0.1.0</version>
  <description>$PACKAGE_NAME package for ROS 2</description>
  <maintainer email="$MAINTAINER_EMAIL">$MAINTAINER_NAME</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Core ROS 2 Python dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  
  <!-- Add your additional dependencies here -->
  <!-- <depend>geometry_msgs</depend> -->
  <!-- <depend>sensor_msgs</depend> -->
  <!-- <depend>tf2_ros</depend> -->

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create example node
cat > "src/$PACKAGE_NAME/$PACKAGE_NAME/example_node.py" << EOF
#!/usr/bin/env python3
"""
Example ROS 2 node for $PACKAGE_NAME package.
This is a template - customize it for your specific needs.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExampleNode(Node):
    """Example ROS 2 node that publishes messages."""
    
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Example node initialized')
        
        # Create publisher
        self.publisher = self.create_publisher(
            String, 
            'example_topic', 
            10
        )
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.count = 0
    
    def timer_callback(self):
        """Timer callback to publish messages."""
        msg = String()
        msg.data = f'Hello from {self.get_name()}! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1


def main(args=None):
    """Main function - required for entry_points."""
    rclpy.init(args=args)
    node = ExampleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Create example launch file
cat > "src/$PACKAGE_NAME/launch/$PACKAGE_NAME.launch.py" << EOF
#!/usr/bin/env python3
"""
Launch file for $PACKAGE_NAME package.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for $PACKAGE_NAME."""
    
    return LaunchDescription([
        Node(
            package='$PACKAGE_NAME',
            executable='example_node',
            name='example_node',
            output='screen',
            parameters=[
                # Add your parameters here
                # {'param_name': 'param_value'}
            ],
            remappings=[
                # Add your topic remappings here
                # ('/old_topic', '/new_topic')
            ]
        ),
    ])
EOF

# Create example config file
cat > "src/$PACKAGE_NAME/config/example_config.yaml" << EOF
# Example configuration file for $PACKAGE_NAME
# Customize these parameters for your specific needs

example_node:
  ros__parameters:
    # Example parameters
    rate_hz: 10.0
    message_prefix: "Hello from $PACKAGE_NAME"
    debug_mode: false
    
    # Add your parameters here
EOF

# Create basic test file
cat > "src/$PACKAGE_NAME/test/test_example.py" << EOF
#!/usr/bin/env python3
"""
Basic tests for $PACKAGE_NAME package.
"""

import unittest
import pytest


class TestExample(unittest.TestCase):
    """Basic test class for $PACKAGE_NAME."""
    
    def test_basic(self):
        """Basic test to ensure package imports work."""
        try:
            import $PACKAGE_NAME
            self.assertTrue(True)
        except ImportError:
            self.fail("Failed to import $PACKAGE_NAME")


if __name__ == '__main__':
    unittest.main()
EOF

echo "✅ Package structure created successfully!"
echo ""
echo "📁 Created structure:"
echo "src/$PACKAGE_NAME/"
echo "├── $PACKAGE_NAME/"
echo "│   ├── __init__.py"
echo "│   └── example_node.py"
echo "├── resource/"
echo "│   └── $PACKAGE_NAME"
echo "├── launch/"
echo "│   └── $PACKAGE_NAME.launch.py"
echo "├── config/"
echo "│   └── example_config.yaml"
echo "├── test/"
echo "│   ├── __init__.py"
echo "│   └── test_example.py"
echo "├── package.xml"
echo "├── setup.py"
echo "└── setup.cfg"
echo ""
echo "🔧 Next steps:"
echo "1. Edit src/$PACKAGE_NAME/setup.py to add your nodes to entry_points"
echo "2. Create your node files in src/$PACKAGE_NAME/$PACKAGE_NAME/"
echo "3. Update package.xml with your dependencies"
echo "4. Build and test:"
echo "   colcon build --packages-select $PACKAGE_NAME"
echo "   source install/setup.bash"
echo "   ros2 run $PACKAGE_NAME example_node"
echo ""
echo "📚 Documentation: See documentation/README.md for complete guides"
echo "🔥 Critical: The setup.cfg file is already configured correctly!"
echo ""
echo "🎉 Happy coding!"
EOF

echo "✅ Package '$PACKAGE_NAME' created successfully!"
