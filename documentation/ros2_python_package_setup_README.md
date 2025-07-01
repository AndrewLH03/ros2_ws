# ROS 2 Python Package Setup Guide (Complete Working Configuration)

This guide explains **EVERYTHING** needed to set up a ROS 2 Python package so that nodes are discoverable and launchable by both `ros2 run` and `ros2 launch`, based on the proven working configuration of the `camera_interface` package.

## Complete Directory Structure

```
ros2_ws/
├── src/
│   └── camera_interface/                    # Package root directory
│       ├── camera_interface/                # Python package subdirectory (SAME NAME!)
│       │   ├── __init__.py                  # Required for Python package
│       │   ├── camera_node.py               # Node implementation
│       │   └── camera_info_node.py          # Another node implementation
│       ├── resource/                        # Resource directory
│       │   └── camera_interface             # Empty marker file (SAME NAME as package!)
│       ├── package.xml                      # ROS 2 package manifest
│       ├── setup.py                         # Python package setup
│       └── setup.cfg                        # ⭐ CRITICAL: Script installation configuration
└── documentation/
    └── ros2_python_package_setup_README.md
```

## 1. Critical Files Analysis

### `setup.cfg` - THE MISSING PIECE! ⭐⭐⭐
**This file is absolutely critical and often missing from tutorials!**

```ini
[develop]
script_dir = src/camera_interface
[install]
install_scripts = $base/lib/camera_interface
```

**What this does:**
- `install_scripts = $base/lib/camera_interface` tells setuptools to install console scripts into `lib/camera_interface/` instead of the default `bin/`
- This is what makes ROS 2 launch files able to find your executables!
- Without this, executables go to `bin/` and launch files fail with "executable not found"

### `setup.py` - Python Package Configuration

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'camera_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),              # Auto-find Python packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),                 # Resource index
        ('share/' + package_name, ['package.xml']),        # Package manifest
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Camera interface for CR3 robot control system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [                               # ⭐ Creates executables
            'camera_node = camera_interface.camera_node:main',
            'camera_info_node = camera_interface.camera_info_node:main',
        ],
    },
)
```

**Key Points:**
- `find_packages(exclude=['test'])` automatically discovers the `camera_interface/` subdirectory
- `entry_points` creates executable scripts without `.py` extensions
- Combined with `setup.cfg`, these scripts are installed to `lib/camera_interface/`

### `package.xml` - ROS 2 Package Manifest

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>camera_interface</name>                           ⭐ Must match directory name
  <version>0.1.0</version>
  <description>Camera interface for CR3 robot control system</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>       ⭐ Python build system

  <depend>rclpy</depend>                                  ⭐ ROS 2 Python client library
  <depend>sensor_msgs</depend>                            # Add your ROS 2 dependencies
  <depend>cv_bridge</depend>

  <export>
    <build_type>ament_python</build_type>                 ⭐ Declares Python package
  </export>
</package>
```

### Node Implementation Files

**Example: `camera_interface/camera_node.py`**

```python
#!/usr/bin/env python3
"""
Camera Node for CR3 Control System

Captures and publishes image data from the camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraNode(Node):
    """Outline: Camera node for image capture and publishing."""
    
    def __init__(self):
        """Initialize the camera node."""
        super().__init__('camera_node')
        # ... node implementation ...

def main(args=None):                                      # ⭐ REQUIRED main() function
    """Main function to start the camera node."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Critical Requirements:**
- Each node must have a `main(args=None)` function
- Must be importable as `package_name.module_name:main`
- Should follow ROS 2 node lifecycle patterns

### `__init__.py` - Python Package Marker

```python
# Empty file that makes the directory a Python package
# This allows imports like: from camera_interface.camera_node import main
```

### `resource/camera_interface` - Package Marker

```
# Empty file that serves as a package marker for ROS 2
# Filename must match package name exactly
```

## 2. Step-by-Step Setup Process

### Step 1: Create Directory Structure
```bash
mkdir -p src/your_package_name/your_package_name
mkdir -p src/your_package_name/resource
touch src/your_package_name/your_package_name/__init__.py
touch src/your_package_name/resource/your_package_name
```

### Step 2: Create and Configure Files
1. **Create `setup.cfg`** (Most important!)
2. **Create `setup.py`** with proper entry_points
3. **Create `package.xml`** with correct dependencies
4. **Implement node scripts** with main() functions

### Step 3: Build and Test
```bash
cd ~/your_workspace
source /opt/ros/jazzy/setup.bash      # Replace 'jazzy' with your ROS 2 distro
colcon build --symlink-install
source install/setup.bash
ros2 run your_package_name your_node_name
```

### Step 4: Verify Installation
```bash
# Check that executable exists in correct location
ls -la install/your_package_name/lib/your_package_name/

# Should show executable without .py extension
# Example: camera_node (not camera_node.py)
```

## 3. Launch File Configuration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_interface',           # Package name
            executable='camera_node',             # Executable name (from entry_points)
            name='camera_node',                   # Node instance name
            output='screen'
        ),
    ])
```

## 4. Common Mistakes and Solutions

### ❌ Common Mistake 1: Missing `setup.cfg`
**Problem:** Nodes work with `ros2 run` but fail in launch files
**Solution:** Add `setup.cfg` with `install_scripts = $base/lib/package_name`

### ❌ Common Mistake 2: Wrong Directory Structure
**Problem:** Package not found during build
**Solution:** Ensure package subdirectory has the same name as the root directory

### ❌ Common Mistake 3: Missing `main()` Function
**Problem:** Entry point fails to import
**Solution:** Each node must have a `main(args=None)` function

### ❌ Common Mistake 4: Wrong Resource File Name
**Problem:** Package not discovered by ROS 2
**Solution:** Resource file must have exact same name as package

## 5. Verification Checklist

- [ ] `setup.cfg` exists with `install_scripts = $base/lib/package_name`
- [ ] Directory structure: `package_name/package_name/`
- [ ] `__init__.py` exists in Python package subdirectory
- [ ] Resource file exists: `resource/package_name` (empty file)
- [ ] Each node has `main(args=None)` function
- [ ] `entry_points` in `setup.py` matches node filenames
- [ ] `package.xml` has correct `<name>` and dependencies
- [ ] Build succeeds: `colcon build --symlink-install`
- [ ] Executables exist in `install/package_name/lib/package_name/`
- [ ] `ros2 run package_name node_name` works
- [ ] Launch files can find and start nodes

---

## Summary: The Complete Formula

**The working formula for ROS 2 Python packages:**
1. **`setup.cfg`** - Directs executables to `lib/package_name/` (CRITICAL!)
2. **`setup.py`** - Defines entry_points for executables + standard ROS 2 data_files
3. **`package.xml`** - Standard ROS 2 Python package manifest
4. **Directory structure** - `package_name/package_name/` with `__init__.py`
5. **Resource marker** - Empty file `resource/package_name`
6. **Node scripts** - Must have `main(args=None)` functions

**This configuration ensures nodes work with both `ros2 run` and `ros2 launch`!**
