#!/usr/bin/env python3
"""
System Validation Script for Enhanced Hand Tracking Integration

This script validates that all nodes, topics, and dependencies are properly configured
and discoverable in the ROS 2 system.
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import sys
from typing import List, Dict, Tuple


class SystemValidator(Node):
    """Node to validate system configuration and functionality."""
    
    def __init__(self):
        super().__init__('system_validator')
        self.validation_results = {}
        
    def run_validation(self):
        """Run comprehensive system validation."""
        self.get_logger().info("Starting system validation...")
        
        # Test 1: Check if all packages can be imported
        self.validate_package_imports()
        
        # Test 2: Check if all nodes can be launched
        self.validate_node_executables()
        
        # Test 3: Check topic availability
        self.validate_topics()
        
        # Test 4: Check dependencies
        self.validate_dependencies()
        
        # Print summary
        self.print_validation_summary()
        
    def validate_package_imports(self):
        """Test if all enhanced Python modules can be imported."""
        self.get_logger().info("Validating package imports...")
        
        test_imports = [
            ('perception.coordinate_transform_node', 'CoordinateTransformNode'),
            ('control.pose_to_command_node', 'PoseToCommandNode'),
            ('ui.ui_dashboard_node', 'UIDashboardNode'),
            ('ui.mode_switcher_node', 'ModeSwitcherNode'),
            ('ui.ui_components', 'draw_button'),
        ]
        
        import_results = {}
        for module_name, class_name in test_imports:
            try:
                module = __import__(module_name, fromlist=[class_name])
                getattr(module, class_name)
                import_results[module_name] = "✅ SUCCESS"
                self.get_logger().info(f"  ✅ {module_name}: Import successful")
            except Exception as e:
                import_results[module_name] = f"❌ FAILED: {str(e)}"
                self.get_logger().error(f"  ❌ {module_name}: {str(e)}")
        
        self.validation_results['imports'] = import_results
    
    def validate_node_executables(self):
        """Test if all node executables are properly registered."""
        self.get_logger().info("Validating node executables...")
        
        expected_executables = [
            ('perception', 'coordinate_transform_node'),
            ('control', 'pose_to_command_node'),
            ('ui', 'ui_dashboard_node'),
            ('ui', 'mode_switcher_node'),
            ('perception', 'hand_pose_node'),
            ('perception', 'body_pose_node'),
        ]
        
        executable_results = {}
        for package, executable in expected_executables:
            try:
                # Test if executable is registered
                result = subprocess.run(
                    ['ros2', 'pkg', 'executables', package],
                    capture_output=True, text=True, timeout=5
                )
                
                if executable in result.stdout:
                    executable_results[f"{package}::{executable}"] = "✅ REGISTERED"
                    self.get_logger().info(f"  ✅ {package}::{executable}: Properly registered")
                else:
                    executable_results[f"{package}::{executable}"] = "❌ NOT REGISTERED"
                    self.get_logger().error(f"  ❌ {package}::{executable}: Not found in package executables")
                    
            except Exception as e:
                executable_results[f"{package}::{executable}"] = f"❌ ERROR: {str(e)}"
                self.get_logger().error(f"  ❌ {package}::{executable}: {str(e)}")
        
        self.validation_results['executables'] = executable_results
    
    def validate_topics(self):
        """Test expected topic structure and message types.""" 
        self.get_logger().info("Validating topic structure...")
        
        expected_topics = {
            # New topics added by our integration
            '/perception/hand_confidence': 'std_msgs/msg/Float32',
            '/perception/shoulder_pose': 'geometry_msgs/msg/Pose',
            '/perception/wrist_pose': 'geometry_msgs/msg/Pose',
            '/perception/tracking_vector': 'geometry_msgs/msg/Vector3',
            '/ui/control_commands': 'std_msgs/msg/String',
            '/ui/visualization_frame': 'sensor_msgs/msg/Image',
            
            # Enhanced existing topics
            '/perception/hand_pose_robot_frame': 'geometry_msgs/msg/PoseArray',
            '/mode': 'std_msgs/msg/String',
            
            # Core existing topics that should still work
            '/camera/image_raw': 'sensor_msgs/msg/Image',
            '/cr3/target_pose': 'geometry_msgs/msg/Pose',
        }
        
        # Note: Topics won't exist until nodes are running, so we validate message type availability
        topic_results = {}
        for topic_name, msg_type in expected_topics.items():
            try:
                # Check if message type is available
                module_parts = msg_type.split('/')
                package = module_parts[0]
                msg_module = module_parts[1] 
                msg_class = module_parts[2]
                
                # Try to import the message type
                module = __import__(f"{package}.{msg_module}", fromlist=[msg_class])
                getattr(module, msg_class)
                
                topic_results[topic_name] = f"✅ MSG TYPE AVAILABLE ({msg_type})"
                self.get_logger().info(f"  ✅ {topic_name}: Message type {msg_type} available")
                
            except Exception as e:
                topic_results[topic_name] = f"❌ MSG TYPE ERROR: {str(e)}"
                self.get_logger().error(f"  ❌ {topic_name}: Message type {msg_type} error: {str(e)}")
        
        self.validation_results['topics'] = topic_results
    
    def validate_dependencies(self):
        """Test if all required dependencies are available."""
        self.get_logger().info("Validating system dependencies...")
        
        python_deps = [
            'cv2',
            'numpy', 
            'rclpy',
            'cv_bridge',
        ]
        
        system_deps = [
            'ros2',
        ]
        
        dep_results = {}
        
        # Test Python dependencies
        for dep in python_deps:
            try:
                __import__(dep)
                dep_results[f"python::{dep}"] = "✅ AVAILABLE"
                self.get_logger().info(f"  ✅ Python dependency {dep}: Available")
            except ImportError as e:
                dep_results[f"python::{dep}"] = f"❌ MISSING: {str(e)}"
                self.get_logger().error(f"  ❌ Python dependency {dep}: Missing")
        
        # Test system dependencies
        for dep in system_deps:
            try:
                result = subprocess.run([dep, '--version'], capture_output=True, timeout=5)
                if result.returncode == 0:
                    dep_results[f"system::{dep}"] = "✅ AVAILABLE"
                    self.get_logger().info(f"  ✅ System dependency {dep}: Available")
                else:
                    dep_results[f"system::{dep}"] = "❌ NOT WORKING"
                    self.get_logger().error(f"  ❌ System dependency {dep}: Not working properly")
            except Exception as e:
                dep_results[f"system::{dep}"] = f"❌ ERROR: {str(e)}"
                self.get_logger().error(f"  ❌ System dependency {dep}: {str(e)}")
        
        self.validation_results['dependencies'] = dep_results
    
    def print_validation_summary(self):
        """Print comprehensive validation summary."""
        print("\n" + "="*80)
        print("SYSTEM VALIDATION SUMMARY")
        print("="*80)
        
        total_tests = 0
        passed_tests = 0
        
        for category, results in self.validation_results.items():
            print(f"\n{category.upper()}:")
            print("-" * 40)
            
            for test_name, result in results.items():
                print(f"  {test_name}: {result}")
                total_tests += 1
                if "✅" in result:
                    passed_tests += 1
        
        print("\n" + "="*80)
        print(f"OVERALL RESULT: {passed_tests}/{total_tests} tests passed")
        
        if passed_tests == total_tests:
            print("🎉 ALL TESTS PASSED - System is ready for deployment!")
            return True
        else:
            print("⚠️  SOME TESTS FAILED - Please address issues before deployment")
            return False


def main():
    """Main validation function."""
    rclpy.init()
    
    validator = SystemValidator()
    
    try:
        success = validator.run_validation()
        
        if success:
            validator.get_logger().info("System validation completed successfully!")
            exit_code = 0
        else:
            validator.get_logger().error("System validation failed - see summary above")
            exit_code = 1
            
    except KeyboardInterrupt:
        validator.get_logger().info("Validation interrupted by user")
        exit_code = 1
    except Exception as e:
        validator.get_logger().error(f"Validation error: {str(e)}")
        exit_code = 1
    finally:
        validator.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
