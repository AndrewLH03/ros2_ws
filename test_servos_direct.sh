#!/bin/bash
# Direct Servo Testing Launcher
# Advanced servo testing with raw position control

echo "🔧 Direct Servo Testing Mode"
echo "============================="
echo ""

# Check if ROS2 is running
if ! pgrep -f "ros2" > /dev/null; then
    echo "❌ ROS2 nodes are not running. Please start the system first:"
    echo "   cd /home/andrewlh/VSCode/ros2_ws"
    echo "   ./run_all_nodes.sh"
    echo ""
    exit 1
fi

echo "⚠️  WARNING: This is direct servo position control!"
echo "   Make sure servos are properly connected and powered."
echo "   This bypasses safety limits of the finger curl system."
echo ""
echo "🔧 Features:"
echo "   - Raw servo position control (0-4095)"
echo "   - Preset configurations (open, closed, pinch, etc.)"
echo "   - Individual servo testing"
echo "   - Position range safety checks"
echo ""

read -p "❓ Continue with direct servo testing? (y/N): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Direct servo testing cancelled."
    exit 0
fi

echo ""
echo "🚀 Starting direct servo tester..."
echo "   Use 'help' command for full usage instructions"
echo "   Use Ctrl+C to exit safely"
echo ""

# Build the package first to ensure latest changes
cd /home/andrewlh/VSCode/ros2_ws
colcon build --packages-select servo_control --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ Package built successfully"
    source install/setup.bash
    echo ""
    echo "🎯 Launching direct servo tester..."
    ros2 run servo_control direct_servo_tester
else
    echo "❌ Failed to build servo_control package"
    exit 1
fi
