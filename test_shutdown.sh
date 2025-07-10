#!/bin/bash
# Test Manual Node Shutdown
# Quick test to verify the manual servo control node shuts down properly

echo "🧪 Testing Manual Node Shutdown"
echo "==============================="
echo ""

# Source workspace
source /home/andrewlh/VSCode/ros2_ws/install/setup.bash

echo "🚀 Starting manual servo control node..."
ros2 run servo_control manual_servo_control_node &
MANUAL_PID=$!

echo "🔄 Setting manual mode..."
sleep 2
ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"

echo "⏳ Waiting 5 seconds..."
sleep 5

echo "🛑 Testing shutdown..."
echo "📋 Nodes before shutdown:"
ros2 node list | grep -E "(manual|servo)" || echo "No manual/servo nodes found"

echo ""
echo "⚡ Running shutdown script..."
./shutdown_all_nodes.sh

echo ""
echo "📋 Nodes after shutdown:"
ros2 node list | grep -E "(manual|servo)" || echo "No manual/servo nodes found"

echo ""
echo "🔍 Checking for remaining processes..."
pgrep -f "manual_servo_control_node" || echo "✅ No manual servo control processes found"

echo ""
echo "✅ Shutdown test complete!"
