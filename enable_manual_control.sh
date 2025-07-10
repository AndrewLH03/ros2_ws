#!/bin/bash
# Manual Servo Control Interface
# Simple script to enable manual servo control mode and provide usage instructions

echo "🎮 Manual Servo Control Interface"
echo "=================================="
echo ""

# Check if ROS2 is running
if ! pgrep -f "ros2" > /dev/null; then
    echo "❌ ROS2 nodes are not running. Please start the system first:"
    echo "   cd /home/andrewlh/VSCode/ros2_ws"
    echo "   ./run_all_nodes.sh"
    echo ""
    exit 1
fi

# Switch to manual mode
echo "🔄 Switching to manual mode..."
ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"

sleep 1

echo ""
echo "✅ Manual servo control mode activated!"
echo ""
echo "📖 AVAILABLE COMMANDS:"
echo "======================"
echo ""
echo "🤖 Preset Hand Positions:"
echo "  open     - Open all fingers"
echo "  closed   - Close all fingers" 
echo "  pinch    - Thumb + index pinch"
echo "  point    - Point with index finger"
echo "  peace    - Peace sign (index + middle)"
echo "  rock     - Rock and roll sign"
echo ""
echo "🔧 Individual Finger Control:"
echo "  t 0.5    - Set thumb to 50% curl"
echo "  i 1.0    - Set index to 100% curl"
echo "  m 0.0    - Set middle to 0% curl"
echo "  r 0.8    - Set ring to 80% curl"
echo "  p 0.3    - Set pinky to 30% curl"
echo ""
echo "📊 Status Commands:"
echo "  status   - Show current finger positions"
echo "  help     - Show help in terminal"
echo "  reset    - Reset all fingers to open"
echo "  quit     - Exit manual control"
echo ""
echo "💡 Tips:"
echo "  - Values range from 0.0 (fully open) to 1.0 (fully closed)"
echo "  - Use the UI dashboard to monitor finger positions visually"
echo "  - Emergency stop is available via UI or: ros2 topic pub /emergency_stop std_msgs/msg/Bool 'data: true'"
echo ""
echo "🔄 To switch back to camera tracking mode:"
echo "   ros2 topic pub --once /mode std_msgs/msg/String \"data: 'pose_tracking'\""
echo ""
echo "🎯 Manual control node should now be accepting commands."
echo "   Look for the interactive prompt in the terminal where run_all_nodes.sh is running."
echo ""

# Optional: Check if manual control node is running
if pgrep -f "manual_servo_control_node" > /dev/null; then
    echo "✅ Manual servo control node is running and ready for commands."
else
    echo "⚠️  Manual servo control node may not be running. Check the main terminal."
fi

echo ""
echo "🚀 Ready for manual servo control!"
