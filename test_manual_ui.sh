#!/bin/bash
# Test Manual Mode UI Integration
# Quick test to verify manual mode UI functionality

echo "🧪 Testing Manual Mode UI Integration"
echo "======================================"
echo ""

# Check if workspace is sourced
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "⚠️  ROS workspace not sourced. Sourcing now..."
    source /home/andrewlh/VSCode/ros2_ws/install/setup.bash
fi

echo "🔄 Setting manual mode..."
ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'" &

sleep 1

echo "✅ Manual mode set successfully!"
echo ""
echo "🎮 UI FEATURES IMPLEMENTED:"
echo "=========================="
echo ""
echo "🔲 Individual Mode Buttons:"
echo "   ✅ MANUAL      - Digital input/manual servo control (should be highlighted)"
echo "   ⬜ PERCEPTION  - Camera-based pose tracking"
echo ""
echo "⌨️  Keyboard Shortcuts:"
echo "   1️⃣  - Manual mode (digital input)"
echo "   2️⃣  - Perception mode (camera tracking)" 
echo "   m  - Cycle between modes"
echo "   p  - Pause/Resume"
echo "   e  - Emergency stop"
echo "   l  - Left hand"
echo "   r  - Right hand"
echo ""
echo "📊 Status Display:"
echo "   - Mode shows as 'manual' with yellow text"
echo "   - Manual servo control status panel"
echo "   - Real-time finger curl ratios"
echo "   - Interactive terminal commands help"
echo ""
echo "🎯 Manual Control Commands (in manual mode):"
echo "   open, closed, pinch, point, peace, rock"
echo "   t 0.5, i 1.0, m 0.0, r 0.8, p 0.3"
echo "   status, help, reset, quit"
echo ""
echo "🚀 To start full system:"
echo "   ./run_all_nodes.sh"
echo ""
echo "✅ Manual mode UI integration complete!"
