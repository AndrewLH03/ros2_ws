#!/bin/bash
# Build and run the CR3 Unified Control System

# Exit on error
set -e

# Function to show help
show_help() {
    echo ""
    echo "🎮 CR3 Unified Control System"
    echo "============================="
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help, -h     Show this help message"
    echo "  --test-servo   Test servo control system only"
    echo "  --manual       Start directly in manual control mode"
    echo ""
    echo "Control Modes:"
    echo "  MANUAL         Direct/digital servo control via terminal"
    echo "  PERCEPTION     Camera-based hand tracking control"
    echo ""
    echo "Mode Switching:"
    echo "  - Use UI buttons: MANUAL / PERCEPTION"
    echo "  - Keyboard shortcuts: 1=manual, 2=perception"
    echo "  - ROS topics: ros2 topic pub /mode std_msgs/msg/String \"data: 'manual'\""
    echo ""
    echo "Manual Control Commands (when in manual mode):"
    echo "  open, closed, pinch, point, peace, rock"
    echo "  t 0.5, i 1.0, m 0.0, r 0.8, p 0.3"
    echo "  status, help, reset, quit"
    echo ""
    echo "System Features:"
    echo "  🔄 Automatic node lifecycle management"
    echo "  📹 Camera starts only in perception mode"
    echo "  🎮 Manual control always available"
    echo "  🛑 Emergency stop support"
    echo "  📊 Real-time status monitoring"
    echo ""
}

# Parse command line arguments
START_MANUAL=false
TEST_SERVO_ONLY=false

for arg in "$@"; do
    case $arg in
        --help|-h)
            show_help
            exit 0
            ;;
        --manual)
            START_MANUAL=true
            ;;
        --test-servo)
            TEST_SERVO_ONLY=true
            ;;
        *)
            echo "Unknown option: $arg"
            show_help
            exit 1
            ;;
    esac
done

# Source ROS 2 installation first
source /opt/ros/jazzy/setup.bash

# Move to workspace root
cd ~/VSCode/ros2_ws

# Build the workspace
echo "🔨 Building workspace..."
colcon build

# Source the workspace
echo "📦 Sourcing workspace..."
source install/setup.bash

# Function to run servo test only
run_servo_test() {
    echo ""
    echo "🧪 Starting servo test environment..."
    echo ""
    
    echo "Starting unified control system..."
    ros2 run control_system unified_control_node &
    UNIFIED_CONTROL_PID=$!
    
    echo "Starting servo interface..."
    ros2 run control_system servo_interface_node &
    SERVO_INTERFACE_PID=$!
    
    echo "Starting manual control..."
    ros2 run control_system manual_control_node &
    MANUAL_CONTROL_PID=$!
    
    echo "Starting mode manager..."
    ros2 run control_system mode_manager_node &
    MODE_MANAGER_PID=$!
    
    sleep 3
    
    echo ""
    echo "✅ Servo test environment ready!"
    echo ""
    echo "🎮 Available test commands:"
    echo "  ros2 topic pub --once /mode std_msgs/msg/String \"data: 'manual'\""
    echo "  ros2 topic echo /servo/commands"
    echo "  ros2 topic echo /servo/positions" 
    echo "  ros2 topic echo /perception/finger_curl_ratios"
    echo ""
    echo "Manual control should now be accepting commands."
    echo "Try commands like: open, closed, pinch, t 0.5, etc."
    echo ""
    echo "Press Ctrl+C to stop test environment..."
    
    # Switch to manual mode
    sleep 2
    ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
    
    # Wait for user to stop
    wait
}

# Function to run full system
run_full_system() {
    echo ""
    echo "🚀 Starting unified control system..."
    echo "Press Ctrl+C to stop all nodes"
    echo ""

    # Start core control system nodes
    echo "🎛️  Starting unified control node..."
    ros2 run control_system unified_control_node &
    UNIFIED_CONTROL_PID=$!

    echo "🔧 Starting servo interface..."
    ros2 run control_system servo_interface_node &
    SERVO_INTERFACE_PID=$!

    echo "🎮 Starting manual control node..."
    ros2 run control_system manual_control_node &
    MANUAL_CONTROL_PID=$!

    echo "🔄 Starting mode manager (with node lifecycle management)..."
    ros2 run control_system mode_manager_node &
    MODE_MANAGER_PID=$!

    # Wait for core system to initialize
    sleep 3

    # Start diagnostics and UI
    echo "🔍 Starting health monitor..."
    ros2 run diagnostics health_monitor_node &
    HEALTH_MONITOR_PID=$!

    echo "📊 Starting logger..."
    ros2 run diagnostics logger_node &
    LOGGER_PID=$!

    # Set initial mode
    if [[ "$START_MANUAL" == true ]]; then
        echo "🎮 Setting initial mode to manual..."
        ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
    else
        echo "🎮 Starting in manual mode (default)..."
        ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
    fi

    sleep 2

    echo ""
    echo "✅ Unified control system started successfully!"
    echo ""
    echo "🎮 Control Modes:"
    echo "  MANUAL     - Direct servo control via terminal commands"
    echo "  PERCEPTION - Camera-based hand tracking (starts camera automatically)"
    echo ""
    echo "🔄 Mode Switching:"
    echo "  - UI buttons: MANUAL / PERCEPTION"
    echo "  - Keyboard: 1=manual, 2=perception"
    echo "  - Terminal: manual / perception commands below"
    echo ""
    
    # Start the main UI dashboard (foreground process)
    echo "🖥️  Starting UI dashboard (OpenCV window will appear)..."
    echo "Camera will start automatically when switching to PERCEPTION mode"
    echo ""
    
    ros2 run ui ui_dashboard_node
}

# Interactive command handler function
handle_command() {
    case $1 in
        "manual")
            echo "🎮 Switching to manual control mode..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
            echo "Manual control activated! Camera will stop automatically."
            echo "Try commands in the manual control terminal: open, closed, pinch, etc."
            ;;
        "perception")
            echo "📷 Switching to perception mode..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'perception'"
            echo "Perception mode activated! Camera will start automatically."
            ;;
        "emergency")
            echo "🛑 Activating emergency stop..."
            ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
            sleep 1
            echo "🔄 Releasing emergency stop..."
            ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
            ;;
        "test-servos")
            echo "🧪 Running servo test sequence..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
            sleep 1
            echo "Manual mode activated for testing. Use terminal commands for servo control."
            ;;
        "status")
            echo "📊 System Status:"
            echo "  Running nodes:"
            ros2 node list 2>/dev/null | grep -v "/rosout" || echo "    No nodes detected"
            echo "  Active topics:"
            ros2 topic list 2>/dev/null | grep -E "(mode|servo|finger)" || echo "    No relevant topics"
            ;;
        "help")
            echo "🎮 Available Commands:"
            echo "  manual      - Switch to manual control mode"
            echo "  perception  - Switch to perception (camera) mode"
            echo "  emergency   - Toggle emergency stop"
            echo "  test-servos - Start servo testing"
            echo "  status      - Show system status"
            echo "  help        - Show this help"
            ;;
        *)
            echo "❌ Unknown command: $1"
            echo "Type 'help' for available commands"
            ;;
    esac
}

# Main execution logic
if [[ "$TEST_SERVO_ONLY" == true ]]; then
    run_servo_test
else
    run_full_system
fi

# Cleanup function for Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Shutting down all nodes..."
    
    # Use the shutdown script if available
    if [ -f "./shutdown_all_nodes.sh" ]; then
        ./shutdown_all_nodes.sh
    else
        # Manual cleanup
        echo "Killing all ROS2 processes..."
        pkill -f "ros2 run" || true
        pkill -f "control_system" || true
        pkill -f "ui" || true
        pkill -f "diagnostics" || true
    fi
    
    echo "✅ Shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo ""
echo "🎮 CR3 Unified Control System Ready!"
echo ""
echo "Available terminal commands while system is running:"
echo "  manual, perception, emergency, test-servos, status, help"
echo ""

# Interactive command loop for background operation
if [[ "$TEST_SERVO_ONLY" != true ]]; then
    while true; do
        echo -n "Command: "
        read -r cmd
        handle_command "$cmd"
    done
fi
