#!/bin/bash
# Build and run all nodes for the CR3 Unified Control System

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

while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h)
            show_help
            exit 0
            ;;
        --test-servo)
            TEST_SERVO_ONLY=true
            SKIP_CAMERA=true
            shift
            ;;
        --manual)
            START_MANUAL=true
            shift
            ;;
        --no-camera)
            SKIP_CAMERA=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Source ROS 2 installation first
source /opt/ros/jazzy/setup.bash

# Ensure we use system Python for ROS2 compatibility
export PATH="/usr/bin:$PATH"

# Move to workspace root
cd /home/andrewlh/Desktop/robotics/arm/ros2_ws

# Build the workspace
echo "🔨 Building workspace..."
colcon build

# Source the workspace
echo "📦 Sourcing workspace..."
source install/setup.bash

# Function to test servo control system
test_servo_system() {
    echo ""
    echo "🧪 Testing Servo Control System"
    echo "==============================="
    echo ""
    
    # Start only servo-related nodes for testing
    echo "Starting servo controller..."
    ros2 run servo_control finger_servo_controller_node &
    SERVO_CONTROLLER_PID=$!
    
    echo "Starting servo interface..."
    ros2 run servo_control servo_interface_node &
    SERVO_INTERFACE_PID=$!
    
    echo "Starting manual servo control..."
    ros2 run servo_control manual_servo_control_node &
    MANUAL_SERVO_PID=$!
    
    echo "Starting mode switcher..."
    ros2 run ui mode_switcher_node &
    MODE_SWITCHER_PID=$!
    
    echo "Starting lifecycle action server..."
    ros2 run action_server lifecycle_action_server &
    LIFECYCLE_ACTION_PID=$!
    
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
    echo "Manual control should now be accepting commands in the background."
    echo "Switch to manual mode and try commands like: open, closed, pinch"
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
    echo "🚀 Starting all nodes for hand tracking dashboard system..."
    echo "Press Ctrl+C to stop all nodes"
    echo ""

    # Start camera interface node in background (unless skipped)
    if [[ "$SKIP_CAMERA" != true ]]; then
        echo "📷 Starting camera node..."
        ros2 run camera_interface camera_node &
        CAMERA_PID=$!
        # Wait a moment for camera to initialize
        sleep 2
    else
        echo "⏭️  Skipping camera initialization"
        CAMERA_PID=""
    fi

    # Start perception nodes in background
    echo "🧠 Starting hand pose node..."
    ros2 run perception hand_pose_node &
    HAND_POSE_PID=$!

    echo "🧠 Starting body pose node..."
    ros2 run perception body_pose_node &
    BODY_POSE_PID=$!

    echo "🧠 Starting coordinate transform node..."
    ros2 run perception coordinate_transform_node &
    COORD_TRANSFORM_PID=$!

    # Wait a moment for perception to initialize
    sleep 2

    # Start control nodes in background
    echo "🎛️  Starting pose to command node..."
    ros2 run control pose_to_command_node &
    POSE_COMMAND_PID=$!

    # Start servo control nodes in background
    echo "🦾 Starting finger servo controller..."
    ros2 run servo_control finger_servo_controller_node &
    SERVO_CONTROLLER_PID=$!

    echo "🔧 Starting servo interface (simulation mode)..."
    ros2 run servo_control servo_interface_node &
    SERVO_INTERFACE_PID=$!

    echo "🎮 Starting manual servo control..."
    ros2 run servo_control manual_servo_control_node &
    MANUAL_SERVO_PID=$!

    # Wait a moment for servo control to initialize
    sleep 2

    # Start UI nodes in background
    echo "🔄 Starting mode switcher node..."
    ros2 run ui mode_switcher_node &
    MODE_SWITCHER_PID=$!

    echo "⚙️ Starting lifecycle action server..."
    ros2 run action_server lifecycle_action_server &
    LIFECYCLE_ACTION_PID=$!

    # Set initial mode
    sleep 2
    if [[ "$START_MANUAL" == true ]]; then
        echo "🎮 Switching to manual control mode..."
        ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
    fi

    echo ""
    echo "✅ All nodes started successfully!"
    echo ""
    echo "🎮 Manual Control Quick Commands:"
    echo "  manual    - Switch to manual mode"
    echo "  camera    - Switch to camera mode"
    echo "  emergency - Toggle emergency stop"
    echo "  status    - Show system status"
    echo ""
    
    # Start the main UI dashboard (this should be the foreground process to see the OpenCV window)
    echo "🖥️  Starting UI dashboard (OpenCV window will appear)..."
    echo "Use CYCLE MODE button or 'm' key to switch between manual/camera control"
    echo ""
    
    ros2 run ui ui_dashboard_node
}

# Interactive command handler function
handle_command() {
    case $1 in
        "manual")
            echo "🎮 Switching to manual control mode..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
            echo "Manual control activated! Try commands: open, closed, pinch, t 0.5, etc."
            ;;
        "camera")
            echo "📷 Switching to camera tracking mode..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'pose_tracking'"
            echo "Camera tracking activated!"
            ;;
        "emergency")
            echo "🛑 Toggling emergency stop..."
            ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
            sleep 1
            ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
            ;;
        "test-servos")
            echo "🧪 Running servo test sequence..."
            ros2 topic pub --once /mode std_msgs/msg/String "data: 'manual'"
            sleep 1
            echo "Testing open position..."
            # This would trigger the manual control node to execute "open"
            echo "Check the UI dashboard for visual feedback"
            ;;
        "status")
            echo "📊 System Status:"
            echo "Nodes running:"
            ros2 node list | grep -E "(hand_pose|servo|manual|mode_switcher|ui_dashboard)" || echo "No matching nodes found"
            echo ""
            echo "Active topics:"
            ros2 topic list | grep -E "(mode|servo|finger)" || echo "No matching topics found"
            ;;
        "help")
            echo ""
            echo "🎮 Available Commands:"
            echo "  manual      - Switch to manual servo control"
            echo "  camera      - Switch to camera tracking"
            echo "  emergency   - Toggle emergency stop"
            echo "  status      - Show system status"
            echo "  help        - Show this help"
            echo ""
            echo "In manual mode, try: open, closed, pinch, point, peace, rock"
            echo "Individual control: t 0.5, i 1.0, m 0.0, r 0.8, p 0.3"
            ;;
        *)
            echo "Unknown command: $1"
            echo "Available: manual, camera, emergency, status, help"
            ;;
    esac
}

# Cleanup function to kill all background processes
cleanup() {
    echo ""
    echo "🛑 Stopping all nodes..."
    # Kill all the PIDs we've stored
    for pid in $CAMERA_PID $HAND_POSE_PID $BODY_POSE_PID $COORD_TRANSFORM_PID $POSE_COMMAND_PID $SERVO_CONTROLLER_PID $SERVO_INTERFACE_PID $MANUAL_SERVO_PID $MODE_SWITCHER_PID $LIFECYCLE_ACTION_PID; do
        if [[ -n "$pid" ]]; then
            kill $pid 2>/dev/null || true
        fi
    done
    echo "✅ All nodes stopped."
}

# Set up trap to cleanup on script exit
trap cleanup EXIT

# Main execution logic
if [[ "$TEST_SERVO_ONLY" == true ]]; then
    test_servo_system
else
    run_full_system
fi
