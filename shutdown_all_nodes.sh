#!/bin/bash
# Smart shutdown script for all ROS2 nodes
# Automatically detects and stops all running ROS2 nodes

# Exit on error
set -e

echo "🛑 Smart ROS2 Node Shutdown Script"
echo "===================================="

# Function to get all running ROS2 nodes
get_running_nodes() {
    # Get list of all nodes except /rosout (system node we don't want to kill)
    ros2 node list 2>/dev/null | grep -v "^/_" | grep -v "^/rosout" || true
}

# Function to get ROS2 related processes
get_ros2_processes() {
    # Find all python processes running ROS2 nodes
    ps aux | grep -E "(ros2 run|python.*\.py)" | grep -v grep | awk '{print $2}' || true
}

# Function to gracefully shutdown nodes
graceful_shutdown() {
    echo "📋 Detecting running ROS2 nodes..."
    
    # Get list of running nodes
    RUNNING_NODES=$(get_running_nodes)
    
    if [ -z "$RUNNING_NODES" ]; then
        echo "✅ No ROS2 nodes detected running"
    else
        echo "🔍 Found running nodes:"
        echo "$RUNNING_NODES" | sed 's/^/  - /'
        echo ""
        
        # Try to shutdown nodes gracefully using ROS2 lifecycle if available
        echo "⏳ Attempting graceful shutdown..."
        
        # Send SIGTERM to all ROS2 related processes
        ROS2_PIDS=$(get_ros2_processes)
        if [ ! -z "$ROS2_PIDS" ]; then
            echo "🔄 Sending SIGTERM to ROS2 processes..."
            echo "$ROS2_PIDS" | xargs -r kill -TERM 2>/dev/null || true
            
            # Wait a moment for graceful shutdown
            sleep 3
            
            # Check if any nodes are still running
            REMAINING_NODES=$(get_running_nodes)
            if [ ! -z "$REMAINING_NODES" ]; then
                echo "⚠️  Some nodes still running, forcing shutdown..."
                REMAINING_PIDS=$(get_ros2_processes)
                if [ ! -z "$REMAINING_PIDS" ]; then
                    echo "$REMAINING_PIDS" | xargs -r kill -KILL 2>/dev/null || true
                fi
            fi
        fi
    fi
}

# Function to kill specific ROS2 processes by name patterns
kill_ros2_processes() {
    echo "🧹 Cleaning up ROS2 processes..."
    
    # Common ROS2 process patterns to kill
    PROCESS_PATTERNS=(
        "camera_node"
        "hand_pose_node"
        "body_pose_node"
        "coordinate_transform_node"
        "pose_to_command_node"
        "finger_servo_controller_node"
        "servo_interface_node"
        "manual_servo_control_node"
        "mode_switcher_node"
        "ui_dashboard_node"
        "ros2 run"
        "ros2 launch"
    )
    
    for pattern in "${PROCESS_PATTERNS[@]}"; do
        PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [ ! -z "$PIDS" ]; then
            echo "  🎯 Killing processes matching '$pattern'"
            echo "$PIDS" | xargs -r kill -TERM 2>/dev/null || true
        fi
    done
    
    # Wait a moment then force kill if needed
    sleep 2
    
    for pattern in "${PROCESS_PATTERNS[@]}"; do
        PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [ ! -z "$PIDS" ]; then
            echo "  ⚡ Force killing remaining '$pattern' processes"
            echo "$PIDS" | xargs -r kill -KILL 2>/dev/null || true
        fi
    done
}

# Function to clean up shared memory and other ROS2 resources
cleanup_resources() {
    echo "🗑️  Cleaning up ROS2 resources..."
    
    # Clean up FastDDS shared memory segments (if they exist)
    if command -v fastdds >/dev/null 2>&1; then
        fastdds shm clean 2>/dev/null || true
    fi
    
    # Clean up any remaining shared memory segments related to ROS2
    # (Be careful here - only clean ROS2 specific ones)
    ipcs -m 2>/dev/null | awk '/^0x/ {print $2}' | head -10 | xargs -r ipcrm -m 2>/dev/null || true
}

# Main shutdown sequence
main() {
    echo "🚀 Starting shutdown sequence..."
    echo ""
    
    # Step 1: Graceful shutdown of ROS2 nodes
    graceful_shutdown
    
    # Step 2: Kill any remaining ROS2 processes
    kill_ros2_processes
    
    # Step 3: Clean up resources
    cleanup_resources
    
    # Step 4: Final verification
    echo ""
    echo "🔍 Final verification..."
    FINAL_NODES=$(get_running_nodes)
    FINAL_PROCESSES=$(get_ros2_processes)
    
    if [ -z "$FINAL_NODES" ] && [ -z "$FINAL_PROCESSES" ]; then
        echo "✅ All ROS2 nodes successfully shutdown!"
    else
        echo "⚠️  Some processes may still be running:"
        if [ ! -z "$FINAL_NODES" ]; then
            echo "  Remaining nodes: $FINAL_NODES"
        fi
        if [ ! -z "$FINAL_PROCESSES" ]; then
            echo "  Remaining processes: $FINAL_PROCESSES"
        fi
        echo "  You may need to manually kill these processes."
    fi
    
    echo ""
    echo "🏁 Shutdown complete!"
}

# Add help option
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "Smart ROS2 Node Shutdown Script"
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -f, --force    Force immediate shutdown (skip graceful)"
    echo "  -v, --verbose  Verbose output"
    echo ""
    echo "This script automatically detects and shuts down all running ROS2 nodes"
    echo "and cleans up associated resources."
    exit 0
fi

# Handle force shutdown
if [ "$1" = "--force" ] || [ "$1" = "-f" ]; then
    echo "⚡ FORCE SHUTDOWN MODE"
    kill_ros2_processes
    cleanup_resources
    echo "✅ Force shutdown complete!"
    exit 0
fi

# Handle verbose mode
if [ "$1" = "--verbose" ] || [ "$1" = "-v" ]; then
    set -x  # Enable verbose bash output
fi

# Run main shutdown sequence
main
