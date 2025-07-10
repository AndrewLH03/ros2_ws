#!/usr/bin/env python3
"""
Manual Servo Control Node

Provides manual control interface for servo testing and calibration.
Can be used instead of camera-based finger tracking for direct servo control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np
import threading
import time

class ManualServoControlNode(Node):
    """
    Manual servo control interface for testing and calibration.
    
    Subscribes to:
    - /mode - Control mode (manual, pose_tracking, etc.)
    - /emergency_stop - Emergency stop signal
    - /manual/servo_direct_commands - Direct servo position commands
    
    Publishes to:
    - /perception/finger_curl_ratios - Simulated finger curl data for manual control
    - /manual/servo_status - Manual control status
    
    Features:
    - Interactive terminal control
    - Preset finger positions (open, closed, pinch, etc.)
    - Individual finger control
    - Safety limits and emergency stop
    """
    
    def __init__(self):
        super().__init__('manual_servo_control_node')
        
        # Control state
        self.control_mode = "idle"
        self.emergency_stop = False
        self.manual_mode_active = False
        self.shutdown_requested = False  # Add shutdown flag
        
        # Finger configuration
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        # Current manual finger curl values (0.0 = open, 1.0 = closed)
        self.manual_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Preset hand positions
        self.hand_presets = {
            'open': [0.0, 0.0, 0.0, 0.0, 0.0],
            'closed': [1.0, 1.0, 1.0, 1.0, 1.0],
            'pinch': [0.8, 0.9, 0.0, 0.0, 0.0],  # Thumb + index
            'point': [0.0, 0.0, 1.0, 1.0, 1.0],  # Index pointing
            'peace': [0.0, 0.0, 0.0, 1.0, 1.0],  # Index + middle
            'rock': [0.0, 0.0, 1.0, 1.0, 0.0],   # Rock and roll
        }
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/mode',
            self.handle_mode_change,
            5)
            
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.handle_emergency_stop,
            5)
            
        self.direct_commands_sub = self.create_subscription(
            Float32MultiArray,
            '/manual/servo_direct_commands',
            self.handle_direct_commands,
            10)
        
        # Publishers
        self.finger_curl_pub = self.create_publisher(
            Float32MultiArray,
            '/perception/finger_curl_ratios',
            10)
            
        self.manual_status_pub = self.create_publisher(
            String,
            '/manual/servo_status',
            10)
        
        # Control timer for publishing manual finger data
        self.control_timer = self.create_timer(0.1, self.publish_manual_finger_data)  # 10Hz
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Start interactive control in separate thread
        self.control_thread = threading.Thread(target=self.interactive_control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info('Manual servo control node started')
        self.get_logger().info('Available modes: manual (for direct control)')
        self.print_help()

    def handle_mode_change(self, msg):
        """Handle control mode changes."""
        new_mode = msg.data
        if new_mode != self.control_mode:
            self.get_logger().info(f'Control mode changed from {self.control_mode} to {new_mode}')
            self.control_mode = new_mode
            
            if new_mode == "manual":
                self.manual_mode_active = True
                self.get_logger().info('🎮 Manual servo control activated!')
                self.print_help()
            else:
                self.manual_mode_active = False
                self.get_logger().info('Manual servo control deactivated')

    def handle_emergency_stop(self, msg):
        """Handle emergency stop signal."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('🛑 Emergency stop activated - manual control disabled')
            # Reset to safe positions
            self.manual_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.get_logger().info('✅ Emergency stop released - manual control enabled')

    def handle_direct_commands(self, msg):
        """Handle direct servo position commands (for advanced users)."""
        if len(msg.data) == 5 and self.manual_mode_active and not self.emergency_stop:
            # Convert servo positions (0-4095) to curl ratios (0-1)
            curl_ratios = []
            servo_ranges = [(1200, 2800), (1000, 3000), (1000, 3000), (1000, 3000), (1100, 2900)]
            
            for i, servo_pos in enumerate(msg.data):
                min_pos, max_pos = servo_ranges[i]
                curl_ratio = (servo_pos - min_pos) / (max_pos - min_pos)
                curl_ratio = np.clip(curl_ratio, 0.0, 1.0)
                curl_ratios.append(curl_ratio)
            
            self.manual_finger_curls = np.array(curl_ratios)
            self.get_logger().info(f'Direct commands received: {[f"{c:.2f}" for c in curl_ratios]}')

    def publish_manual_finger_data(self):
        """Publish manual finger curl data when in manual mode."""
        if self.manual_mode_active and not self.emergency_stop:
            curl_msg = Float32MultiArray()
            curl_msg.data = self.manual_finger_curls.tolist()
            self.finger_curl_pub.publish(curl_msg)

    def publish_status(self):
        """Publish manual control status."""
        status_info = {
            'manual_active': self.manual_mode_active,
            'control_mode': self.control_mode,
            'emergency_stop': self.emergency_stop,
            'finger_curls': [f"{c:.2f}" for c in self.manual_finger_curls]
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.manual_status_pub.publish(status_msg)

    def print_help(self):
        """Print help information for manual control."""
        help_text = """
🎮 MANUAL SERVO CONTROL COMMANDS:
═══════════════════════════════════
📖 Presets:
  'open'   - Open all fingers
  'closed' - Close all fingers  
  'pinch'  - Thumb + index pinch
  'point'  - Point with index finger
  'peace'  - Peace sign (index + middle)
  'rock'   - Rock and roll sign

🔧 Individual Control:
  't 0.5'  - Set thumb to 50% curl
  'i 1.0'  - Set index to 100% curl
  'm 0.0'  - Set middle to 0% curl
  'r 0.8'  - Set ring to 80% curl
  'p 0.3'  - Set pinky to 30% curl

📊 Commands:
  'status' - Show current finger positions
  'help'   - Show this help
  'reset'  - Reset all fingers to open
  'quit'   - Exit manual control

💡 Values: 0.0 = fully open, 1.0 = fully closed
⚠️  Emergency stop: /emergency_stop topic
═══════════════════════════════════
"""
        print(help_text)

    def interactive_control_loop(self):
        """Interactive control loop for manual servo control."""
        while rclpy.ok() and not self.shutdown_requested:
            if self.manual_mode_active and not self.emergency_stop:
                try:
                    print(f"\n🎮 Manual Control [{self.finger_names}]: {[f'{c:.1f}' for c in self.manual_finger_curls]}")
                    command = input("Enter command: ").strip().lower()
                    
                    if command in self.hand_presets:
                        # Apply preset
                        self.manual_finger_curls = np.array(self.hand_presets[command])
                        self.get_logger().info(f"Applied preset: {command}")
                        
                    elif command.startswith(('t ', 'i ', 'm ', 'r ', 'p ')):
                        # Individual finger control
                        parts = command.split()
                        if len(parts) == 2:
                            finger_map = {'t': 0, 'i': 1, 'm': 2, 'r': 3, 'p': 4}
                            finger_idx = finger_map.get(parts[0])
                            try:
                                value = float(parts[1])
                                value = np.clip(value, 0.0, 1.0)
                                self.manual_finger_curls[finger_idx] = value
                                finger_name = self.finger_names[finger_idx]
                                self.get_logger().info(f"Set {finger_name} to {value:.2f}")
                            except ValueError:
                                print("❌ Invalid value. Use 0.0 to 1.0")
                        
                    elif command == 'status':
                        print(f"📊 Current positions:")
                        for i, name in enumerate(self.finger_names):
                            print(f"  {name}: {self.manual_finger_curls[i]:.2f}")
                            
                    elif command == 'help':
                        self.print_help()
                        
                    elif command == 'reset':
                        self.manual_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
                        self.get_logger().info("Reset all fingers to open position")
                        
                    elif command == 'quit':
                        self.shutdown_requested = True
                        break
                        
                    elif command == '':
                        continue  # Ignore empty input
                        
                    else:
                        print("❌ Unknown command. Type 'help' for available commands.")
                        
                except EOFError:
                    self.shutdown_requested = True
                    break
                except KeyboardInterrupt:
                    self.shutdown_requested = True
                    break
                except Exception as e:
                    print(f"❌ Error: {e}")
            else:
                time.sleep(1)  # Wait when not in manual mode
        
        self.get_logger().info("Interactive control loop shutting down")

    def shutdown_node(self):
        """Properly shutdown the node and its threads."""
        self.get_logger().info("Shutting down manual servo control node...")
        self.shutdown_requested = True
        
        # Wait for the control thread to finish (with timeout)
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
            if self.control_thread.is_alive():
                self.get_logger().warn("Control thread did not shutdown gracefully")
        
        self.get_logger().info("Manual servo control node shutdown complete")

def main(args=None):
    rclpy.init(args=args)
    node = ManualServoControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual servo control node stopped by user')
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
