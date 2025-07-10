#!/usr/bin/env python3
"""
Manual Control Node

Provides manual/digital input control for servo testing and direct control.
Part of the unified control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np
import threading
import time


class ManualControlNode(Node):
    """
    Manual control interface for direct servo control.
    
    Subscribes to:
    - /mode - Control mode (manual, perception, etc.)
    - /emergency_stop - Emergency stop signal
    
    Publishes to:
    - /perception/finger_curl_ratios - Manual finger curl data
    - /manual/status - Manual control status
    """
    
    def __init__(self):
        super().__init__('manual_control_node')
        
        # Control state
        self.current_mode = "manual"
        self.emergency_stop = False
        self.manual_active = False
        self.shutdown_requested = False
        
        # Finger configuration
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
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
            String, '/mode', self.handle_mode_change, 10)
            
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.handle_emergency_stop, 10)
        
        # Publishers
        self.finger_curl_pub = self.create_publisher(
            Float32MultiArray, '/perception/finger_curl_ratios', 10)
            
        self.manual_status_pub = self.create_publisher(
            String, '/manual/status', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.publish_manual_data)
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Interactive control thread
        self.control_thread = threading.Thread(target=self.interactive_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info('Manual control node started')
        self.print_help()
    
    def handle_mode_change(self, msg):
        """Handle control mode changes."""
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            
            if new_mode == "manual":
                self.manual_active = True
                self.get_logger().info('🎮 Manual control activated!')
                self.print_help()
            else:
                self.manual_active = False
                self.get_logger().info('Manual control deactivated')
    
    def handle_emergency_stop(self, msg):
        """Handle emergency stop signal."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('🛑 Emergency stop - manual control disabled')
            self.manual_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.get_logger().info('✅ Emergency stop released')
    
    def publish_manual_data(self):
        """Publish manual finger curl data when active."""
        if self.manual_active and not self.emergency_stop:
            curl_msg = Float32MultiArray()
            curl_msg.data = self.manual_finger_curls.tolist()
            self.finger_curl_pub.publish(curl_msg)
    
    def publish_status(self):
        """Publish manual control status."""
        status_info = {
            'manual_active': self.manual_active,
            'current_mode': self.current_mode,
            'emergency_stop': self.emergency_stop,
            'finger_curls': [f"{c:.2f}" for c in self.manual_finger_curls]
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.manual_status_pub.publish(status_msg)
    
    def print_help(self):
        """Print help information."""
        help_text = """
🎮 MANUAL CONTROL COMMANDS:
═══════════════════════════
📖 Presets:
  'open', 'closed', 'pinch', 'point', 'peace', 'rock'

🔧 Individual Control:
  't 0.5' - Set thumb to 50%
  'i 1.0' - Set index to 100%
  'm 0.0' - Set middle to 0%
  'r 0.8' - Set ring to 80%
  'p 0.3' - Set pinky to 30%

📊 Commands:
  'status', 'help', 'reset', 'quit'

💡 Values: 0.0 = open, 1.0 = closed
═══════════════════════════
"""
        print(help_text)
    
    def interactive_loop(self):
        """Interactive control loop."""
        while rclpy.ok() and not self.shutdown_requested:
            if self.manual_active and not self.emergency_stop:
                try:
                    print(f"\n🎮 Manual [{self.finger_names}]: {[f'{c:.1f}' for c in self.manual_finger_curls]}")
                    command = input("Command: ").strip().lower()
                    
                    if command in self.hand_presets:
                        self.manual_finger_curls = np.array(self.hand_presets[command])
                        self.get_logger().info(f"Applied preset: {command}")
                        
                    elif command.startswith(('t ', 'i ', 'm ', 'r ', 'p ')):
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
                        self.get_logger().info("Reset to open position")
                        
                    elif command == 'quit':
                        self.shutdown_requested = True
                        break
                        
                    elif command == '':
                        continue
                        
                    else:
                        print("❌ Unknown command. Type 'help' for commands.")
                        
                except (EOFError, KeyboardInterrupt):
                    self.shutdown_requested = True
                    break
                except Exception as e:
                    print(f"❌ Error: {e}")
            else:
                time.sleep(1)
        
        self.get_logger().info("Interactive loop shutting down")
    
    def shutdown_node(self):
        """Shutdown the node properly."""
        self.get_logger().info("Shutting down manual control node...")
        self.shutdown_requested = True
        
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual control node stopped by user')
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
