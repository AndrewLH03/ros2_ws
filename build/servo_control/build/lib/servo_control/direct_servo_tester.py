#!/usr/bin/env python3
"""
Direct Servo Testing Script

Provides direct servo position control for hardware testing and calibration.
This script bypasses the finger curl conversion and sends raw servo positions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np
import time

class DirectServoTester(Node):
    """Direct servo position testing interface."""
    
    def __init__(self):
        super().__init__('direct_servo_tester')
        
        # Current servo positions (0-4095 for Dynamixel XL330-288)
        self.servo_positions = [2048, 2048, 2048, 2048, 2048]  # Neutral positions
        self.finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        
        # Safe position ranges for each servo (min, max)
        self.position_ranges = [
            (1200, 2800),  # Thumb
            (1000, 3000),  # Index  
            (1000, 3000),  # Middle
            (1000, 3000),  # Ring
            (1100, 2900),  # Pinky
        ]
        
        # Publishers
        self.servo_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/servo/commands',
            10)
            
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        
        # Switch to manual mode
        self.switch_to_manual_mode()
        
        self.get_logger().info('Direct servo tester initialized')
        
    def switch_to_manual_mode(self):
        """Switch the system to manual mode."""
        time.sleep(1)  # Give time for node to initialize
        msg = String()
        msg.data = 'manual'
        self.mode_pub.publish(msg)
        self.get_logger().info('Switched to manual mode')
    
    def send_servo_positions(self):
        """Send current servo positions."""
        msg = Float32MultiArray()
        msg.data = [float(pos) for pos in self.servo_positions]
        self.servo_commands_pub.publish(msg)
        
    def set_servo_position(self, servo_index, position):
        """Set a specific servo position with safety checks."""
        if 0 <= servo_index < 5:
            min_pos, max_pos = self.position_ranges[servo_index]
            safe_position = np.clip(position, min_pos, max_pos)
            self.servo_positions[servo_index] = int(safe_position)
            self.send_servo_positions()
            
            finger_name = self.finger_names[servo_index]
            self.get_logger().info(f'Set {finger_name} to position {safe_position}')
            return safe_position
        else:
            self.get_logger().error(f'Invalid servo index: {servo_index}')
            return None
    
    def set_all_servos(self, position):
        """Set all servos to the same position."""
        for i in range(5):
            self.set_servo_position(i, position)
    
    def apply_preset(self, preset_name):
        """Apply a preset servo configuration."""
        presets = {
            'neutral': [2048, 2048, 2048, 2048, 2048],
            'open': [1200, 1000, 1000, 1000, 1100],  # Minimum positions
            'closed': [2800, 3000, 3000, 3000, 2900],  # Maximum positions
            'pinch': [2400, 2800, 1000, 1000, 1100],  # Thumb + index
            'point': [1200, 1000, 3000, 3000, 2900],  # Index pointing
        }
        
        if preset_name in presets:
            self.servo_positions = presets[preset_name].copy()
            self.send_servo_positions()
            self.get_logger().info(f'Applied preset: {preset_name}')
            return True
        else:
            self.get_logger().error(f'Unknown preset: {preset_name}')
            return False
    
    def print_status(self):
        """Print current servo positions."""
        print(f"\n📊 Current Servo Positions:")
        print("=" * 40)
        for i, (name, pos) in enumerate(zip(self.finger_names, self.servo_positions)):
            min_pos, max_pos = self.position_ranges[i]
            percentage = (pos - min_pos) / (max_pos - min_pos) * 100
            print(f"{name:>7}: {pos:>4} ({percentage:>5.1f}%)")
        print("=" * 40)
    
    def print_help(self):
        """Print help information."""
        help_text = """
🔧 DIRECT SERVO TESTING COMMANDS:
═══════════════════════════════════
📍 Direct Position Control:
  set <servo> <position>  - Set servo (0-4) to position (0-4095)
  Example: set 0 2048     - Set thumb to neutral position

🎯 Preset Configurations:
  neutral  - All servos to center position (2048)
  open     - All fingers fully open
  closed   - All fingers fully closed
  pinch    - Thumb + index pinch position
  point    - Index finger pointing

🔧 Bulk Operations:
  all <position>  - Set all servos to same position
  Example: all 2048

📊 Status Commands:
  status   - Show current positions and percentages
  ranges   - Show safe position ranges for each servo
  help     - Show this help

⚠️  Safety Information:
  - Positions are clamped to safe ranges automatically
  - Thumb:  1200-2800 (range of 1600)
  - Index:  1000-3000 (range of 2000)  
  - Middle: 1000-3000 (range of 2000)
  - Ring:   1000-3000 (range of 2000)
  - Pinky:  1100-2900 (range of 1800)

🚪 Exit Commands:
  quit, exit, q  - Exit direct servo testing

💡 Tips:
  - Start with 'neutral' preset for safe initial positions
  - Use small increments when testing (±100-200)
  - Monitor servo heat and current draw during testing
  - Emergency stop available: Ctrl+C or UI emergency button
═══════════════════════════════════
"""
        print(help_text)
    
    def run_interactive_control(self):
        """Run the interactive control loop."""
        print("\n🔧 Direct Servo Testing Mode")
        print("Type 'help' for available commands")
        self.print_help()
        
        while rclpy.ok():
            try:
                self.print_status()
                command = input("\n🎯 Enter command: ").strip().lower()
                
                if command in ['quit', 'exit', 'q']:
                    break
                elif command == 'help':
                    self.print_help()
                elif command == 'status':
                    continue  # Status is printed every loop
                elif command == 'ranges':
                    print("\n📏 Safe Position Ranges:")
                    for i, (name, (min_pos, max_pos)) in enumerate(zip(self.finger_names, self.position_ranges)):
                        print(f"{name:>7}: {min_pos:>4} - {max_pos:>4} (range: {max_pos-min_pos})")
                elif command.startswith('set '):
                    parts = command.split()
                    if len(parts) == 3:
                        try:
                            servo_idx = int(parts[1])
                            position = int(parts[2])
                            self.set_servo_position(servo_idx, position)
                        except ValueError:
                            print("❌ Invalid numbers. Usage: set <servo_index> <position>")
                    else:
                        print("❌ Usage: set <servo_index> <position>")
                elif command.startswith('all '):
                    parts = command.split()
                    if len(parts) == 2:
                        try:
                            position = int(parts[1])
                            self.set_all_servos(position)
                        except ValueError:
                            print("❌ Invalid number. Usage: all <position>")
                    else:
                        print("❌ Usage: all <position>")
                elif command in ['neutral', 'open', 'closed', 'pinch', 'point']:
                    self.apply_preset(command)
                elif command == '':
                    continue
                else:
                    print("❌ Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
        
        self.get_logger().info('Direct servo testing completed')

def main():
    rclpy.init()
    
    try:
        tester = DirectServoTester()
        
        # Run interactive control
        tester.run_interactive_control()
        
    except KeyboardInterrupt:
        print("\n🛑 Direct servo testing stopped by user")
    finally:
        try:
            tester.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
