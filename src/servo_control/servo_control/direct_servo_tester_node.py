#!/usr/bin/env python3
"""
Direct Servo Tester Node

Comprehensive testing tool for Dynamixel XL330-288 servos.
Provides hardware validation, calibration, and diagnostic features.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np
import time
import threading

class DirectServoTesterNode(Node):
    """
    Direct servo testing and calibration node.
    
    Publishes to:
    - /servo/commands - Direct servo position commands
    - /manual/servo_direct_commands - Direct position commands for manual control
    
    Subscribes to:
    - /servo/positions - Current servo positions
    - /servo/hardware_status - Hardware status
    """
    
    def __init__(self):
        super().__init__('direct_servo_tester_node')
        
        # Test configuration
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        self.servo_ranges = [
            (1200, 2800),  # Thumb
            (1000, 3000),  # Index
            (1000, 3000),  # Middle
            (1000, 3000),  # Ring
            (1100, 2900)   # Pinky
        ]
        
        # Current positions
        self.current_positions = [2048, 2048, 2048, 2048, 2048]  # Neutral
        self.hardware_connected = False
        
        # Publishers
        self.servo_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/servo/commands',
            10)
            
        self.direct_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/manual/servo_direct_commands',
            10)
        
        # Subscribers
        self.positions_sub = self.create_subscription(
            Float32MultiArray,
            '/servo/positions',
            self.positions_callback,
            10)
            
        self.hardware_status_sub = self.create_subscription(
            String,
            '/servo/hardware_status',
            self.hardware_status_callback,
            10)
        
        # Test timer
        self.test_timer = None
        self.test_sequence_active = False
        
        # Start interactive control
        self.control_thread = threading.Thread(target=self.interactive_control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info('Direct servo tester node started')
        self.print_help()

    def positions_callback(self, msg):
        """Update current servo positions."""
        if len(msg.data) == 5:
            self.current_positions = list(msg.data)

    def hardware_status_callback(self, msg):
        """Update hardware connection status."""
        try:
            status = eval(msg.data)  # Convert string back to dict
            self.hardware_connected = status.get('hardware_connected', False)
        except:
            pass

    def send_servo_commands(self, positions):
        """Send direct servo position commands."""
        if len(positions) != 5:
            self.get_logger().warn(f'Expected 5 positions, got {len(positions)}')
            return
            
        # Clamp positions to safe ranges
        safe_positions = []
        for i, pos in enumerate(positions):
            min_pos, max_pos = self.servo_ranges[i]
            safe_pos = np.clip(pos, min_pos, max_pos)
            safe_positions.append(float(safe_pos))
        
        # Publish to both topics
        msg = Float32MultiArray()
        msg.data = safe_positions
        
        self.servo_commands_pub.publish(msg)
        self.direct_commands_pub.publish(msg)
        
        mode = "Hardware" if self.hardware_connected else "Simulation"
        self.get_logger().info(f'[{mode}] Sent positions: {[int(p) for p in safe_positions]}')

    def run_calibration_sequence(self):
        """Run a calibration sequence for all servos."""
        self.get_logger().info('🔧 Starting calibration sequence...')
        self.test_sequence_active = True
        
        sequences = [
            ("Neutral position", [2048, 2048, 2048, 2048, 2048]),
            ("Minimum positions", [1200, 1000, 1000, 1000, 1100]),
            ("Maximum positions", [2800, 3000, 3000, 3000, 2900]),
            ("Back to neutral", [2048, 2048, 2048, 2048, 2048]),
        ]
        
        for desc, positions in sequences:
            self.get_logger().info(f'📍 {desc}...')
            self.send_servo_commands(positions)
            time.sleep(3)  # Wait 3 seconds between positions
            
        self.test_sequence_active = False
        self.get_logger().info('✅ Calibration sequence complete')

    def run_range_test(self, servo_index):
        """Test the full range of a specific servo."""
        if servo_index < 0 or servo_index >= 5:
            self.get_logger().warn('Invalid servo index. Use 0-4.')
            return
            
        finger_name = self.finger_names[servo_index]
        min_pos, max_pos = self.servo_ranges[servo_index]
        
        self.get_logger().info(f'🧪 Testing {finger_name} servo range ({min_pos} to {max_pos})...')
        self.test_sequence_active = True
        
        # Create sweep positions
        positions = [2048] * 5  # Start with all neutral
        
        # Sweep from min to max and back
        test_positions = list(range(min_pos, max_pos + 1, 200)) + list(range(max_pos, min_pos - 1, -200))
        
        for pos in test_positions:
            positions[servo_index] = pos
            self.send_servo_commands(positions)
            time.sleep(0.5)
            
        # Return to neutral
        positions[servo_index] = 2048
        self.send_servo_commands(positions)
        
        self.test_sequence_active = False
        self.get_logger().info(f'✅ {finger_name} range test complete')

    def run_functional_test(self):
        """Run functional tests simulating hand gestures."""
        self.get_logger().info('🤖 Starting functional gesture tests...')
        self.test_sequence_active = True
        
        gestures = [
            ("Open hand", [1200, 1000, 1000, 1000, 1100]),
            ("Closed fist", [2800, 3000, 3000, 3000, 2900]),
            ("Pinch (thumb + index)", [2500, 2800, 1000, 1000, 1100]),
            ("Point (index only)", [1200, 1000, 3000, 3000, 2900]),
            ("Peace sign", [1200, 1000, 1000, 3000, 2900]),
            ("Rock sign", [1200, 1000, 3000, 3000, 1100]),
            ("Neutral", [2048, 2048, 2048, 2048, 2048]),
        ]
        
        for desc, positions in gestures:
            self.get_logger().info(f'👋 {desc}...')
            self.send_servo_commands(positions)
            time.sleep(2)
            
        self.test_sequence_active = False
        self.get_logger().info('✅ Functional tests complete')

    def print_help(self):
        """Print help information."""
        help_text = """
🧪 DIRECT SERVO TESTER COMMANDS:
═══════════════════════════════════
🔧 Calibration & Testing:
  'cal'        - Run full calibration sequence
  'range N'    - Test range of servo N (0-4: thumb, index, middle, ring, pinky)
  'func'       - Run functional gesture tests
  'sweep'      - Sweep all servos slowly

📍 Direct Position Control:
  'set N POS'  - Set servo N to position POS (e.g., 'set 0 2048')
  'all POS'    - Set all servos to same position
  'neutral'    - Set all servos to neutral (2048)
  'min'        - Set all servos to minimum positions
  'max'        - Set all servos to maximum positions

📊 Status & Diagnostics:
  'status'     - Show current positions and hardware status
  'ranges'     - Show servo ranges
  'positions'  - Show current positions
  'hardware'   - Show hardware connection status

⚡ Quick Gestures:
  'open'       - Open hand
  'close'      - Close hand
  'pinch'      - Pinch gesture
  'point'      - Point gesture

🛠️  Utility:
  'help'       - Show this help
  'quit'       - Exit tester

💡 Servo ranges: 0-4095 (XL330-288), Safe ranges are pre-configured
⚠️  Always test in simulation first before connecting hardware!
═══════════════════════════════════
"""
        print(help_text)

    def interactive_control_loop(self):
        """Interactive control loop."""
        while rclpy.ok():
            try:
                print(f"\n🧪 Servo Tester [{self.finger_names}]")
                print(f"Current: {[int(p) for p in self.current_positions]} | Hardware: {'✅' if self.hardware_connected else '❌'}")
                
                command = input("Tester> ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'help':
                    self.print_help()
                elif command == 'cal':
                    threading.Thread(target=self.run_calibration_sequence).start()
                elif command == 'func':
                    threading.Thread(target=self.run_functional_test).start()
                elif command.startswith('range '):
                    try:
                        servo_idx = int(command.split()[1])
                        threading.Thread(target=self.run_range_test, args=(servo_idx,)).start()
                    except (IndexError, ValueError):
                        print("❌ Usage: range N (where N is 0-4)")
                elif command.startswith('set '):
                    try:
                        parts = command.split()
                        servo_idx = int(parts[1])
                        position = int(parts[2])
                        if 0 <= servo_idx <= 4:
                            positions = list(self.current_positions)
                            positions[servo_idx] = position
                            self.send_servo_commands(positions)
                        else:
                            print("❌ Servo index must be 0-4")
                    except (IndexError, ValueError):
                        print("❌ Usage: set N POS (e.g., set 0 2048)")
                elif command.startswith('all '):
                    try:
                        position = int(command.split()[1])
                        self.send_servo_commands([position] * 5)
                    except (IndexError, ValueError):
                        print("❌ Usage: all POS (e.g., all 2048)")
                elif command == 'neutral':
                    self.send_servo_commands([2048, 2048, 2048, 2048, 2048])
                elif command == 'min':
                    min_positions = [r[0] for r in self.servo_ranges]
                    self.send_servo_commands(min_positions)
                elif command == 'max':
                    max_positions = [r[1] for r in self.servo_ranges]
                    self.send_servo_commands(max_positions)
                elif command == 'open':
                    self.send_servo_commands([1200, 1000, 1000, 1000, 1100])
                elif command == 'close':
                    self.send_servo_commands([2800, 3000, 3000, 3000, 2900])
                elif command == 'pinch':
                    self.send_servo_commands([2500, 2800, 1000, 1000, 1100])
                elif command == 'point':
                    self.send_servo_commands([1200, 1000, 3000, 3000, 2900])
                elif command == 'status':
                    print(f"📊 Current positions: {[int(p) for p in self.current_positions]}")
                    print(f"🔌 Hardware connected: {self.hardware_connected}")
                    print(f"🧪 Test sequence active: {self.test_sequence_active}")
                elif command == 'ranges':
                    print("📏 Servo ranges:")
                    for i, (name, (min_pos, max_pos)) in enumerate(zip(self.finger_names, self.servo_ranges)):
                        print(f"  {i}: {name} = {min_pos} to {max_pos}")
                elif command == 'positions':
                    print("📍 Current positions:")
                    for i, (name, pos) in enumerate(zip(self.finger_names, self.current_positions)):
                        print(f"  {i}: {name} = {int(pos)}")
                elif command == 'hardware':
                    status = "Connected" if self.hardware_connected else "Simulation Mode"
                    print(f"🔌 Hardware status: {status}")
                elif command == '':
                    continue
                else:
                    print(f"❌ Unknown command: {command}")
                    print("Type 'help' for available commands")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectServoTesterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Direct servo tester stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
