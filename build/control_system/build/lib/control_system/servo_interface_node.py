#!/usr/bin/env python3
"""
Servo Interface Node

Direct hardware interface for Dynamixel XL330-288 servos.
Handles low-level servo communication and hardware-specific operations.
Part of the unified control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np

# Dynamixel SDK imports (will need to be installed)
try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False

class ServoInterfaceNode(Node):
    """
    Hardware interface for Dynamixel servos.
    
    Subscribes to:
    - /servo/commands - Servo position commands from unified control
    
    Publishes to:
    - /servo/hardware_status - Hardware status and diagnostics
    - /servo/positions - Current actual servo positions
    """
    
    def __init__(self):
        super().__init__('servo_interface_node')
        
        # Hardware configuration
        self.device_name = '/dev/ttyUSB0'  # Adjust based on your setup
        self.baudrate = 57600
        self.protocol_version = 2.0
        
        # Servo IDs (adjust based on your wiring)
        self.servo_ids = [1, 2, 3, 4, 5]  # thumb, index, middle, ring, pinky
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        # Dynamixel control table addresses for XL330-288
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_MOVING_STATUS = 123
        
        # Initialize hardware interface
        self.hardware_connected = False
        self.port_handler = None
        self.packet_handler = None
        
        if DYNAMIXEL_AVAILABLE:
            self.initialize_hardware()
        else:
            self.get_logger().warn('Dynamixel SDK not available - running in simulation mode')
        
        # Subscribers
        self.servo_commands_sub = self.create_subscription(
            Float32MultiArray,
            '/servo/commands',
            self.handle_servo_commands,
            10)
        
        # Publishers
        self.hardware_status_pub = self.create_publisher(
            String,
            '/servo/hardware_status',
            10)
            
        self.positions_pub = self.create_publisher(
            Float32MultiArray,
            '/servo/positions',
            10)
        
        # Timers
        self.status_timer = self.create_timer(0.1, self.read_servo_positions)  # 10Hz position reading
        self.health_timer = self.create_timer(5.0, self.publish_health_status)  # 5s health check
        
        self.get_logger().info(f'Servo interface node started (Hardware: {"Connected" if self.hardware_connected else "Simulated"})')

    def initialize_hardware(self):
        """Initialize Dynamixel hardware interface."""
        try:
            # Initialize PortHandler and PacketHandler
            self.port_handler = PortHandler(self.device_name)
            self.packet_handler = PacketHandler(self.protocol_version)
            
            # Open port
            if not self.port_handler.openPort():
                self.get_logger().error(f'Failed to open port {self.device_name}')
                return
                
            # Set baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                self.get_logger().error(f'Failed to set baudrate to {self.baudrate}')
                return
            
            # Enable torque for all servos
            for servo_id in self.servo_ids:
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, 1)
                
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'Failed to enable torque for servo {servo_id}')
                    return
                elif dxl_error != 0:
                    self.get_logger().error(f'Servo {servo_id} error: {dxl_error}')
                    return
            
            self.hardware_connected = True
            self.get_logger().info('Successfully initialized Dynamixel hardware')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize hardware: {e}')
            self.hardware_connected = False

    def handle_servo_commands(self, msg):
        """Handle incoming servo position commands."""
        if len(msg.data) != 5:
            self.get_logger().warn(f'Expected 5 servo commands, got {len(msg.data)}')
            return
        
        if not self.hardware_connected:
            # Simulation mode - just log the commands
            self.get_logger().debug(f'[SIM] Servo commands: {[int(p) for p in msg.data]}')
            return
        
        # Send commands to actual hardware
        for i, position in enumerate(msg.data):
            servo_id = self.servo_ids[i]
            position_int = int(np.clip(position, 0, 4095))
            
            try:
                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_GOAL_POSITION, position_int)
                
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'Communication failed for servo {servo_id}')
                elif dxl_error != 0:
                    self.get_logger().error(f'Servo {servo_id} error: {dxl_error}')
                    
            except Exception as e:
                self.get_logger().error(f'Error commanding servo {servo_id}: {e}')

    def read_servo_positions(self):
        """Read current positions from all servos."""
        if not self.hardware_connected:
            # Simulation mode - return dummy positions
            sim_positions = [2048, 2048, 2048, 2048, 2048]  # Neutral positions
            position_msg = Float32MultiArray()
            position_msg.data = sim_positions
            self.positions_pub.publish(position_msg)
            return
        
        positions = []
        for servo_id in self.servo_ids:
            try:
                dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_PRESENT_POSITION)
                
                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    positions.append(float(dxl_present_position))
                else:
                    self.get_logger().warn(f'Failed to read position from servo {servo_id}')
                    positions.append(0.0)  # Default value
                    
            except Exception as e:
                self.get_logger().error(f'Error reading servo {servo_id}: {e}')
                positions.append(0.0)
        
        # Publish current positions
        position_msg = Float32MultiArray()
        position_msg.data = positions
        self.positions_pub.publish(position_msg)

    def publish_health_status(self):
        """Publish hardware health status."""
        status_info = {
            'hardware_connected': self.hardware_connected,
            'device': self.device_name,
            'baudrate': self.baudrate,
            'servo_count': len(self.servo_ids),
            'dynamixel_sdk': DYNAMIXEL_AVAILABLE
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.hardware_status_pub.publish(status_msg)

    def shutdown_hardware(self):
        """Safely shutdown hardware interface."""
        if self.hardware_connected and self.port_handler:
            # Disable torque for all servos
            for servo_id in self.servo_ids:
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, 0)
                except:
                    pass  # Ignore errors during shutdown
            
            # Close port
            self.port_handler.closePort()
            self.get_logger().info('Hardware interface safely shutdown')

def main(args=None):
    rclpy.init(args=args)
    node = ServoInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Servo interface node stopped by user')
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
