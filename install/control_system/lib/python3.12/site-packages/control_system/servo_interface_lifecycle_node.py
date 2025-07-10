#!/usr/bin/env python3
"""
Servo Interface Lifecycle Node

Direct hardware interface for Dynamixel XL330-288 servos with lifecycle management.
Handles low-level servo communication and hardware-specific operations.
Part of the unified control system.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import Float32MultiArray, String, Bool
import numpy as np

# Dynamixel SDK imports (will need to be installed)
try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False

class ServoInterfaceLifecycleNode(LifecycleNode):
    """
    Hardware interface for Dynamixel servos with lifecycle management.
    
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
        
        # Initialize variables (but don't create hardware connections yet)
        self.hardware_connected = False
        self.port_handler = None
        self.packet_handler = None
        
        # Publishers and subscribers (will be created in on_configure)
        self.servo_commands_sub = None
        self.hardware_status_pub = None
        self.positions_pub = None
        self.status_timer = None
        self.health_timer = None
        
        self.get_logger().info('Servo interface lifecycle node initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the servo interface - initialize hardware connections."""
        try:
            self.get_logger().info('Configuring servo interface...')
            
            # Initialize hardware interface
            if DYNAMIXEL_AVAILABLE:
                success = self.initialize_hardware()
                if not success:
                    self.get_logger().error('Failed to initialize hardware')
                    return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().warn('Dynamixel SDK not available - running in simulation mode')
            
            # Create publishers
            self.hardware_status_pub = self.create_lifecycle_publisher(
                String, '/servo/hardware_status', 10)
            self.positions_pub = self.create_lifecycle_publisher(
                Float32MultiArray, '/servo/positions', 10)
            
            # Create subscribers
            self.servo_commands_sub = self.create_subscription(
                Float32MultiArray, '/servo/commands', self.handle_servo_commands, 10)
            
            self.get_logger().info('Servo interface configured successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure servo interface: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the servo interface - start servo operations."""
        try:
            self.get_logger().info('Activating servo interface...')
            
            # Activate publishers
            self.hardware_status_pub.on_activate()
            self.positions_pub.on_activate()
            
            # Enable servo torque
            if self.hardware_connected:
                self.enable_servos()
            
            # Start timers for position reading and health monitoring
            self.status_timer = self.create_timer(0.1, self.read_servo_positions)  # 10Hz
            self.health_timer = self.create_timer(5.0, self.publish_health_status)  # 5s
            
            self.get_logger().info('Servo interface activated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate servo interface: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the servo interface - stop servo operations but keep connections."""
        try:
            self.get_logger().info('Deactivating servo interface...')
            
            # Disable servo torque for safety
            if self.hardware_connected:
                self.disable_servos()
            
            # Stop timers
            if self.status_timer:
                self.destroy_timer(self.status_timer)
                self.status_timer = None
            if self.health_timer:
                self.destroy_timer(self.health_timer)
                self.health_timer = None
            
            # Deactivate publishers
            self.hardware_status_pub.on_deactivate()
            self.positions_pub.on_deactivate()
            
            self.get_logger().info('Servo interface deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate servo interface: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup the servo interface - release hardware resources."""
        try:
            self.get_logger().info('Cleaning up servo interface...')
            
            # Close hardware connections
            if self.hardware_connected and self.port_handler:
                self.port_handler.closePort()
                self.hardware_connected = False
            
            # Destroy publishers and subscribers
            if self.hardware_status_pub:
                self.destroy_lifecycle_publisher(self.hardware_status_pub)
            if self.positions_pub:
                self.destroy_lifecycle_publisher(self.positions_pub)
            if self.servo_commands_sub:
                self.destroy_subscription(self.servo_commands_sub)
            
            self.get_logger().info('Servo interface cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup servo interface: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the servo interface."""
        self.get_logger().info('Servo interface shutting down')
        return TransitionCallbackReturn.SUCCESS

    def initialize_hardware(self):
        """Initialize Dynamixel hardware interface."""
        try:
            # Initialize PortHandler and PacketHandler
            self.port_handler = PortHandler(self.device_name)
            self.packet_handler = PacketHandler(self.protocol_version)
            
            # Open port
            if not self.port_handler.openPort():
                self.get_logger().error(f'Failed to open port {self.device_name}')
                return False
                
            # Set baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                self.get_logger().error(f'Failed to set baudrate to {self.baudrate}')
                return False
            
            # Test communication with servos
            communication_errors = 0
            for servo_id in self.servo_ids:
                position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_PRESENT_POSITION)
                
                if dxl_comm_result != COMM_SUCCESS:
                    communication_errors += 1
                    self.get_logger().warn(f'Communication failed for servo {servo_id}')
            
            if communication_errors == len(self.servo_ids):
                self.get_logger().error('No servos responding - check connections')
                return False
            elif communication_errors > 0:
                self.get_logger().warn(f'{communication_errors} servos not responding')
            
            self.hardware_connected = True
            self.get_logger().info(f'Hardware initialized - {len(self.servo_ids) - communication_errors}/{len(self.servo_ids)} servos connected')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Hardware initialization failed: {e}')
            return False

    def enable_servos(self):
        """Enable torque for all servos."""
        if not self.hardware_connected:
            return
        
        for servo_id in self.servo_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, 1)
            
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f'Failed to enable torque for servo {servo_id}')

    def disable_servos(self):
        """Disable torque for all servos."""
        if not self.hardware_connected:
            return
        
        for servo_id in self.servo_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, self.ADDR_TORQUE_ENABLE, 0)
            
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f'Failed to disable torque for servo {servo_id}')

    def handle_servo_commands(self, msg):
        """Handle servo position commands - only process when active."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        
        if not self.hardware_connected:
            # Simulation mode
            self.get_logger().debug(f'Simulation mode - would move servos to: {list(msg.data)}')
            return
        
        if len(msg.data) != len(self.servo_ids):
            self.get_logger().warn(f'Expected {len(self.servo_ids)} servo commands, got {len(msg.data)}')
            return
        
        # Convert from normalized positions (0-1) to servo positions (0-4095)
        for i, (servo_id, position) in enumerate(zip(self.servo_ids, msg.data)):
            # Clamp position to valid range
            clamped_position = max(0.0, min(1.0, position))
            servo_position = int(clamped_position * 4095)
            
            # Send position command
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, servo_id, self.ADDR_GOAL_POSITION, servo_position)
            
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f'Failed to set position for servo {servo_id}')

    def read_servo_positions(self):
        """Read current servo positions - only when active."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        
        positions = []
        
        if self.hardware_connected:
            for servo_id in self.servo_ids:
                position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, servo_id, self.ADDR_PRESENT_POSITION)
                
                if dxl_comm_result == COMM_SUCCESS:
                    # Convert from servo position (0-4095) to normalized (0-1)
                    normalized_position = position / 4095.0
                    positions.append(normalized_position)
                else:
                    positions.append(0.0)  # Default on communication failure
        else:
            # Simulation mode - return dummy positions
            positions = [0.5] * len(self.servo_ids)
        
        # Publish positions
        msg = Float32MultiArray()
        msg.data = positions
        self.positions_pub.publish(msg)

    def publish_health_status(self):
        """Publish hardware health status - only when active."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        
        if self.hardware_connected:
            status = f"Hardware connected: {len(self.servo_ids)} servos"
        else:
            status = "Simulation mode - no hardware"
        
        msg = String()
        msg.data = status
        self.hardware_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        servo_interface = ServoInterfaceLifecycleNode()
        rclpy.spin(servo_interface)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            servo_interface.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
