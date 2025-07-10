#!/usr/bin/env python3
"""
Unit tests for lifecycle nodes in the control system.
"""

import unittest
import rclpy
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
import time
from std_msgs.msg import String


class TestLifecycleNodes(unittest.TestCase):
    """Test lifecycle node functionality."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test."""
        pass
    
    def tearDown(self):
        """Clean up after each test."""
        pass
    
    def test_lifecycle_manager_startup(self):
        """Test that lifecycle manager starts correctly."""
        from control_system.lifecycle_manager import LifecycleManager
        
        manager = LifecycleManager()
        
        # Check that node groups are defined
        self.assertIn('core', manager.node_groups)
        self.assertIn('manual', manager.node_groups)
        self.assertIn('perception', manager.node_groups)
        
        # Check initial mode
        self.assertEqual(manager.current_mode, 'manual')
        
        manager.destroy_node()
    
    def test_mode_switching_request(self):
        """Test mode switching functionality."""
        from control_system.lifecycle_manager import LifecycleManager
        
        manager = LifecycleManager()
        
        # Create a mode change message
        mode_msg = String()
        mode_msg.data = 'perception'
        
        # Test mode change handling
        initial_mode = manager.current_mode
        manager.handle_mode_change(mode_msg)
        
        # Note: In real test, we'd check if nodes actually changed state
        # For now, just verify the method runs without error
        
        manager.destroy_node()
    
    def test_transition_name_mapping(self):
        """Test transition name mapping."""
        from control_system.lifecycle_manager import LifecycleManager
        from lifecycle_msgs.msg import TransitionEvent
        
        manager = LifecycleManager()
        
        # Test transition name mappings
        self.assertEqual(
            manager._get_transition_name(TransitionEvent.TRANSITION_CONFIGURE),
            'configure'
        )
        self.assertEqual(
            manager._get_transition_name(TransitionEvent.TRANSITION_ACTIVATE),
            'activate'
        )
        self.assertEqual(
            manager._get_transition_name(TransitionEvent.TRANSITION_DEACTIVATE),
            'deactivate'
        )
        
        manager.destroy_node()
    
    def test_node_groups_structure(self):
        """Test that all required node groups are properly defined."""
        from control_system.lifecycle_manager import LifecycleManager
        
        manager = LifecycleManager()
        
        required_groups = ['core', 'manual', 'perception', 'simulation', 'diagnostic']
        
        for group in required_groups:
            self.assertIn(group, manager.node_groups)
            self.assertIsInstance(manager.node_groups[group], list)
            self.assertGreater(len(manager.node_groups[group]), 0)
        
        manager.destroy_node()


class TestLifecycleNodeBase(unittest.TestCase):
    """Base test class for lifecycle node testing."""
    
    def create_mock_lifecycle_node(self):
        """Create a mock lifecycle node for testing."""
        from rclpy.lifecycle import LifecycleNode
        
        class MockLifecycleNode(LifecycleNode):
            def __init__(self):
                super().__init__('mock_node')
                self.configured = False
                self.activated = False
            
            def on_configure(self, state):
                self.configured = True
                return TransitionCallbackReturn.SUCCESS
            
            def on_activate(self, state):
                self.activated = True
                return TransitionCallbackReturn.SUCCESS
            
            def on_deactivate(self, state):
                self.activated = False
                return TransitionCallbackReturn.SUCCESS
            
            def on_cleanup(self, state):
                self.configured = False
                return TransitionCallbackReturn.SUCCESS
            
            def on_shutdown(self, state):
                return TransitionCallbackReturn.SUCCESS
        
        return MockLifecycleNode()
    
    def test_lifecycle_transitions(self):
        """Test basic lifecycle transitions."""
        node = self.create_mock_lifecycle_node()
        
        # Test configure transition
        result = node.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertTrue(node.configured)
        
        # Test activate transition
        result = node.on_activate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertTrue(node.activated)
        
        # Test deactivate transition
        result = node.on_deactivate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertFalse(node.activated)
        
        # Test cleanup transition
        result = node.on_cleanup(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertFalse(node.configured)
        
        node.destroy_node()


if __name__ == '__main__':
    # Run specific test
    suite = unittest.TestLoader().loadTestsFromTestCase(TestLifecycleNodes)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Exit with error code if tests failed
    exit(0 if result.wasSuccessful() else 1)
