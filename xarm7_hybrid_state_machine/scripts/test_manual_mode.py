#!/usr/bin/env python3

"""
Test script to demonstrate MANUAL_MODE transitions and logging.
This script specifically tests the MANUAL_MODE state to verify proper logging and command handling.
"""

import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotStateAndTargetPose
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose
import time

class ManualModeTestNode(Node):
    def __init__(self):
        super().__init__('manual_mode_test')
        
        # Publisher for state commands
        self.state_publisher = self.create_publisher(
            RobotStateAndTargetPose, 
            '/xarm7_state_topic', 
            10
        )
        
        # Subscriber to monitor state machine state
        self.state_subscriber = self.create_subscription(
            UInt8,
            '/xarm7_state_machine_state',
            self.state_callback,
            10
        )
        
        self.current_sm_state = 0
        self.state_names = {
            0: "IDLE", 1: "MOVING", 2: "PICKING", 3: "PLACING", 
            4: "MANUAL_MODE", 5: "FINAL", 6: "ERROR"
        }
        
        # Wait for publishers to be ready
        time.sleep(1)
        
        self.get_logger().info("MANUAL-MODE-TEST: Starting MANUAL_MODE test sequence...")
        
        # Create test timer
        self.test_timer = self.create_wall_timer(6.0, self.test_sequence)
        self.test_step = 0
        
    def state_callback(self, msg):
        """Monitor state machine state changes"""
        if msg.data != self.current_sm_state:
            old_state = self.state_names.get(self.current_sm_state, "UNKNOWN")
            new_state = self.state_names.get(msg.data, "UNKNOWN")
            self.get_logger().info(f"MANUAL-MODE-TEST: State changed from {old_state} to {new_state}")
            self.current_sm_state = msg.data
        
    def test_sequence(self):
        """Test sequence to demonstrate MANUAL_MODE transitions and logging"""
        
        if self.test_step == 0:
            self.get_logger().info("MANUAL-MODE-TEST: Step 1 - Sending MANUAL_MODE command (should show entry logs with TEACHING_JOINT mode)")
            self.send_manual_mode_command()
        elif self.test_step == 1:
            self.get_logger().info("MANUAL-MODE-TEST: Step 2 - Sending MANUAL_MODE again (should stay in MANUAL_MODE, no mode change)")
            self.send_manual_mode_command()
        elif self.test_step == 2:
            self.get_logger().info("MANUAL-MODE-TEST: Step 3 - Sending IDLE command (should show exit logs with POSITION mode)")
            self.send_idle_command()
        elif self.test_step == 3:
            self.get_logger().info("MANUAL-MODE-TEST: Step 4 - Testing direct transition to MANUAL_MODE from IDLE again")
            self.send_manual_mode_command()
        elif self.test_step == 4:
            self.get_logger().info("MANUAL-MODE-TEST: Step 5 - Final transition back to IDLE")
            self.send_idle_command()
        elif self.test_step == 5:
            self.get_logger().info("MANUAL-MODE-TEST: Test sequence completed!")
            self.get_logger().info("MANUAL-MODE-TEST: Check logs for 'MODE: TEACHING_JOINT' when entering MANUAL_MODE")
            self.get_logger().info("MANUAL-MODE-TEST: and 'MODE: POSITION' when leaving MANUAL_MODE")
            self.test_timer.cancel()
            return
            
        self.test_step += 1
    
    def send_manual_mode_command(self):
        """Send MANUAL_MODE command to state machine"""
        msg = RobotStateAndTargetPose()
        msg.robot_next_state = 4  # MANUAL_MODE
        
        self.state_publisher.publish(msg)
        self.get_logger().info("MANUAL-MODE-TEST: Sent MANUAL_MODE command")
    
    def send_idle_command(self):
        """Send IDLE command to state machine"""
        msg = RobotStateAndTargetPose()
        msg.robot_next_state = 0  # IDLE
        
        self.state_publisher.publish(msg)
        self.get_logger().info("MANUAL-MODE-TEST: Sent IDLE command")

def main(args=None):
    rclpy.init(args=args)
    test_node = ManualModeTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
