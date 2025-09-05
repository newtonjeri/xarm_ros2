#!/usr/bin/env python3

"""
Test script to demonstrate state synchronization between state machine and mode switcher.
This script publishes test commands to the state machine to show the improved logging format.
"""

import rclpy
from rclcpp.node import Node
from xarm_msgs.msg import RobotStateAndTargetPose
from geometry_msgs.msg import Pose
import time

class StateMachineTestNode(Node):
    def __init__(self):
        super().__init__('state_machine_test')
        
        # Publisher for state commands
        self.state_publisher = self.create_publisher(
            RobotStateAndTargetPose, 
            '/xarm7_state_topic', 
            10
        )
        
        # Wait for publishers to be ready
        time.sleep(1)
        
        self.get_logger().info("STATE-MACHINE-TEST: Starting test sequence...")
        
        # Create test timer
        self.test_timer = self.create_wall_timer(5.0, self.test_sequence)
        self.test_step = 0
        
    def test_sequence(self):
        """Test sequence to demonstrate state machine transitions"""
        
        if self.test_step == 0:
            self.send_moving_command()
        elif self.test_step == 1:
            self.send_manual_mode_command()
        elif self.test_step == 2:
            self.send_idle_command()
        elif self.test_step == 3:
            self.get_logger().info("STATE-MACHINE-TEST: Test sequence completed!")
            self.test_timer.cancel()
            return
            
        self.test_step += 1
    
    def send_moving_command(self):
        """Send MOVING command to state machine"""
        msg = RobotStateAndTargetPose()
        msg.robot_next_state = 1  # MOVING
        
        # Set test pose
        msg.target_pose_1.position.x = 0.5
        msg.target_pose_1.position.y = 0.1
        msg.target_pose_1.position.z = 0.3
        msg.target_pose_1.orientation.w = 1.0
        
        msg.target_pose_2 = msg.target_pose_1
        
        self.state_publisher.publish(msg)
        self.get_logger().info("STATE-MACHINE-TEST: Sent MOVING command")
    
    def send_manual_mode_command(self):
        """Send MANUAL_MODE command to state machine"""
        msg = RobotStateAndTargetPose()
        msg.robot_next_state = 4  # MANUAL_MODE
        
        self.state_publisher.publish(msg)
        self.get_logger().info("STATE-MACHINE-TEST: Sent MANUAL_MODE command")
    
    def send_idle_command(self):
        """Send IDLE command to state machine"""
        msg = RobotStateAndTargetPose()
        msg.robot_next_state = 0  # IDLE
        
        self.state_publisher.publish(msg)
        self.get_logger().info("STATE-MACHINE-TEST: Sent IDLE command")

def main(args=None):
    rclpy.init(args=args)
    test_node = StateMachineTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
