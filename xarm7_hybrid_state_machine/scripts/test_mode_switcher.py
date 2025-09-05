#!/usr/bin/env python3

"""
Quick test to verify the mode switcher is working with the new logging format.
This script simply runs the mode switcher for a few seconds to show the improved logs.
"""

import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotMode
import time

class ModeTestNode(Node):
    def __init__(self):
        super().__init__('mode_test')
        
        # Publisher for robot mode
        self.mode_publisher = self.create_publisher(RobotMode, '/robot_mode', 10)
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        self.get_logger().info("MODE-TEST: Starting mode switching test...")
        
        # Test switching between modes
        self.test_timer = self.create_wall_timer(3.0, self.test_mode_switch)
        self.test_step = 0
        
    def test_mode_switch(self):
        """Test mode switching"""
        
        if self.test_step == 0:
            self.get_logger().info("MODE-TEST: Sending MANUAL mode (2)")
            msg = RobotMode()
            msg.data = 2  # MANUAL mode (TEACHING_JOINT)
            self.mode_publisher.publish(msg)
        elif self.test_step == 1:
            self.get_logger().info("MODE-TEST: Sending MOVEIT mode (0)")
            msg = RobotMode()
            msg.data = 0  # MOVEIT mode (POSITION)
            self.mode_publisher.publish(msg)
        elif self.test_step == 2:
            self.get_logger().info("MODE-TEST: Test completed!")
            self.test_timer.cancel()
            return
            
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    test_node = ModeTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
