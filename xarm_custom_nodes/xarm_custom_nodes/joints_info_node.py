#!/usr/bin/env python3

#----------------------------------------------------------------#
# Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
# Software License Agreement (BSD License)
#
# Author: Newton Kariuki <newtonkaris45@gmail.com>
#----------------------------------------------------------------#

import rclpy
import numpy as np
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from datetime import datetime

class JointInfoNode(Node):
    def __init__(self):
        super().__init__('joint_info_publisher_node')
        self.freq = 60.0    # Publish frequency
        
        # Initialize variables
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.tcp_position = []
        
        # Create subscribers
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.tcp_position_subscriber = self.create_subscription(
            Pose,
            '/tcp_pose',
            self.tcp_position_callback,
            10
        )
        
        # Create publisher
        self.joint_info_publisher = self.create_publisher(JointNamesAndAngles, '/joint_info', 10)
        
        # Create timer for 60Hz publishing
        self.timer_period = 1/self.freq
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)
        
        self.get_logger().info(f'Joint Publisher Node started - Publishing at {self.freq}Hz')

    def joint_states_callback(self, msg):
        # Extract joint names and positions from JointState message
        self.joint_names = msg.name
        self.joint_positions = msg.position
        self.joint_velocities = msg.velocity
        
        # Convert to numpy array for processing
        joint_positions_array = np.array(self.joint_positions)
        
        # Convert angles from radians to degrees
        joint_positions_deg = np.rad2deg(joint_positions_array)
        self.joint_positions = list(joint_positions_deg)

    def tcp_position_callback(self, msg):
        # Extract TCP pose (position only)
        self.tcp_position = [msg.position.x, msg.position.y, msg.position.z]

    def publisher_callback(self):
        # Only publish if we have received joint data
        if not self.joint_names:
            return
            
        # Create a JointNamesAndAngles message
        joint_info_msg = JointNamesAndAngles()
        joint_info_msg.names = self.joint_names
        joint_info_msg.positions = self.joint_positions
        joint_info_msg.velocities = self.joint_velocities
        joint_info_msg.tcp_position = self.tcp_position
        
        # Add timestamp
        joint_info_msg.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        
        # Publish the message
        self.joint_info_publisher.publish(joint_info_msg)

        self.get_logger().info(f'Published joint info: {joint_info_msg.names} at {joint_info_msg.timestamp}')
        self.get_logger().debug(f'Joint positions: {joint_info_msg.positions}, TCP pose: {self.tcp_position}')

def main(args=None):
    rclpy.init(args=args)
    
    joint_info_node = JointInfoNode()
    
    try:
        rclpy.spin(joint_info_node)
    except KeyboardInterrupt:
        pass
    
    joint_info_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()