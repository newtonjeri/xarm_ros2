#!/usr/bin/env python3

#----------------------------------------------------------------#
# Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
# Software License Agreement (BSD License)
#
# Author: Newton Kariuki <newtonkaris45@gmail.com>
#----------------------------------------------------------------#

import rclpy
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from geometry_msgs.msg import Pose
import csv
import os
from datetime import datetime

class JointsInfoSaverNode(Node):
    def __init__(self):
        super().__init__('joint_info_saver_node')
        
        # Initialize variables
        self.tcp_position = []
        self.start_time = ''
        self.trigger = True
        
        # Create the execution directory if it doesn't exist
        self.execution_dir = 'data'
        if not os.path.exists(self.execution_dir):
            os.makedirs(self.execution_dir)
        
        # File to store joint data
        self.exec_filename = f"{self.execution_dir}/execution_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # Create subscribers
        self.joint_info_subscription = self.create_subscription(
            JointNamesAndAngles,
            '/joint_info',
            self.joint_info_callback,
            10
        )
        
        self.tcp_position_subscriber = self.create_subscription(
            Pose,
            '/tcp_pose',
            self.tcp_position_callback,
            10
        )
        
        self.get_logger().info(f'Joint Data Saver Node started - Saving to {self.exec_filename}')

    def tcp_position_callback(self, msg):
        # Extract TCP pose (position only)
        self.tcp_position = [msg.position.x, msg.position.y, msg.position.z]

    def joint_info_callback(self, msg):
        # Set start time on first message
        if self.trigger:
            self.start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            self.trigger = False
        
        # Calculate time delta
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        frame_time = datetime.strptime(current_time, "%Y-%m-%d %H:%M:%S.%f") - \
                    datetime.strptime(self.start_time, "%Y-%m-%d %H:%M:%S.%f")
        
        # Save data to CSV
        self.save_to_csv(
            msg.names, 
            msg.positions, 
            self.tcp_position,
            msg.velocities, 
            msg.timestamp,  # Use timestamp from the message
            frame_time
        )

    def save_to_csv(self, joint_names, positions, tcp_position, joint_velocities, timestamp, time_delta):
        if len(joint_names) != len(positions) or len(joint_names) != len(joint_velocities):
            self.get_logger().error("Error: Arrays must be of equal size.")
            return

        with open(self.exec_filename, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            if csvfile.tell() == 0:  # Check if the file is empty
                csvwriter.writerow(['Joint Name', 'Joint Position', 'tcp position x', 'tcp position y', 'tcp position z', 'Joint Velocity', 'Time data is sent', 'Time Delta'])
        
            # Write joint data to CSV
            for joint_name, position, velocity in zip(joint_names, tcp_position, joint_velocities):
                csvwriter.writerow([joint_name, position, tcp_position[0], tcp_position[1], tcp_position[2], velocity, timestamp, time_delta])

def main(args=None):
    rclpy.init(args=args)
    
    joints_info_saver_node = JointsInfoSaverNode()
    
    try:
        rclpy.spin(joints_info_saver_node)
    except KeyboardInterrupt:
        pass
    
    joints_info_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()