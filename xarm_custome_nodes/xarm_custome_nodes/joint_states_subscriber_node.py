#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from sensor_msgs.msg import JointState

import csv
import os
from datetime import datetime

class JointStatesSubscriberNode(Node):
    def __init__(self):
        super().__init__('joint_states_subscriber_node')
        self.freq = 60.0

        # Start time of the execution
        self.start_time = ''

        # Create boolean trigger to start saving data
        self.trigger = True
                
        # Create the execution directory if it doesn't exist
        self.execution_dir = 'data'
        if not os.path.exists(self.execution_dir):
            os.makedirs(self.execution_dir)

        # File to store joint data shared to unity
        self.exec_filename = f"{self.execution_dir}/execution_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.plan_filename = f"{self.execution_dir}/plan_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.joint_states_subscription  # prevent unused variable warning
        self.joint_info_publisher = self.create_publisher(JointNamesAndAngles, '/joint_info', 10)
        self.timer_period = 1/self.freq
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)

    def joint_states_callback(self, msg):
        # Extract joint names and positions from JointState message
        self.joint_names = msg.name
        self.joint_positions = msg.position
        self.joint_velocities = msg.velocity
        self.joint_positions = np.array(self.joint_positions)

        # Convert angles into degrees
        joint_positions_deg = np.rad2deg(self.joint_positions)
        self.joint_positions: JointNamesAndAngles.positions = list(joint_positions_deg)


    def publisher_callback(self):

        # Create a JointNamesAndAngles message to publish joint names and positions
        joint_info_msg = JointNamesAndAngles()
        joint_info_msg.names = self.joint_names
        joint_info_msg.positions = self.joint_positions
        joint_info_msg.velocities = self.joint_velocities

        if self.trigger == True:
            self.start_time = datetime.now().strftime("%m/%d/%Y %H:%M:%S.%f ")
            self.trigger = False

        # Time data is sent
        self.time_data_sent = datetime.now().strftime("%m/%d/%Y %H:%M:%S.%f ")
        joint_info_msg.timestamp = self.time_data_sent
        # Publish joint names and positions to another topic
        self.joint_info_publisher.publish(joint_info_msg)


        self.save_to_csv(joint_info_msg.names, joint_info_msg.positions, \
                         joint_info_msg.velocities, self.time_data_sent,\
                        datetime.strptime(self.time_data_sent, "%m/%d/%Y %H:%M:%S.%f ") - datetime.strptime(self.start_time, "%m/%d/%Y %H:%M:%S.%f "))
                        
        self.get_logger().info("Published joint names and positions")

    def save_to_csv(self, joint_names, positions, velocities, timestamp, time_delta):
        if len(joint_names) != len(positions) or len(joint_names) != len(velocities):
            print("Error: Arrays must be of equal size.")
            return

        with open(self.exec_filename, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            if csvfile.tell() == 0:  # Check if the file is empty
                csvwriter.writerow(['Joint Name', 'Position', 'Velocity', 'Time data is sent', 'Time Delta'])
            for joint_name, position, velocity in zip(joint_names, positions, velocities):
                csvwriter.writerow([joint_name, position, velocity, timestamp, time_delta])



def main(args=None):
    rclpy.init(args=args)

    joint_states_subscriber_node = JointStatesSubscriberNode()

    rclpy.spin(joint_states_subscriber_node)

    joint_states_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
