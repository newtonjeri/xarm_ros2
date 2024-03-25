#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from sensor_msgs.msg import JointState


class JointStatesSubscriberNode(Node):
    def __init__(self):
        super().__init__('joint_states_subscriber_node')

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
        self.timer_period = 0.5
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

        # Set the timestamp in the header
        # joint_info_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish joint names and positions to another topic
        self.joint_info_publisher.publish(joint_info_msg)
        self.get_logger().info("Published joint names and positions")


def main(args=None):
    rclpy.init(args=args)

    joint_states_subscriber_node = JointStatesSubscriberNode()

    rclpy.spin(joint_states_subscriber_node)

    joint_states_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
