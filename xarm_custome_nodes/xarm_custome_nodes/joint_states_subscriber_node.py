#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from sensor_msgs.msg import JointState

class JointStatesSubscriberNode(Node):
    def __init__(self):
        super().__init__('joint_states_subscriber_node')
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.joint_states_subscription  # prevent unused variable warning
        self.joint_info_publisher = self.create_publisher(JointNamesAndAngles, '/joint_info', 10)

    def joint_states_callback(self, msg):
        # Extract joint names and positions from JointState message
        joint_names = msg.name
        joint_positions = msg.position

        # Create a JointNamesAndAngles message to publish joint names and positions
        joint_info_msg = JointNamesAndAngles()
        joint_info_msg.names = joint_names
        joint_info_msg.positions = joint_positions

        # Set the timestamp in the header
        joint_info_msg.header.stamp = self.get_clock().now().to_msg()

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
