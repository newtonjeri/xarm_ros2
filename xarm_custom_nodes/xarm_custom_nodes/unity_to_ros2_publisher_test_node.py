#!/usr/env/bin python3
import rclpy
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
import numpy as np

PI = np.pi

class UnityToRos2PublisherTestNode(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_positions: JointNamesAndAngles.positions = [PI, 0.0, 0.0, 0.0, 0.0, 0.0, PI/2]

        # Publish topic
        self.publish_topic = "/unity_to_ros2_joint_info"

        # Publisher
        self.publisher = self.create_publisher(JointNamesAndAngles, self.publish_topic, 10)

        # Timer 
        self.timer = self.create_timer(1/60, self.publish_unity_joints)


    def publish_unity_joints(self):
        joints_info = JointNamesAndAngles()
        joints_info.names = self.joint_names
        joints_info.positions = self.joint_positions

        self.publisher.publish(joints_info)
        self.get_logger().info(f"Published Joint Positions {list(joints_info.positions)}")

def main(args = None):
    rclpy.init()
    unity_ros2_publisher = UnityToRos2PublisherTestNode('unity_to_ros2_publisher_node')
    rclpy.spin(unity_ros2_publisher)
    unity_ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()