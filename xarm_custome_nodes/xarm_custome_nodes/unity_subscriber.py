#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from xarm_msgs.msg import JointNamesAndAngles
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UnitySubscriberNode(Node):
    def __init__(self):
        super().__init__('unity_subscriber')

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_positions = []

        self.subscription_topic = "/unity_to_ros2_joint_info"
        self.publish_topic = "xarm7_traj_controller/joint_trajectory"

        self.unity_subscriber_ = self.create_subscription(
            JointNamesAndAngles,
            self.subscription_topic,
            self.joint_states_callback,
            10
        )

        self.unity_subscriber_  # prevent unused variable warning
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.publish_topic,
            10)
        
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)

    def joint_states_callback(self, msg):
        # Extract joint names and positions from JointNamesAndAngle message
        self.joint_names = msg.names
        self.joint_positions = msg.positions


    def publisher_callback(self):
        # Create a JointTrajectory message to publish joint names
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.joint_names = self.joint_names

        # Create a JointTrajectoryPoint message to publish joint positions
        joint_points = JointTrajectoryPoint()
        joint_points.positions = self.joint_positions

        joint_points.time_from_start = Duration(sec=2)
        joint_traj_msg.points.append(joint_points)
        # Publish joint names and positions to another topic
        self.joint_trajectory_publisher.publish(joint_traj_msg)



def main(args=None):
    rclpy.init(args=args)

    unity_subscriber = UnitySubscriberNode()

    rclpy.spin(unity_subscriber)

    unity_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
