#!/usr/bin/env python

import sys
import copy
import numpy as np
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from niryo_moveit_interfaces.msg import NiryoUnityTargetPoseToROS, NiryoROSJointsToUnity
import tf.transformations as transformations

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

def rotate_pose(pose, roll, pitch, yaw):
    roll = np.deg2rad([roll])
    pitch = np.deg2rad([pitch])
    yaw = np.deg2rad([yaw])
    rotation_matrix = transformations.euler_matrix(roll, pitch, yaw)

    current_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    current_pose_matrix = transformations.quaternion_matrix(current_orientation)
    current_pose_matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

    new_pose_matrix = transformations.concatenate_matrices(rotation_matrix, current_pose_matrix)

    new_position = new_pose_matrix[:3, 3]
    new_orientation = transformations.quaternion_from_matrix(new_pose_matrix)

    new_pose = Pose()
    new_pose.position.x = new_position[0]
    new_pose.position.y = new_position[1]
    new_pose.position.z = new_position[2]
    new_pose.orientation.x = new_orientation[0]
    new_pose.orientation.y = new_orientation[1]
    new_pose.orientation.z = new_orientation[2]
    new_pose.orientation.w = new_orientation[3]

    return new_pose

class MoveItServer(Node):
    def __init__(self):
        super().__init__('niryo_moveit_server')
        self.planning = False
        self.motionPlan = None
        self.pub = self.create_publisher(NiryoROSJointsToUnity, 'niryo_ROS_joints_to_unity', 10)
        self.sub = self.create_subscription(NiryoUnityTargetPoseToROS, 'niryo_unity_target_pose_to_ROS', self.niryo_unity_callback, 10)

        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_pose_reference_frame('base_link')  # Assuming 'base_link' is the robot's base frame

        self.get_logger().info("Ready to plan")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz loop rate

    def plan_trajectory(self, move_group, destination_pose, start_joint_angles, robot_arm_rotation): 
        current_joint_state = JointState()
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        move_group.set_start_state(moveit_robot_state)

        rotated_destination_pose = rotate_pose(destination_pose, robot_arm_rotation.x, robot_arm_rotation.y, robot_arm_rotation.z)
        move_group.set_pose_target(rotated_destination_pose)

        plan = move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            raise Exception(exception_str)

        return plan[1]

    def niryo_unity_callback(self, msg):
        if not self.planning:
            self.planning = True
            self.motionPlan = self.plan_trajectory(self.move_group, msg.target_pose, msg.joints_input.joints, msg.robotArmRotation)

    def timer_callback(self):
        if self.motionPlan:
            self.get_logger().info("Sending Motion Plan to Unity")

            response_msg = NiryoROSJointsToUnity()
            if self.motionPlan and self.motionPlan.joint_trajectory.points:
                response_msg.trajectory = self.motionPlan
                self.pub.publish(response_msg)
            else:
                self.pub.publish(response_msg)
            
            self.move_group.clear_pose_targets()
            self.planning = False
            self.motionPlan = None

def main(args=None):
    rclpy.init(args=args)
    node = MoveItServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
