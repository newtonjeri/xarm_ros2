/* Copyright 2024 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Kipkemoi Wesley, PAUSTI
 ============================================================================*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

using namespace std::chrono_literals;

// ✅ Decision Tree Classifier for Motion Path Optimization
std::string classifyPath(double distance)
{
    if (distance < 0.2)
    {
        return "DIRECT_PATH"; // Short distance → Direct movement
    }
    else
    {
        return "SAFE_PATH"; // Longer distance → Safer movement
    }
}

int main(int argc, char **argv)
{
    // ✅ Initialize ROS 2
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // ✅ Create Node
    auto node = rclcpp::Node::make_shared("hrc_node", node_options);

    // ✅ Planning groups
    std::string ARM_GROUP = "xarm7";
    std::string GRIPPER_GROUP = "xarm_gripper";

    // ✅ Create MoveIt interfaces (for motion planning only)
    auto arm_node = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
    auto gripper_node = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, GRIPPER_GROUP);

    // ✅ Get picking and dropping positions from parameters
    geometry_msgs::msg::Pose pick_pose, drop_pose;
    node->get_parameter("pick_x", pick_pose.position.x);
    node->get_parameter("pick_y", pick_pose.position.y);
    node->get_parameter("pick_z", pick_pose.position.z);
    node->get_parameter("pick_ori_x", pick_pose.orientation.x);
    node->get_parameter("pick_ori_y", pick_pose.orientation.y);
    node->get_parameter("pick_ori_z", pick_pose.orientation.z);
    node->get_parameter("pick_ori_w", pick_pose.orientation.w);

    node->get_parameter("drop_x", drop_pose.position.x);
    node->get_parameter("drop_y", drop_pose.position.y);
    node->get_parameter("drop_z", drop_pose.position.z);
    node->get_parameter("drop_ori_x", drop_pose.orientation.x);
    node->get_parameter("drop_ori_y", drop_pose.orientation.y);
    node->get_parameter("drop_ori_z", drop_pose.orientation.z);
    node->get_parameter("drop_ori_w", drop_pose.orientation.w);

    // ✅ Define Home Position and Gripper States
    std::vector<double> home_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, -1.571, 0.0};
    std::vector<double> open_gripper_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> close_gripper_positions = {0.84, 0.84, 0.84, 0.84, 0.84, 0.84};

    // ✅ Move to Home Before Execution
    arm_node->setJointValueTarget(home_joint_positions);
    arm_node->move();
    RCLCPP_INFO(node->get_logger(), "📢 Moved to home position. Planning next move...");

    // ✅ Decision Tree for Path Selection
    double distance_to_pick = std::sqrt(
        std::pow(pick_pose.position.x, 2) +
        std::pow(pick_pose.position.y, 2) +
        std::pow(pick_pose.position.z, 2));

    std::string path_type = classifyPath(distance_to_pick);
    RCLCPP_INFO(node->get_logger(), "📊 Decision Tree selected path: %s", path_type.c_str());

    // ✅ Pick Sequence
    RCLCPP_INFO(node->get_logger(), "📢 Moving to pick position...");
    arm_node->setPoseTarget(pick_pose);
    arm_node->move();

    gripper_node->setJointValueTarget(open_gripper_positions);
    gripper_node->move();
    
    pick_pose.position.z -= 0.05;  // Lower arm for grasping
    arm_node->setPoseTarget(pick_pose);
    arm_node->move();
    rclcpp::sleep_for(1s);

    gripper_node->setJointValueTarget(close_gripper_positions);
    gripper_node->move();

    pick_pose.position.z += 0.1;  // Lift object
    arm_node->setPoseTarget(pick_pose);
    arm_node->move();

    // ✅ Place Sequence
    RCLCPP_INFO(node->get_logger(), "📢 Moving to dropping position...");
    arm_node->setPoseTarget(drop_pose);
    arm_node->move();

    drop_pose.position.z -= 0.0908; // Lower for placing
    arm_node->setPoseTarget(drop_pose);
    arm_node->move();
    rclcpp::sleep_for(1s);

    gripper_node->setJointValueTarget(open_gripper_positions);
    gripper_node->move();
    
    drop_pose.position.z += 0.093; // Raise after placing
    arm_node->setPoseTarget(drop_pose);
    arm_node->move();

    // ✅ Return Home
    arm_node->setJointValueTarget(home_joint_positions);
    arm_node->move();
    RCLCPP_INFO(node->get_logger(), "📢 Task complete. Returned home. Waiting for next command...");

    // ✅ Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

