/* Copyright 2024 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "moveit_nodes_pkg/experiment_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pick_and_place_gear_node", node_options);

    // Spinning up a SingleThreadedExecutor for  the current state monitor to get information about the robot's state
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    std::string GRIPPER_GROUP = "xarm_gripper";

    // Arm group object
    auto arm_node = std::make_shared<experiment::ExperimentNode>(node, ARM_GROUP);
    // gripper group object
    auto gripper_node = std::make_shared<experiment::ExperimentNode>(node, GRIPPER_GROUP);

    // home joints positions
    std::vector<double> home_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, -1.571, 0.0};
    // Open gripper
    std::vector<double> open_gripper_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> open_halfway_gripper_positions = {0.40, 0.40, 0.40, 0.40, 0.40, 0.40};
    // Close gripper
    std::vector<double> close_gripper_positions = {0.84, 0.84, 0.84, 0.84, 0.84, 0.84};

    // Idler pick pose
    geometry_msgs::msg::Pose idler_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3357;
        msg.position.y = 0.0295;
        msg.position.z = 0.0558;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Idler place pose
    geometry_msgs::msg::Pose idler_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;
        msg.position.y = -0.3707;
        msg.position.z = 0.1407;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Pick idler gear
    arm_node->planToJointSpaceTarget(home_joint_positions);
    RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Picking idler gear");
    arm_node->planToTargetPose(idler_pick_pose);
    gripper_node->gripperOpenAndClose(open_gripper_positions);
    // arm_node->planCartesianPath(idler_pick_pose, 1, {0, 0, -0.04});
    idler_pick_pose.position.z += -0.05;
    arm_node->planToTargetPose(idler_pick_pose);
    rclcpp::sleep_for(1s);
    gripper_node->gripperOpenAndClose(close_gripper_positions);
    // arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.04});
    idler_pick_pose.position.z += 0.1;
    arm_node->planToTargetPose(idler_pick_pose);
    // Place idler gear
    arm_node->planToTargetPose(idler_place_pose);
    // arm_node->planCartesianPath(idler_place_pose, 1, {0, 0, -0.04});
    idler_place_pose.position.z += -0.0908;
    arm_node->planToTargetPose(idler_place_pose);
    rclcpp::sleep_for(1s);
    gripper_node->gripperOpenAndClose(open_gripper_positions);
    // arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.12});
    idler_place_pose.position.z += 0.093;
    arm_node->planToTargetPose(idler_place_pose);
    // Move to home pose
    arm_node->planToJointSpaceTarget(home_joint_positions);

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}