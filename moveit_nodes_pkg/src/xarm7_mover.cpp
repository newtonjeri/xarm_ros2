/* Copyright 2024 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "moveit_nodes_pkg/xarm7_mover.hpp"

namespace xarm7_mover
{
    void Xarm7Mover::init(const std::string &group_name)
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "End effector link: %s", move_group->getEndEffectorLink().c_str());
        move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    }

    void Xarm7Mover::poseCallback(const geometry_msgs::msg::Pose &pose_msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Received pose goal \n position x:%f y:%f z:%f \n  Orientation x:%f y:%f z:%f w:%f",
                    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
                    pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

        geometry_msgs::msg::Pose pose_ = pose_msg;
        pose_.orientation.x = 1.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 0.0;
        Xarm7Mover::visualisePoint(pose_msg);
        plan_ = Xarm7Mover::planToTargetPose(pose_);

        // TODO: implement logic to select between cartesian path planning and planning to a target pose

        Xarm7Mover::planCallback(plan_.trajectory_);
    }

    moveit::planning_interface::MoveGroupInterface::Plan Xarm7Mover::planToTargetPose(const geometry_msgs::msg::Pose &pose_)
    {
        bool success = move_group->setPoseTarget(pose_);

        if (!success)
        {
            RCLCPP_WARN(rclcpp::get_logger("xarm7_mover_node"), "Set Target Pose is not reachable!");
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            RCLCPP_ERROR(rclcpp::get_logger("xarm7_mover_node"),
                         "Failed to plan a path to the target pose. Please ensure target and destination are reachable by the robot");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Path planning successful");
            // if (is_accept_plan){
            //     move_group->execute(prev_plan);
            // }
            prev_plan = plan;
            return plan;
        }
    }

    moveit_msgs::msg::RobotTrajectory Xarm7Mover::planCartesianPath(
        const geometry_msgs::msg::Pose &start_pose_,
        const std::vector<double> direction)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose start_pose = start_pose_;
        waypoints.push_back(start_pose);
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
        // waypoints.push_back(start_pose);

        start_pose.position.x += direction[0];
        start_pose.position.y += direction[1];
        start_pose.position.z += direction[2];
        waypoints.push_back(start_pose);
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
        // Define trajectory for the approach
        moveit_msgs::msg::RobotTrajectory planned_path;
        // Compute the Cartesian path for the approach
        double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, planned_path);
        if (fraction < 0.90)
        {
            RCLCPP_WARN(rclcpp::get_logger("xarm7_mover_node"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
        }
        else
        {
            // Execute the approach trajectory
            // move_group->execute(planned_path);
            return planned_path;
        }
    }

    void Xarm7Mover::planCallback(const moveit_msgs::msg::RobotTrajectory &plan_trajectory)
    {
        // Publish plan
        plan_publisher->publish(plan_trajectory);
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Published Plan");
    }

    void Xarm7Mover::planAcceptedCallback(const xarm_msgs::msg::AcceptPlan &accept_plan_msg){
        is_accept_plan = accept_plan_msg.is_plan_accepted;
        RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Received Accept Plan: %s", accept_plan_msg.is_plan_accepted? "True" : "False");
        if (is_accept_plan){
            move_group->execute(prev_plan);
            is_accept_plan = false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("xarm7_mover_node"), "Plan execution cancelled");
        }
    }


    void Xarm7Mover::visualisePoint(const geometry_msgs::msg::Pose pose_){
        Xarm7Mover::resetVisualisation();
        moveit_visual_tools_->publishSphere(pose_, rviz_visual_tools::RED, 0.03);
        moveit_visual_tools_->trigger();
    }

    void Xarm7Mover::resetVisualisation()
    {
        // Resets the plannning by cleaning up markers
        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->trigger();
    }
}

void exit_sig_handler([[maybe_unused]] int signum)
{
    fprintf(stderr, "[xarm_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}
// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm7_mover_node", node_options);

    // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    // std::string GRIPPER_GROUP = "xarm_gripper";

    // Arm group object
    auto arm_node = std::make_shared<xarm7_mover::Xarm7Mover>(node, ARM_GROUP);
    // gripper group object
    // auto gripper_node = std::make_shared<xarm7_mover::Xarm7Mover>(node, GRIPPER_GROUP);

    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    // rclcpp::spin(gripper_node);
    rclcpp::shutdown();
}