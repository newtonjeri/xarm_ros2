/* Copyright 2024 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <iostream>
#include "moveit_nodes_pkg/xarm7_mover.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm7_mover_node");

namespace xarm7_mover
{
    void Xarm7Mover::init(const std::string &group_name)
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
        RCLCPP_INFO(LOGGER, "Available Planning Groups:");
        std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(), 
                  std::ostream_iterator<std::string>(std::cout, ", "));
        
        move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
        
        RCLCPP_INFO(LOGGER, "Move group initialized with velocity scaling: %.2f, acceleration scaling: %.2f", 
                    max_velocity_scaling_factor, max_acceleration_scaling_factor);
    }

    void Xarm7Mover::poseCallback(const geometry_msgs::msg::Pose &pose_msg)
    {
        RCLCPP_INFO(LOGGER, "Received pose goal \n position x:%f y:%f z:%f \n  Orientation x:%f y:%f z:%f w:%f",
                    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
                    pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

        try {
            geometry_msgs::msg::Pose pose_ = pose_msg;
            pose_.orientation.x = 1.0;
            pose_.orientation.y = 0.0;
            pose_.orientation.z = 0.0;
            pose_.orientation.w = 0.0;
            
            Xarm7Mover::visualisePoint(pose_msg);
            plan_ = Xarm7Mover::planToTargetPose(pose_);

            // TODO: implement logic to select between cartesian path planning and planning to a target pose
            Xarm7Mover::planCallback(plan_.trajectory);
        }
        catch (const std::exception &ex) {
            RCLCPP_ERROR(LOGGER, "Exception in pose callback: %s", ex.what());
        }
    }

    moveit::planning_interface::MoveGroupInterface::Plan Xarm7Mover::planToTargetPose(const geometry_msgs::msg::Pose &pose_)
    {
        bool success = move_group->setPoseTarget(pose_);

        if (!success)
        {
            RCLCPP_WARN(LOGGER, "Set Target Pose is not reachable!");
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            RCLCPP_ERROR(LOGGER,
                         "Failed to plan a path to the target pose. Please ensure target and destination are reachable by the robot");
            throw std::runtime_error("Planning to target pose failed");
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Path planning successful");
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
        RCLCPP_INFO(LOGGER, "Start pose: x=%.3f, y=%.3f, z=%.3f, ox=%.3f, oy=%.3f, oz=%.3f, ow=%.3f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, 
                    start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);

        start_pose.position.x += direction[0];
        start_pose.position.y += direction[1];
        start_pose.position.z += direction[2];
        waypoints.push_back(start_pose);
        RCLCPP_INFO(LOGGER, "Target pose: x=%.3f, y=%.3f, z=%.3f, ox=%.3f, oy=%.3f, oz=%.3f, ow=%.3f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, 
                    start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
        
        // Define trajectory for the approach
        moveit_msgs::msg::RobotTrajectory planned_path;
        // Compute the Cartesian path for the approach (updated API without deprecated jump_threshold)
        double fraction = move_group->computeCartesianPath(waypoints, 0.005, planned_path);
        if (fraction < 0.90)
        {
            RCLCPP_WARN(LOGGER, "Cartesian path for approach not fully planned, fraction: %.2f", fraction);
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Cartesian path planned successfully, fraction: %.2f", fraction);
            // Execute the approach trajectory
            // move_group->execute(planned_path);
            return planned_path;
        }
        
        return planned_path; // Return even if not fully planned
    }

    void Xarm7Mover::planCallback(const moveit_msgs::msg::RobotTrajectory &plan_trajectory)
    {
        // Publish plan
        plan_publisher->publish(plan_trajectory);
        RCLCPP_INFO(LOGGER, "Published Plan with %zu trajectory points", 
                    plan_trajectory.joint_trajectory.points.size());
    }

    void Xarm7Mover::planAcceptedCallback(const xarm_msgs::msg::AcceptPlan &accept_plan_msg)
    {
        is_accept_plan = accept_plan_msg.is_plan_accepted;
        RCLCPP_INFO(LOGGER, "Received Accept Plan: %s", accept_plan_msg.is_plan_accepted ? "True" : "False");
        
        if (is_accept_plan)
        {
            try 
            {
                auto result = move_group->execute(prev_plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(LOGGER, "Plan executed successfully");
                }
                else
                {
                    RCLCPP_ERROR(LOGGER, "Plan execution failed");
                }
                is_accept_plan = false;
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(LOGGER, "Exception during plan execution: %s", ex.what());
                is_accept_plan = false;
            }
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Plan execution cancelled");
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
    fprintf(stderr, "[xarm7_mover_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm7_mover_node", node_options);

    // Declare parameter only if not already declared
    if (!node->has_parameter("group_name")) {
        node->declare_parameter<std::string>("group_name", "xarm7");
    }

    std::string group_name;
    node->get_parameter("group_name", group_name);

    RCLCPP_INFO(node->get_logger(), "xarm7_mover_node starting with group: %s", group_name.c_str());

    try {
        // Arm group object
        auto arm_node = std::make_shared<xarm7_mover::Xarm7Mover>(node, group_name);
        
        signal(SIGINT, exit_sig_handler);
        
        RCLCPP_INFO(node->get_logger(), "xarm7_mover_node ready");
        rclcpp::spin(node);
    }
    catch (const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", ex.what());
        return -1;
    }
    
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "xarm7_mover_node shutdown");
    return 0;
}