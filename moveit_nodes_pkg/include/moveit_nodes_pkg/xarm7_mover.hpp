/* Copyright 2024 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef __XARM_MOVER_H__
#define __XARM_MOVER_H__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "xarm_msgs/msg/accept_plan.hpp"

namespace xarm7_mover
{

    const double max_velocity_scaling_factor = 0.1;     // [move_group_interface] default is 0.1
    const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1
    class Xarm7Mover
    {
    public:
        Xarm7Mover(
            rclcpp::Node::SharedPtr &node,
            const std::string &group_name) : node_(node)
        {
            init(group_name);

            moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());
            pose_subscriber = node_->create_subscription<geometry_msgs::msg::Pose>(
                POSE_SUBSCRIPTION_TOPIC, 10, std::bind(&Xarm7Mover::poseCallback, this, std::placeholders::_1));

            plan_publisher = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(PUBLISH_TOPIC, 10);

            plan_accepted_subscriber = node_->create_subscription<xarm_msgs::msg::AcceptPlan>(
                PLAN_ACCEPTED_TOPIC, 10, std::bind(&Xarm7Mover::planAcceptedCallback, this, std::placeholders::_1));
        }

    private:
        void init(const std::string &group_name);

        void poseCallback(const geometry_msgs::msg::Pose &pose_msg);

        void planCallback(const moveit_msgs::msg::RobotTrajectory &plan_trajectory);

        void planAcceptedCallback(const xarm_msgs::msg::AcceptPlan &accept_plan_msg);

        moveit::planning_interface::MoveGroupInterface::Plan planToTargetPose(const geometry_msgs::msg::Pose &pose_);

        moveit_msgs::msg::RobotTrajectory planCartesianPath(
            const geometry_msgs::msg::Pose &start_pose_,
            const std::vector<double> direction);

        void visualisePoint(const geometry_msgs::msg::Pose pose_);
        void resetVisualisation();

        rclcpp::Node::SharedPtr node_;

        const std::string POSE_SUBSCRIPTION_TOPIC = "/pose_from_unity";
        const std::string PUBLISH_TOPIC = "/plan_from_moveit";
        const std::string PLAN_ACCEPTED_TOPIC = "/accept_plan";

        bool is_accept_plan = false;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

        // Subscriber to get pose updates from unity
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
        // Subscriber to get plan accepted status
        rclcpp::Subscription<xarm_msgs::msg::AcceptPlan>::SharedPtr plan_accepted_subscriber;


        // plan trajectory Publisher to unity
        rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr plan_publisher;

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        // Prev plan
        moveit::planning_interface::MoveGroupInterface::Plan prev_plan;

        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    };
}

#endif //__XARM_MOVER_H__