/* Copyright 2024 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef __VOICE_COMMAND_SERVER__
#define __VOICE_COMMAND_SERVER__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <xarm_msgs/action/voice_command.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace voice_command
{

    class VoiceCommandServer : public rclcpp::Node
    {
    public:
        VoiceCommandServer(
            const rclcpp::NodeOptions &options)
            : Node("voice_command_server_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
        {
            moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());
            server_ = rclcpp_action::create_server<xarm_msgs::action::VoiceCommand>(
                this, "voice_command_server", std::bind(&VoiceCommandServer::goal_callback, this, _1, _2),
                std::bind(&VoiceCommandServer::cancel_callback, this, _1),
                std::bind(&VoiceCommandServer::accepted_callback, this, _1));
        }
        void reset_visualisation();

        // Method to generate and execute a plan to a joint space target
        void plan_and_execute_to_joint_space_goal(
            moveit::planning_interface::MoveGroupInterface &move_group,
            std::vector<double> &joint_values);

        // Method to generate and execute a cartesian path
        void plan_and_execute_cartesian_path(
            moveit::planning_interface::MoveGroupInterface &move_group,
            geometry_msgs::msg::Pose start_pose,
            const int iterations,
            const std::vector<double> direction);

        // Method to plan and execute an orientation constrained trajectory
        void plan_and_execute_with_orientation_constraint(
            geometry_msgs::msg::Pose &start_pose,
            const geometry_msgs::msg::Pose &orientation_pose,
            const std::vector<double> relative_position);

        // Method to calculate an offset pose from the current pose
        geometry_msgs::msg::Pose get_relative_pose(
            geometry_msgs::msg::Pose pose,
            const std::vector<double> relative_pos);

        // Method to get the pose of a frame relative to another frame
        geometry_msgs::msg::Pose get_frame_pose(
            const std::string target_frame,
            const std::string base_frame,
            const rclcpp::Time &time);

        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const xarm_msgs::action::VoiceCommand::Goal> goal);

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle);

        void accepted_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle);

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<xarm_msgs::action::VoiceCommand>::SharedPtr server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
    };
}

#endif