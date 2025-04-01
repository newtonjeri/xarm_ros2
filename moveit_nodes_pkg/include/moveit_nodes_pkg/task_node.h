/* Copyright 2024 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef __TASK_NODE_H__
#define __TASK_NODE_H__

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <boost/filesystem.hpp>

namespace task_node
{

    class PickAndPlaceTask
    {
    public:
        PickAndPlaceTask(
            rclcpp::Node::SharedPtr &node,
            std::string &group_name);

        void reset_visualisation();

        geometry_msgs::msg::Pose getFramePose(
            const std::string &target_frame,
            const std::string &base_frame,
            const rclcpp::Time &time);

        // Method to initialise and setup the move group interfaces
        void init(const std::string &group_name);

        // Method to plan to a pose target
        void plan_and_execute_to_pose_goal(
            const geometry_msgs::msg::Pose &target_pose,
            bool is_constrained_planning);

        // Method to generate and execute to a joint space target
        void plan_and_execute_to_joint_space_goal(std::vector<double> &joint_values);

        // Method to generate and execute a cartesian path
        void plan_and_execute_cartesian_path(
            const geometry_msgs::msg::Pose &start_pose,
            const int iterations,
            const std::vector<double> step);

        // Method to plan and execute a trajectory with equality line constraint
        void plan_and_execute_with_line_equality_constraint(
            const geometry_msgs::msg::Pose start_pose,
            const geometry_msgs::msg::Pose target_pose,
            const std::string target_link,
            rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> line_dimensions);

        // Method to plan and execute a trajectory with equality plane constraint
        void plan_and_execute_with_plane_equality_constraint(
            const geometry_msgs::msg::Pose start_pose,
            const geometry_msgs::msg::Pose target_pose,
            const std::string target_link,
            rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> plane_dimensions,
            const Eigen::Vector3d &normal,
            double distance);

        // Method to plan and execute a trajectory with box constraints
        void plan_and_execute_with_box_constraint(
            const geometry_msgs::msg::Pose start_pose,
            const geometry_msgs::msg::Pose target_pose,
            const std::string link_name,
            rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> box_dimensions);

        // Method to plan and execute an orientation constrained trajectory
        void plan_and_execute_with_orientation_constraint(
            geometry_msgs::msg::Pose &start_pose,
            const geometry_msgs::msg::Pose &orientation_pose,
            const std::vector<double> relative_position);

        // Method to calculate an offset pose from the current pose
        geometry_msgs::msg::Pose get_relative_pose(
            geometry_msgs::msg::Pose pose,
            const std::vector<double> relative_pos);

        // Function to visualise a plane in rviz
        void visualize_plane(
            const Eigen::Vector3d &normal,
            double distance);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
    };
}

#endif