#ifndef __MOVEITINCLUDE_NODE_HPP__
#define __MOVEITINCLUDE_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <functional> // For std::function
#include <thread>

namespace moveitinclude
{

    const double max_velocity_scaling_factor = 0.1;     // [move_group_interface] default is 0.1
    const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

    class MoveitIncludeNode
    {

    public:
        MoveitIncludeNode(rclcpp::Node::SharedPtr &node, const std::string &group_name) : node_(node)
        {
            init(group_name);
        }

        void init(const std::string &group_name)
        {
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
            moveit_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());
            move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
            moveit_visual_tools->deleteAllMarkers();
            moveit_visual_tools->loadRemoteControl();
        }

        bool planToTargetPose(const geometry_msgs::msg::Pose &target_pose, bool stop_moving = false)
        {
            if (stop_moving)
            {
                move_group->stop();
                RCLCPP_WARN(rclcpp::get_logger("moveitinclude_node"), "Robot motion stopped!!");
                return false; // Indicate failure
            }

            bool success = move_group->setPoseTarget(target_pose);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveitinclude_node"),
                             "Failed to set the pose target. Please ensure target is reachable by the robot");
                return false; // Indicate failure
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_;
            success = (move_group->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
            draw_trajectory_tool_path(plan_);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveitinclude_node"),
                             "Failed to plan a path to the target pose. Please ensure target and destination are reachable by the robot");
                return false; // Indicate failure
            }
            else
            {
                move_group->execute(plan_);
                current_pose = target_pose;
                return true; // Indicate success
            }
        }

        void planToJointSpaceTarget(const std::vector<double> &target_joint_positions, bool stop_moving = false)
        {
            if (stop_moving)
            {
                move_group->stop();
                RCLCPP_WARN(rclcpp::get_logger("moveitinclude_node"), "Robot motion stopped!!");
                return;
            }

            bool success = move_group->setJointValueTarget(target_joint_positions);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveitinclude_node"),
                             "Failed to set the joint value target. Please ensure target is reachable by the robot");
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_;
            success = (move_group->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
            draw_trajectory_tool_path(plan_);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveitinclude_node"),
                             "Failed to plan a path to the target pose. Please ensure target and destination are reachable by the robot");
                return;
            }
            else
            {
                move_group->execute(plan_);
            }
        }

        void gripperOpenAndClose(const std::vector<double> &target_joint_positions)
        {

            bool success = move_group->setJointValueTarget(target_joint_positions);

            moveit::planning_interface::MoveGroupInterface::Plan plan_;
            success = (move_group->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
            draw_trajectory_tool_path(plan_);

            if (!success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("moveitinclude_node"),
                             "Pose-plan Fail -> Failed to plan a path to the target pose. Please ensure target is within gripper joints range");
                return;
            }
            else
            {
                move_group->execute(plan_);
            }
        }

        void planCartesianPath(
            const geometry_msgs::msg::Pose &start_pose_,
            const int iterations,
            const std::vector<double> direction)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::Pose start_pose = start_pose_;

            waypoints.push_back(start_pose);
            RCLCPP_INFO(rclcpp::get_logger("moveitinclude_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                        start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
            // waypoints.push_back(start_pose);
            for (int i = 0; i < iterations; i++)
            {
                start_pose.position.x += (direction[0] / iterations);
                start_pose.position.y += (direction[1] / iterations);
                start_pose.position.z += (direction[2] / iterations);
                waypoints.push_back(start_pose);
            }
            RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                        start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
            // Define trajectory for the approach
            moveit_msgs::msg::RobotTrajectory trajectory_approach;
            // Compute the Cartesian path for the approach
            double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory_approach);
            if (fraction < 0.90)
            {
                RCLCPP_WARN(rclcpp::get_logger("moveitinclude_node"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
                current_pose = *waypoints.begin();
            }
            else
            {
                // draw_trajectory_tool_path(trajectory_approach);
                // Execute the approach trajectory
                move_group->execute(trajectory_approach);
                current_pose = waypoints.back();
            }
        }

        geometry_msgs::msg::Pose current_pose;

        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools;

        // Create a closures for visualization
        std::function<void(const std::string &)> prompt =
            [this](const std::string &text)
        {
            moveit_visual_tools->prompt(text);
            moveit_visual_tools->trigger(); // Ensure RViz visual tools are updated
        };

        std::function<void(const moveit::planning_interface::MoveGroupInterface::Plan &)> draw_trajectory_tool_path =
            [this](const moveit::planning_interface::MoveGroupInterface::Plan &trajectory)
        {
            auto jmg = move_group->getRobotModel()->getJointModelGroup("xarm7");
            moveit_visual_tools->publishTrajectoryLine(trajectory.trajectory_, jmg);
            moveit_visual_tools->trigger();
        };

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

    private:
        // Member Variables
        rclcpp::Node::SharedPtr node_;
    };
}

#endif
