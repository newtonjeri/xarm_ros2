#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <xarm_msgs/action/voice_command.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>
#include <vector>

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

            server_ = rclcpp_action::create_server<xarm_msgs::action::VoiceCommand>(
                this, "voice_command_server", std::bind(&VoiceCommandServer::goal_callback, this, _1, _2),
                std::bind(&VoiceCommandServer::cancel_callback, this, _1),
                std::bind(&VoiceCommandServer::accepted_callback, this, _1));
        }


        void plan_and_execute_cartesian_path(
            moveit::planning_interface::MoveGroupInterface &move_group,
            geometry_msgs::msg::Pose start_pose,
            const int iterations,
            const std::vector<double> direction)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;

            waypoints.push_back(start_pose);
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                        start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
            for (int i = 0; i < iterations; i++)
            {
                start_pose.position.x += (direction[0] / iterations);
                start_pose.position.y += (direction[1] / iterations);
                start_pose.position.z += (direction[2] / iterations);
                waypoints.push_back(start_pose);
            }
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                        start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
            // Define trajectory for the approach
            moveit_msgs::msg::RobotTrajectory trajectory_approach;
            // Compute the Cartesian path for the approach
            double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_approach);
            if (fraction < 0.90)
            {
                RCLCPP_WARN(rclcpp::get_logger("voice_command_server_node"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
                current_pose = *waypoints.begin();
            }
            else
            {
                // Execute the approach trajectory
                move_group.execute(trajectory_approach);
                current_pose = waypoints.back();
            }
        }

        void plan_and_execute_to_joint_space_goal(
            moveit::planning_interface::MoveGroupInterface &move_group,
            std::vector<double> &joint_values)
        {
            bool success = move_group.setJointValueTarget(joint_values);

            if (!success)
            {
                RCLCPP_WARN(rclcpp::get_logger("voice_command_server_node"), "Set JointValueTarget Out of Bounds");
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success)
            {
                throw std::runtime_error(
                    "Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
            }
            else
            {
                move_group.execute(plan);
            }
        }

        geometry_msgs::msg::Pose get_frame_pose(
            const std::string target_frame,
            const std::string base_frame,
            const rclcpp::Time &time = rclcpp::Time(0))
        {
            geometry_msgs::msg::Pose pose;
            try
            {
                // Lookup the transform from target_frame to base_frame
                geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                    base_frame,
                    target_frame,
                    time,
                    tf2::durationFromSec(1.0));

                // Extract the translation and rotation
                pose.position.x = transform_stamped.transform.translation.x;
                pose.position.y = transform_stamped.transform.translation.y;
                pose.position.z = transform_stamped.transform.translation.z;
                pose.orientation = transform_stamped.transform.rotation;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(rclcpp::get_logger("voice_command_server_node"), "Could not transform %s to %s: %s", target_frame.c_str(), base_frame.c_str(), ex.what());
            }
            return pose;
        }

        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const xarm_msgs::action::VoiceCommand::Goal> goal)
        {
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "Received goal request with code %d", goal->task_code);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "Received request to cancel goal");
            move_group->stop();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void accepted_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&VoiceCommandServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<xarm_msgs::action::VoiceCommand>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "Executing goal");
            auto result = std::make_shared<xarm_msgs::action::VoiceCommand::Result>();

            // MoveIt 2 Interface
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), ARM_GROUP);
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), GRIPPER_GROUP);

            arm_move_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            arm_move_group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

            if (goal_handle->get_goal()->task_code == 0)
            {
                // Activate robot
                plan_and_execute_to_joint_space_goal(arm_move_group, home_joint_values);
                plan_and_execute_to_joint_space_goal(gripper_move_group, close_gripper);
            }
            else if (goal_handle->get_goal()->task_code == 1)
            {
                // Move to home pose
                plan_and_execute_to_joint_space_goal(arm_move_group, home_joint_values);
                plan_and_execute_to_joint_space_goal(gripper_move_group, close_gripper);
            }
            else if (goal_handle->get_goal()->task_code == 2)
            {
                // Move to pick pose
                plan_and_execute_to_joint_space_goal(arm_move_group, pregrasp_joint_values);
                // pick object
                plan_and_execute_to_joint_space_goal(gripper_move_group, open_gripper);
                plan_and_execute_cartesian_path(arm_move_group, get_frame_pose("link_tcp", "world", rclcpp::Time(0)), iterations, {0.0, 0.0, -0.1});
                plan_and_execute_to_joint_space_goal(gripper_move_group, close_gripper);
                rclcpp::sleep_for(1s);
                plan_and_execute_cartesian_path(arm_move_group, get_frame_pose("link_tcp", "world", rclcpp::Time(0)), iterations, {0.0, 0.0, 0.1});

                // Move to place pose
                plan_and_execute_to_joint_space_goal(arm_move_group, preplace_joint_values);

                // place object
                plan_and_execute_cartesian_path(arm_move_group, get_frame_pose("link_tcp", "world", rclcpp::Time(0)), iterations, {0.0, 0.0, -0.1});
                plan_and_execute_to_joint_space_goal(gripper_move_group, open_gripper);
                rclcpp::sleep_for(1s);
                plan_and_execute_cartesian_path(arm_move_group, get_frame_pose("link_tcp", "world", rclcpp::Time(0)), iterations, {0.0, 0.0, 0.1});
            }
            else if (goal_handle->get_goal()->task_code == 4)
            { // Open gripper
                plan_and_execute_to_joint_space_goal(gripper_move_group, open_gripper);
            }
            else if (goal_handle->get_goal()->task_code == 5)
            {
                // Close gripper
                plan_and_execute_to_joint_space_goal(gripper_move_group, close_gripper);
            }
            else if (goal_handle->get_goal()->task_code == 3)
            {
                
            }
            else
            {
                // error
                RCLCPP_WARN(rclcpp::get_logger("voice_command_server_node"), "Command unknown!! Please try another command");
                return;
            }

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(rclcpp::get_logger("voice_command_server_node"), "Goal succeeded");
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Server<xarm_msgs::action::VoiceCommand>::SharedPtr server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // arm planning group name
        std::string ARM_GROUP = "xarm7";
        // gripper planning group name
        std::string GRIPPER_GROUP = "xarm_gripper";

        const double max_velocity_scaling_factor = 0.07;    // [move_group_interface] default is 0.1
        const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

        /* Joint Space targets */
        std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};
        // std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> wg_home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796};
        std::vector<double> pregrasp_joint_values = {0.0, 0.0384, -0.7138, 1.1502, -0.01745, 1.0961, -0.7243};
        std::vector<double> preplace_joint_values = {0.0, 0.039, 0.7854, 1.1502, -0.0401, 1.0943, 0.7854};
        std::vector<double> open_gripper = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> close_gripper = {0.45, 0.45, 0.45, 0.45, 0.45, 0.45};

        // Number of iterations for cartesian path planning
        int iterations = 1;

        geometry_msgs::msg::Pose current_pose;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(voice_command::VoiceCommandServer)
