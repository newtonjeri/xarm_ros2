/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SM_HPP_
#define XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SM_HPP_

#include <memory>
#include <string>
#include <map>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "xarm_msgs/msg/robot_mode.hpp"
#include "xarm_msgs/msg/robot_state_and_target_pose.hpp"
#include "xarm_msgs/msg/stop_command.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_include.hpp"

namespace simple_state_machine
{

    enum class CommandSource
    {
        INTERNAL,   // From within state machine
        STATE_TOPIC // From /xarm7_state_topic
    };

    enum STATES
    {
        IDLE = 0,
        MOVING = 1,
        PICKING = 2,
        PLACING = 3,
        MANUAL_MODE = 4,
        FINAL = 5,
        ERROR = 6
    };

    class PickAndPlaceStateMachine : public rclcpp::Node
    {
    public:
        explicit PickAndPlaceStateMachine(const rclcpp::NodeOptions &options);

        ~PickAndPlaceStateMachine();

    private:
        enum class CommandSource
        {
            INTERNAL,
            STATE_TOPIC
        };
        CommandSource command_source = CommandSource::INTERNAL;

        // Constants
        const std::string ARM_GROUP_NAME = "xarm7";
        const std::string GRIPPER_GROUP_NAME = "xarm_gripper";
        const STATES START_STATE = IDLE;

        // ROS interfaces
        rclcpp::Publisher<xarm_msgs::msg::RobotMode>::SharedPtr mode_publisher;
        rclcpp::Subscription<xarm_msgs::msg::RobotStateAndTargetPose>::SharedPtr state_subscriber;
        rclcpp::Subscription<xarm_msgs::msg::StopCommand>::SharedPtr stop_command_subscriber_;
        rclcpp::TimerBase::SharedPtr execution_timer;
        rclcpp::TimerBase::SharedPtr initialization_timer;

        // MoveIt interfaces
        std::shared_ptr<moveitinclude::MoveitIncludeNode> xarm7_object;
        std::shared_ptr<moveitinclude::MoveitIncludeNode> xarm_gripper_object;

        // State machine variables
        STATES current_state;
        STATES previous_state;
        rclcpp::Time state_start_time;
        std::chrono::seconds state_timeout;

        // Operation variables
        geometry_msgs::msg::Pose target_pose_1;
        geometry_msgs::msg::Pose target_pose_2;
        geometry_msgs::msg::Pose previous_pose;
        std::string gripper_state;
        uint8_t current_command;

        //
        rclcpp::CallbackGroup::SharedPtr state_callback_group_;
        rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::thread executor_thread_;

        // State machine core functions
        void init();
        void executeStateMachine();
        bool isValidTransition(STATES next_state);
        void enterState(STATES new_state);
        void exitState(STATES old_state);
        void handleCompletion();
        void handleError();

        bool isPickPlacePose(const geometry_msgs::msg::Pose &pose);
        bool isFinalPose(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Pose &previous_pose);

        // State transition logic
        void stateTransitionLogic();

        // State operations
        void idle(uint8_t next_state = MOVING);
        void moving(geometry_msgs::msg::Pose target_pose_1, geometry_msgs::msg::Pose target_pose_2);
        void moving(geometry_msgs::msg::Pose target_pose);
        void picking(geometry_msgs::msg::Pose target_pose_1, geometry_msgs::msg::Pose target_pose_2);
        void placing(geometry_msgs::msg::Pose target_pose);
        void manual_mode(uint8_t next_state = FINAL);

        // Helper functions
        std::string getStateName(STATES state);
        void stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg);
        void stopCallback(const xarm_msgs::msg::StopCommand::SharedPtr msg);

        // Timeout values for each state (in milliseconds)
        const std::map<STATES, int> state_timeouts = {
            {IDLE, 5000},
            {MOVING, 10000},
            {PICKING, 8000},
            {PLACING, 8000},
            {MANUAL_MODE, 30000},
            {FINAL, 1000},
            {ERROR, 5000}};
    };

} // namespace simple_state_machine

#endif // XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SM_HPP_