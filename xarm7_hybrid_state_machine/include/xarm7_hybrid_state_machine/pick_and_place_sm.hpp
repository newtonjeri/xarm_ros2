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
#include <mutex>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "xarm_msgs/msg/robot_mode.hpp"
#include "xarm_msgs/msg/robot_state_and_target_pose.hpp"
#include "xarm_msgs/msg/stop_command.hpp"
#include "xarm_msgs/msg/robot_msg.hpp"
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
        PICKING = 2,      // Derived from MOVING state
        PLACING = 3,      // Derived from MOVING state
        FINAL = 4,        // Renumbered from 5
        ERROR = 5         // Renumbered from 6
    };

    // Xarm robot states (from RobotMsg)
    enum class XarmState : int16_t 
    {
        RUNNING = 1,        // executing motion command
        SLEEPING = 2,       // not in execution, but ready to move
        PAUSED = 3,         // paused in the middle of unfinished motion
        STOPPED = 4,        // not ready for any motion commands
        CONFIG_CHANGED = 5  // system configuration or mode changed, not ready for motion commands
    };

    // Xarm robot modes (from RobotMsg)
    enum class XarmMode : int16_t
    {
        POSITION = 0,        // position control by xarm controller box, execute api standard commands
        SERVOJ = 1,          // Immediate execution towards received joint space target, like a step response
        TEACHING_JOINT = 2   // Gravity compensated mode, easy for teaching
    };

    class PickAndPlaceStateMachine : public rclcpp::Node
    {
    public:
        PickAndPlaceStateMachine();
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
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher;
        rclcpp::Subscription<xarm_msgs::msg::RobotStateAndTargetPose>::SharedPtr state_subscriber;
        rclcpp::Subscription<xarm_msgs::msg::StopCommand>::SharedPtr stop_command_subscriber_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_subscriber_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;
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

        // Xarm robot state tracking
        XarmState current_xarm_state;
        XarmMode current_xarm_mode;
        int16_t xarm_error_code;
        int16_t xarm_warning_code;
        bool robot_state_received;

        // Operation variables
        geometry_msgs::msg::Pose target_pose_1;
        geometry_msgs::msg::Pose target_pose_2;
        geometry_msgs::msg::Pose previous_pose;
        std::string gripper_state;
        uint8_t current_command;

        // Sequence planning and execution
        std::vector<STATES> planned_sequence_;
        size_t sequence_index_;
        bool sequence_active_;
        
        // Asynchronous operation tracking
        bool movement_in_progress_;
        bool gripper_operation_in_progress_;
        std::chrono::time_point<std::chrono::steady_clock> movement_start_time_;
        std::chrono::time_point<std::chrono::steady_clock> gripper_start_time_;

        //
        rclcpp::CallbackGroup::SharedPtr state_callback_group_;
        rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
        rclcpp::CallbackGroup::SharedPtr robot_state_callback_group_;

        // Thread safety
        mutable std::mutex state_mutex_;  // Changed from recursive_mutex
        mutable std::mutex pose_mutex_;
        mutable std::mutex sequence_mutex_;

        // Initialization
        void initializeNode();

        // State machine core functions
        void init();
        void executeStateMachine();
        bool isValidTransition(STATES next_state);
        void enterState(STATES new_state);
        void enterStateUnsafe(STATES new_state); // Helper for already-locked contexts
        void exitState(STATES old_state);
        void handleCompletion();
        void handleError();

        // Sequence planning and execution
        bool processExternalCommand(int8_t robot_next_state, 
                                   const geometry_msgs::msg::Pose& pose1,
                                   const geometry_msgs::msg::Pose& pose2);
        std::vector<STATES> planSequence(int8_t robot_next_state,
                                        const geometry_msgs::msg::Pose& pose1,
                                        const geometry_msgs::msg::Pose& pose2);
        void executeSequence();
        bool advanceSequence();
        void resetSequence();
        
        // Asynchronous operation management
        bool startMovement(const geometry_msgs::msg::Pose& target_pose);
        bool checkMovementComplete();
        bool startGripperOperation(bool open);
        bool checkGripperComplete();
        void cancelAllOperations();

        bool isPickPlacePose(const geometry_msgs::msg::Pose &pose);
        bool isFinalPose(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Pose &previous_pose);
        bool posesEqual(const geometry_msgs::msg::Pose& pose1, 
                       const geometry_msgs::msg::Pose& pose2, 
                       double position_tolerance = 0.005,
                       double orientation_tolerance = 0.001);

        // State transition logic
        void stateTransitionLogic();

        // State operations (simplified - no longer do transitions)
        void executeIdleState();
        void executeMovingState();
        void executePickingState(); 
        void executePlacingState();
        
        // Pure operations (no state management)
        bool performMovement(const geometry_msgs::msg::Pose& target_pose);
        bool performPickingSequence(const geometry_msgs::msg::Pose& pick_pose);
        bool performPlacingSequence(const geometry_msgs::msg::Pose& place_pose);

        // Helper functions
        std::string getStateName(STATES state);
        std::string getXarmStateName(XarmState state);
        std::string getXarmModeName(XarmMode mode);
        void stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg);
        void stopCallback(const xarm_msgs::msg::StopCommand::SharedPtr msg);
        void robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg);
        void recoveryService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        
        // Robot state validation functions
        bool isRobotReady();
        bool isRobotMoving();
        bool hasRobotError();
        bool shouldTransitionBasedOnRobotState(STATES intended_state);
        
        // Modal state validation functions
        bool isStateValidForCurrentMode(STATES state);
        std::set<STATES> getValidStatesForMode(XarmMode mode);
        bool isModeSwitchRequired(STATES intended_state);

        // Timeout values for each state (in milliseconds)
        const std::map<STATES, int> state_timeouts = {
            {IDLE, 5000},
            {MOVING, 10000},
            {PICKING, 8000},
            {PLACING, 8000},
            {FINAL, 1000},
            {ERROR, 5000}};
    };

} // namespace simple_state_machine

#endif // XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SM_HPP_