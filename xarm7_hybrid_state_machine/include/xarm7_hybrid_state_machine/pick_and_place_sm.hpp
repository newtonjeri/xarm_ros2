/* Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef PICK_AND_PLACE_STATE_MACHINE_HPP
#define PICK_AND_PLACE_STATE_MACHINE_HPP

#include <cstdint>

#include "moveit_include.hpp"
#include "xarm_msgs/msg/robot_mode.hpp"
#include "xarm_msgs/msg/robot_state_and_target_pose.hpp"



namespace simple_state_machine{

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

    STATES current_state;

    class PickAndPlaceStateMachine: public rclcpp::Node{
        public:
            PickAndPlaceStateMachine(const rclcpp::NodeOptions& options);
            void init();

            void idle(uint8_t next_state);

            void moving(geometry_msgs::msg::Pose target_pose);

            void moving(geometry_msgs::msg::Pose target_pose, uint8_t next_state);

            void picking(geometry_msgs::msg::Pose target_pose, uint8_t next_state);

            void placing(geometry_msgs::msg::Pose target_pose, uint8_t next_state);

            void manual_mode(uint8_t next_state);

            void stateTransitionLogic(uint8_t next_state, geometry_msgs::msg::Pose pose_, geometry_msgs::msg::Pose place_pose);

            void stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg);

            std::string getStateName(STATES state);

        private:
            std::string ARM_GROUP_NAME = "xarm7";
            std::string GRIPPER_GROUP_NAME = "xarm_gripper";
            std::shared_ptr<moveitinclude::MoveitIncludeNode> xarm7_object;
            std::shared_ptr<moveitinclude::MoveitIncludeNode> xarm_gripper_object;

            // Timer for initialization
            rclcpp::TimerBase::SharedPtr timer_;
            // Change mode publisher
            rclcpp::Publisher<xarm_msgs::msg::RobotMode>::SharedPtr mode_publisher;

            // State subscriber
            rclcpp::Subscription<xarm_msgs::msg::RobotStateAndTargetPose>::SharedPtr state_subscriber;

            // Transitions variables
            bool is_idle;
            bool is_moving;
            bool is_picking;
            bool is_placing;
            bool is_manual_mode;
            std::string gripper_state;

            // Variable to store the previous
            geometry_msgs::msg::Pose previous_pose;

            uint8_t START_STATE = 0;
    };
}


#endif // PICK_AND_PLACE_STATE_MACHINE_HPP