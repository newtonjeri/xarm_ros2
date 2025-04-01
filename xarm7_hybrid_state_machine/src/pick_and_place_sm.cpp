/* Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_hybrid_state_machine/pick_and_place_sm.hpp"

using namespace std::chrono_literals;

namespace simple_state_machine
{

    PickAndPlaceStateMachine::PickAndPlaceStateMachine(const rclcpp::NodeOptions &options)
        : Node("pick_and_place_sm_node")
    {

        // Create a publisher for the robot mode
        mode_publisher = this->create_publisher<xarm_msgs::msg::RobotMode>("/robot_mode", 10);

        // Create a subscriber for the robot mode
        state_subscriber = this->create_subscription<xarm_msgs::msg::RobotStateAndTargetPose>("/xarm7_state_topic", 10,
                                                                                              std::bind(&PickAndPlaceStateMachine::stateCallback, this, std::placeholders::_1));
        // Initialize MoveIt interface in a separate method using a timer
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PickAndPlaceStateMachine::init, this));
    }

    void PickAndPlaceStateMachine::init()
    {

        auto node_ptr = shared_from_this();

        xarm7_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, ARM_GROUP_NAME);
        xarm_gripper_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, GRIPPER_GROUP_NAME);

        // Initialize state variables
        is_idle = true;
        is_moving = false;
        is_picking = false;
        is_placing = false;
        is_manual_mode = false;
        current_state = IDLE;
        previous_pose = []
        {
            geometry_msgs::msg::Pose msg;
            msg.position.x = 0.4912;
            msg.position.y = 0.0661;
            msg.position.z = 0.0650;
            msg.orientation.x = 1.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
            msg.orientation.w = 0.0;
            return msg;
        }();
        gripper_state = "FREE"; // GRASPING when object is attached, FREE is used when object is detached

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");
        timer_->cancel(); // Stop the timer after initialization
    }


    void PickAndPlaceStateMachine::idle(uint8_t next_state = MOVING)
    {
        is_idle = true;
        if (next_state == 4)
        {
            current_state = (STATES)next_state;
        }
        else
        {
            // Publish robot mode to 0 for idle mode
            xarm_msgs::msg::RobotMode mode_msg;
            mode_msg.data = 0;
            mode_publisher->publish(mode_msg);
            current_state = MOVING;
        }

        is_idle = false;
    }

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose, uint8_t next_state)
    {
        is_moving = true;

        // Attempt to move to the target pose
        bool success = xarm7_object->planToTargetPose(target_pose, false);

        if (!success)
        {
            // Transition to ERROR state if planning or execution fails
            current_state = ERROR;
            RCLCPP_ERROR(this->get_logger(), "Failed to move to target pose. Transitioning to ERROR state.");
            return;
        }

        // Transition to the next state based on gripper state
        if (target_pose.orientation.z == 0.0 && target_pose.orientation.x == 1.0)
        {

            if (gripper_state == "FREE")
            {
                current_state = PICKING;
            }
            else if (gripper_state == "GRASPING")
            {
                current_state = PLACING;
            }
        }
        else
        {
            // Implement Logic to move to MOVING, MANUAL_MODE, IDLE and FINAL state
            if (previous_pose == target_pose)
            {
                current_state = FINAL;
            }
            else
            {
                current_state = MOVING;
            }
        }
        previous_pose = target_pose;
        is_moving = false;
    }

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose)
    {
        // Attempt to move to the target pose
        bool success = xarm7_object->planToTargetPose(target_pose, false);

        if (!success)
        {
            // Transition to ERROR state if planning or execution fails
            current_state = ERROR;
            RCLCPP_ERROR(this->get_logger(), "Failed to move to target pose. Transitioning to ERROR state.");
            return;
        }
    }

    void PickAndPlaceStateMachine::picking(geometry_msgs::msg::Pose target_pose, uint8_t next_state = MOVING)
    {
        is_picking = true;

        // Open gripper
        xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        // Move down to pick the object
        target_pose.position.z += -0.05;
        moving(target_pose);

        // Close gripper
        xarm_gripper_object->gripperOpenAndClose({0.84, 0.84, 0.84, 0.84, 0.84, 0.84});
        gripper_state = "GRASPING";

        // Move back up
        target_pose.position.z += 0.05;
        moving(target_pose);

        // Transition to the next state
        current_state = (STATES)next_state;
        is_picking = false;
    }

    void PickAndPlaceStateMachine::placing(geometry_msgs::msg::Pose target_pose, uint8_t next_state = MOVING)
    {
        is_placing = true;

        // Move down to place the object
        target_pose.position.z += -0.05;

        moving(target_pose);

        // Open gripper
        xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        gripper_state = "FREE";

        // Move back up
        target_pose.position.z += 0.05;
        moving(target_pose);

        // Transition to the next state
        current_state = (STATES)next_state;
        is_placing = false;
    }
    void PickAndPlaceStateMachine::manual_mode(uint8_t next_state = 5)
    {
        is_manual_mode = true;
        // Publish mode message
        xarm_msgs::msg::RobotMode mode_msg;
        mode_msg.data = 2;
        mode_publisher->publish(mode_msg);

        current_state = (STATES)next_state;
        RCLCPP_INFO(rclcpp::get_logger("State Machine"), "Next state: %s", getStateName((STATES)next_state).c_str());
    }

    std::string PickAndPlaceStateMachine::getStateName(STATES state)
    {
        switch (state)
        {
        case IDLE:
            return "IDLE";
        case MOVING:
            return "MOVING";
        case PICKING:
            return "PICKING";
        case PLACING:
            return "PLACING";
        case MANUAL_MODE:
            return "MANUAL_MODE";
        case FINAL:
            return "FINAL";
        case ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
        }
    }    void PickAndPlaceStateMachine::stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg)
    {

        uint8_t next_state = msg->robot_next_state;
        geometry_msgs::msg::Pose pose_ = msg->target_pose_1;
        geometry_msgs::msg::Pose place_pose = msg->target_pose_2;

        stateTransitionLogic(START_STATE, pose_, place_pose);
        current_state = (STATES)next_state;

        while (true)
        {
            stateTransitionLogic(current_state, pose_, place_pose);

            if (current_state == FINAL)
            {
                RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: FINAL - Motion completed!");
                is_idle = false;
                is_moving = false;
                is_picking = false;
                is_placing = false;
                gripper_state = "FREE";
                break;
            }
            else if (current_state == ERROR)
            {
                RCLCPP_ERROR(rclcpp::get_logger("State Machine"), "State: ERROR - Motion failure detected!");
                break;
            }
        }
    }

    void PickAndPlaceStateMachine::stateTransitionLogic(uint8_t next_state, geometry_msgs::msg::Pose pose_, geometry_msgs::msg::Pose place_pose)
    {
        switch (current_state)
        {
        case IDLE:
            RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: IDLE");
            idle(next_state);
            break;

        case MOVING:
            RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: MOVING");
            moving(pose_, next_state);
            break;

        case PICKING:
            RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: PICKING");
            picking(pose_);
            break;

        case PLACING:
            RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: PLACING");
            placing(place_pose);
            break;

        case MANUAL_MODE:
            RCLCPP_INFO(rclcpp::get_logger("State Machine"), "State: MANUAL_MODE");
            manual_mode();
            break;

        default:
            break;
        }
    }

} // namespace simple_state_machine

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_state_machine::PickAndPlaceStateMachine)
