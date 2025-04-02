/* Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_hybrid_state_machine/pick_and_place_sm.hpp"

using namespace std::chrono_literals;

void exit_sig_handler(int signum)
{
    rclcpp::shutdown();
    fprintf(stderr, "[pick_and_place_sm_node] Ctrl-C caught, shutting down...\n");
    exit(0);  // Use 0 for clean exit
}

namespace simple_state_machine
{

    PickAndPlaceStateMachine::PickAndPlaceStateMachine(const rclcpp::NodeOptions &options)
        : Node("pick_and_place_sm_node", options),
          current_state(IDLE),
          previous_state(IDLE),
          state_timeout(30s),
          gripper_state("FREE")
    {
         // Set up signal handler
        signal(SIGINT, exit_sig_handler);
        // Initialize publishers and subscribers
        mode_publisher = this->create_publisher<xarm_msgs::msg::RobotMode>("/robot_mode", 10);
        state_subscriber = this->create_subscription<xarm_msgs::msg::RobotStateAndTargetPose>(
            "/xarm7_state_topic", 10,
            std::bind(&PickAndPlaceStateMachine::stateCallback, this, std::placeholders::_1));

        // Initialize default pose
        previous_pose.position.x = 0.4912;
        previous_pose.position.y = 0.0661;
        previous_pose.position.z = 0.0650;
        previous_pose.orientation.x = 1.0;
        previous_pose.orientation.y = 0.0;
        previous_pose.orientation.z = 0.0;
        previous_pose.orientation.w = 0.0;

        // Setup initialization timer
        initialization_timer = this->create_wall_timer(
            500ms, std::bind(&PickAndPlaceStateMachine::init, this));
    }

    void PickAndPlaceStateMachine::init()
    {
        auto node_ptr = shared_from_this();
        xarm7_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, ARM_GROUP_NAME);
        xarm_gripper_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, GRIPPER_GROUP_NAME);

        // Setup state machine execution timer (100ms cycle)
        execution_timer = this->create_wall_timer(
            100ms, std::bind(&PickAndPlaceStateMachine::executeStateMachine, this));

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");
        initialization_timer->cancel();
        enterState(IDLE);
    }

    void PickAndPlaceStateMachine::executeStateMachine()
    {
        // Check for state timeouts
        // auto elapsed = this->now() - state_start_time;
        // if (elapsed.seconds() > state_timeouts.at(current_state) / 1000.0)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Timeout in state %s", getStateName(current_state).c_str());
        //     enterState(ERROR);
        // }

        // Execute current state logic
        stateTransitionLogic();

        // Handle completion or error conditions
        if (current_state == FINAL)
        {
            handleCompletion();
        }
        else if (current_state == ERROR)
        {
            handleError();
        }
    }

    bool PickAndPlaceStateMachine::isValidTransition(STATES next_state)
    {
        // Updated transition rules including IDLE->FINAL
        static const std::map<STATES, std::set<STATES>> valid_transitions = {
            {IDLE, {IDLE, MOVING, MANUAL_MODE, FINAL, ERROR}}, // Added FINAL
            {MOVING, {PICKING, PLACING, IDLE, MANUAL_MODE, FINAL, ERROR}},
            {PICKING, {MOVING, ERROR}},
            {PLACING, {MOVING, FINAL, ERROR}},
            {MANUAL_MODE, {IDLE, FINAL, ERROR}},
            {FINAL, {IDLE, ERROR}},
            {ERROR, {IDLE}}};

        // Special case: Allow forced ERROR transition from any state
        if (next_state == ERROR)
        {
            RCLCPP_DEBUG(this->get_logger(), "Allowing emergency transition to ERROR state");
            return true;
        }

        // Find current state in transition map
        auto current_state_transitions = valid_transitions.find(current_state);
        if (current_state_transitions == valid_transitions.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Current state %d not found in transition map!",
                         current_state);
            return false;
        }

        // Check if requested transition exists
        bool valid = current_state_transitions->second.count(next_state) > 0;

        if (!valid)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid transition: %s → %s",
                        getStateName(current_state).c_str(),
                        getStateName(next_state).c_str());

            // Log available transitions for debugging
            std::string available;
            for (const auto &s : current_state_transitions->second)
            {
                available += getStateName(s) + " ";
            }
            RCLCPP_DEBUG(this->get_logger(),
                         "Available from %s: %s",
                         getStateName(current_state).c_str(),
                         available.c_str());
        }

        return valid;
    }

    void PickAndPlaceStateMachine::enterState(STATES new_state)
    {
        RCLCPP_INFO(this->get_logger(), "Transitioning from %s to %s",
                    getStateName(current_state).c_str(),
                    getStateName(new_state).c_str());

        exitState(current_state);
        previous_state = current_state;
        current_state = new_state;

        // State-specific entry actions
        xarm_msgs::msg::RobotMode mode_msg;

        switch (current_state)
        {
            case ERROR:
                handleError();
                break;

            default:
                break;
        }
    }

    void PickAndPlaceStateMachine::exitState(STATES old_state)
    {
        RCLCPP_DEBUG(this->get_logger(), "Exiting state: %s", getStateName(old_state).c_str());
        // State-specific cleanup can be added here
    }

    void PickAndPlaceStateMachine::stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg)
    {
        command_source = CommandSource::STATE_TOPIC;
        current_command = msg->robot_next_state;
        target_pose_1 = msg->target_pose_1;
        target_pose_2 = msg->target_pose_2;
        RCLCPP_DEBUG(this->get_logger(), "Received new command: %d", current_command);
    }

    void PickAndPlaceStateMachine::stateTransitionLogic()
    {
        switch (current_state)
        {
        case IDLE:
            // Delegate all idle state transitions to the idle() function
            idle(current_command);

            // RCLCPP_DEBUG_ONCE(this->get_logger(), 
            //                  "Waiting for commands on /xarm7_state_topic");
            break;

        case MOVING:
            moving(target_pose_1, current_command);
            break;

        case PICKING:
            picking(target_pose_1, target_pose_2, current_command);
            break;

        case PLACING:
            placing(target_pose_2, current_command);
            break;

        case MANUAL_MODE:
            manual_mode(current_command);
            break;

        case FINAL:
            handleCompletion();
            break;

        case ERROR:
            handleError();
            break;
        }
    }

    void PickAndPlaceStateMachine::handleCompletion()
    {
        RCLCPP_INFO(this->get_logger(), "Operation completed successfully");
        gripper_state = "FREE";
        previous_state = IDLE;
        current_state = IDLE;
        current_command = 0;
        enterState(IDLE);
    }

    void PickAndPlaceStateMachine::handleError()
    {
        RCLCPP_ERROR(this->get_logger(), "Error encountered in state %s",
                     getStateName(current_state).c_str());

        // Stop all movements
        xarm7_object->move_group->stop();
        xarm_gripper_object->move_group->stop();

        // Transition back to IDLE after timeout
        if ((this->now() - state_start_time).seconds() > 5.0)
        {
            enterState(IDLE);
        }
    }

    void PickAndPlaceStateMachine::idle(uint8_t next_state)
    {
        // 1. State Entry Actions
        RCLCPP_INFO(this->get_logger(), "IDLE state entered");

        if(previous_state != IDLE){
            // Publish idle mode (0)
            xarm_msgs::msg::RobotMode mode_msg;
            mode_msg.data = 0; // IDLE mode
            mode_publisher->publish(mode_msg);
        }

        // 2. Validate Requested Transition
        STATES requested_state = static_cast<STATES>(next_state);

        if (!isValidTransition(requested_state))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejected invalid transition from IDLE to %s",
                        getStateName(requested_state).c_str());
            return;
        }

        // 3. Handle Special Cases
        if (requested_state == FINAL)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Direct transition from IDLE to FINAL initiated");
            // Add any special finalization logic here
            enterState(FINAL);
            return;
        }

        // 4. Normal Transition Handling
        switch (requested_state)
        {
        case IDLE:
            // RCLCPP_INFO(this->get_logger(), "Already in IDLE state");
            break;
        case MOVING:
            RCLCPP_INFO(this->get_logger(), "Starting normal operation sequence");
            enterState(MOVING);
            break;

        case MANUAL_MODE:
            RCLCPP_INFO(this->get_logger(), "Entering manual control mode");
            enterState(MANUAL_MODE);
            break;

        case ERROR:
            RCLCPP_ERROR(this->get_logger(), "Error condition detected while idle");
            enterState(ERROR);
            break;

        default:
            RCLCPP_WARN(this->get_logger(),
                        "Unhandled transition from IDLE to %s",
                        getStateName(requested_state).c_str());
            break;
        }
    }

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose, uint8_t next_state)
    {
        // 1. Entry Actions
        RCLCPP_INFO(this->get_logger(),
                    "MOVING to pose (x: %.3f, y: %.3f, z: %.3f)",
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z);

        // 2. Execute Movement
        bool success = xarm7_object->planToTargetPose(target_pose, false);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Movement failed to (x: %.3f, y: %.3f)",
                         target_pose.position.x,
                         target_pose.position.y);
            enterState(ERROR);
            return;
        }

        // 3. Determine Next State
        STATES next = MOVING; // Default to continue moving

        // Check for special pose conditions
        if (isPickPlacePose(target_pose))
        {
            // Transition based on gripper state
            next = (gripper_state == "FREE") ? PICKING : PLACING;
        }
        // Check for manual mode override
        else if (next_state == MANUAL_MODE && isValidTransition(MANUAL_MODE))
        {
            next = MANUAL_MODE;
        }
        // Check for return to idle
        else if (next_state == IDLE && isValidTransition(IDLE))
        {
            next = IDLE;
        }
        // Check if reached final pose
        else if (isFinalPose(target_pose, previous_pose))
        {
            next = FINAL;
        }

        enterState(next);        // Store and Transition
        previous_pose = target_pose;
    }

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose)
    {
        // Simplified version without state transition logic
        RCLCPP_DEBUG(this->get_logger(), "Executing movement only");

        bool success = xarm7_object->planToTargetPose(target_pose, false);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Movement command failed");
            enterState(ERROR);
            return;
        }
    }

    // Helper Functions
    bool PickAndPlaceStateMachine::isPickPlacePose(const geometry_msgs::msg::Pose &pose)
    {
        // Check for characteristic pick/place pose orientation
        const double tolerance = 0.001;
        return (std::abs(pose.orientation.x - 1.0) < tolerance &&
                std::abs(pose.orientation.z) < tolerance);
    }

    bool PickAndPlaceStateMachine::isFinalPose(const geometry_msgs::msg::Pose &pose,
                                               const geometry_msgs::msg::Pose &previous_pose)
    {
        // Check if we've reached the final position
        const double position_tolerance = 0.005; // 5mm
        const double orientation_tolerance = 0.01;

        return (std::abs(pose.position.x - previous_pose.position.x)) < position_tolerance &&
               (std::abs(pose.position.y - previous_pose.position.y)) < position_tolerance &&
               (std::abs(pose.position.z - previous_pose.position.z)) < position_tolerance &&
               (std::abs(pose.orientation.x - previous_pose.orientation.x)) < orientation_tolerance &&
               (std::abs(pose.orientation.y - previous_pose.orientation.y)) < orientation_tolerance &&
               (std::abs(pose.orientation.z - previous_pose.orientation.z)) < orientation_tolerance &&
               (std::abs(pose.orientation.w - previous_pose.orientation.w)) < orientation_tolerance;
    }

    void PickAndPlaceStateMachine::picking(geometry_msgs::msg::Pose target_pose_1, geometry_msgs::msg::Pose target_pose_2, uint8_t next_state)
    {
        // 1. Entry Actions
        RCLCPP_INFO(this->get_logger(),
                    "Starting PICK operation at (x: %.3f, y: %.3f)",
                    target_pose_1.position.x,
                    target_pose_1.position.y);

        // 2. Open Gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}))
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper open failed");
            enterState(ERROR);
            return;
        }

        // 3. Approach Object (using simplified move)
        geometry_msgs::msg::Pose approach_pose = target_pose_1;
        approach_pose.position.z -= 0.05; // 5cm approach
        moving(approach_pose);            // Using simple movement without state transition logic

        // Check if movement failed
        if (current_state == ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Approach movement failed");
            return;
        }
        // 4. Close Gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.84, 0.84, 0.84, 0.84, 0.84, 0.84}))
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper close failed");
            enterState(ERROR);
            return;
        }
        gripper_state = "GRASPING";

        // 5. Retreat (using simplified move)
        moving(target_pose_1); // Return to original pose    
        // Check if movement failed
        if (current_state == ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Approach movement failed");
            return;
        }
        

        // 6. Transition Logic
        STATES next = MOVING; // Default transition
        if (isValidTransition(static_cast<STATES>(next_state)))
        {
            next = static_cast<STATES>(next_state);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Requested transition to %s not allowed, defaulting to MOVING",
                        getStateName(static_cast<STATES>(next_state)).c_str());
        }

        if(isFinalPose(target_pose_1, target_pose_2))
        {
            next = FINAL; // Auto-transition if at final pose
        }

        enterState(next);
    }

    void PickAndPlaceStateMachine::placing(geometry_msgs::msg::Pose target_pose, uint8_t next_state)
    {
        // 1. Entry Actions
        RCLCPP_INFO(this->get_logger(),
                    "Starting PLACE operation at (x: %.3f, y: %.3f)",
                    target_pose.position.x,
                    target_pose.position.y);

        // 2. Verify preconditions
        if (gripper_state != "GRASPING")
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Cannot place - gripper not holding object");
            enterState(ERROR);
            return;
        } 
        
        // Move to target pose
        moving(target_pose); // Return to original pose
            
        // Check if movement failed
        if (current_state == ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Approach movement failed");
            return;
        }

        // 3. Approach Place Position (using simplified move)
        geometry_msgs::msg::Pose approach_pose = target_pose;
        approach_pose.position.z -= 0.05; // 5cm approach

        moving(approach_pose);            // Basic movement only
        // Check if movement failed
        if (current_state == ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Approach movement failed");
            return;
        }

        // 4. Open Gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}))
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper open failed");
            enterState(ERROR);
            return;
        }
        gripper_state = "FREE";

        // 5. Retreat (using simplified move)
        moving(target_pose); // Return to original pose
            
        // Check if movement failed
        if (current_state == ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Approach movement failed");
            return;
        }

        // 6. Transition Logic
        STATES next = MOVING; // Default transition
        if (isFinalPose(target_pose, target_pose))
        {
            next = FINAL; // Auto-transition if at final pose
        }
        else if (isValidTransition(static_cast<STATES>(next_state)))
        {
            next = static_cast<STATES>(next_state);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Requested transition to %s not allowed, using %s",
                        getStateName(static_cast<STATES>(next_state)).c_str(),
                        getStateName(next).c_str());
        }

        enterState(next);
    }

    void PickAndPlaceStateMachine::manual_mode(uint8_t next_state)
    {
        // 1. Entry Actions (only execute on first entry)
        if (previous_state != MANUAL_MODE)
        {
            RCLCPP_INFO(this->get_logger(), "Entering MANUAL_MODE");

            // Publish manual mode command (2)
            xarm_msgs::msg::RobotMode mode_msg;
            mode_msg.data = 2; // Manual mode
            mode_publisher->publish(mode_msg);
        }

        // 2. Only process transitions if they come from the state topic
        // (Ignore the next_state parameter unless it's a fresh command)
        if (command_source == CommandSource::STATE_TOPIC)
        {
            STATES requested_state = static_cast<STATES>(next_state);
            if (isValidTransition(requested_state))
            {
                enterState(requested_state);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Invalid transition from MANUAL_MODE to %s",
                            getStateName(requested_state).c_str());
            }
        }
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
    }

} // namespace simple_state_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_state_machine::PickAndPlaceStateMachine)