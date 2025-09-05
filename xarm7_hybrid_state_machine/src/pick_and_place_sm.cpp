/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_hybrid_state_machine/pick_and_place_sm.hpp"

using namespace std::chrono_literals;

void exit_sig_handler(int signum)
{
    (void)signum; // Suppress unused parameter warning
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
          current_xarm_state(XarmState::SLEEPING),
          current_xarm_mode(XarmMode::POSITION),
          xarm_error_code(0),
          xarm_warning_code(0),
          robot_state_received(false),
          gripper_state("FREE"),
          current_command(static_cast<uint8_t>(IDLE))
    {
        // Set up signal handler
        signal(SIGINT, exit_sig_handler);

        // Create callback groups
        state_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        stop_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        robot_state_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
            
        
        // Initialize publishers and subscribers
        mode_publisher = this->create_publisher<xarm_msgs::msg::RobotMode>("/robot_mode", 10);
        state_publisher = this->create_publisher<std_msgs::msg::UInt8>("/xarm7_state_machine_state", 10);

        auto state_sub_options = rclcpp::SubscriptionOptions();
        state_sub_options.callback_group = state_callback_group_;
        state_subscriber = this->create_subscription<xarm_msgs::msg::RobotStateAndTargetPose>(
            "/xarm7_state_topic", 10,
            std::bind(&PickAndPlaceStateMachine::stateCallback, this, std::placeholders::_1),
            state_sub_options);

        auto stop_command_sub_options = rclcpp::SubscriptionOptions();
        stop_command_sub_options.callback_group = stop_callback_group_;
        stop_command_subscriber_ = this->create_subscription<xarm_msgs::msg::StopCommand>(
            "/xarm7_stop_command", 10,
            std::bind(&PickAndPlaceStateMachine::stopCallback, this, std::placeholders::_1),
            stop_command_sub_options);

        auto robot_state_sub_options = rclcpp::SubscriptionOptions();
        robot_state_sub_options.callback_group = robot_state_callback_group_;
        robot_state_subscriber_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            "/xarm/robot_states", 10,
            std::bind(&PickAndPlaceStateMachine::robotStateCallback, this, std::placeholders::_1),
            robot_state_sub_options);

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

    PickAndPlaceStateMachine::~PickAndPlaceStateMachine()
    {
        if (executor_thread_.joinable()) {
            executor_->cancel();
            executor_thread_.join();
        }
    }

    void PickAndPlaceStateMachine::init()
    {
        auto node_ptr = shared_from_this();
        xarm7_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, ARM_GROUP_NAME);
        xarm_gripper_object = std::make_shared<moveitinclude::MoveitIncludeNode>(node_ptr, GRIPPER_GROUP_NAME);

        // Setup state machine execution timer (100ms cycle)
        execution_timer = this->create_wall_timer(
            100ms, std::bind(&PickAndPlaceStateMachine::executeStateMachine, this));

        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: MoveIt interfaces initialized", 
                   getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
        initialization_timer->cancel();
        enterState(IDLE);
    }

    void PickAndPlaceStateMachine::executeStateMachine()
    {
        // Handle completion or error conditions
        if (current_state == FINAL)
        {
            handleCompletion();
            return;
        }
        else if (current_state == ERROR)
        {
            handleError();
            return;
        }
        // Execute current state logic
        stateTransitionLogic();
    }

    bool PickAndPlaceStateMachine::isValidTransition(STATES next_state)
    {
        static const std::map<STATES, std::set<STATES>> valid_transitions = {
            {IDLE, {IDLE, MOVING, MANUAL_MODE, FINAL, ERROR}},
            {MOVING, {MOVING, PICKING, PLACING, IDLE, MANUAL_MODE, FINAL, ERROR}},
            {PICKING, {IDLE, MOVING, ERROR}},
            {PLACING, {IDLE, MOVING, FINAL, ERROR}},
            {MANUAL_MODE, {MANUAL_MODE, IDLE, FINAL, ERROR}},
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
        if(new_state != previous_state){
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Transitioning from %s to %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getStateName(previous_state).c_str(),
                        getStateName(new_state).c_str());
        }

        previous_state = current_state;
        current_state = new_state;

        // Publish state change for synchronization with other nodes
        std_msgs::msg::UInt8 state_msg;
        state_msg.data = static_cast<uint8_t>(current_state);
        state_publisher->publish(state_msg);

        // State-specific entry actions
        xarm_msgs::msg::RobotMode mode_msg;

        switch (current_state)
        {
            case MANUAL_MODE:
                // Set robot to TEACHING_JOINT mode when entering MANUAL_MODE
                RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Setting robot to TEACHING_JOINT mode", 
                           getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
                mode_msg.data = 2; // TEACHING_JOINT mode
                mode_publisher->publish(mode_msg);
                break;
                
            case ERROR:
                handleError();
                break;

            default:
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s", 
                   getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
    }

    void PickAndPlaceStateMachine::robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
    {
        current_xarm_state = static_cast<XarmState>(msg->state);
        current_xarm_mode = static_cast<XarmMode>(msg->mode);
        xarm_error_code = msg->err;
        xarm_warning_code = msg->warn;
        robot_state_received = true;

        // Log significant state changes
        static XarmState last_state = XarmState::SLEEPING;
        static int16_t last_error = 0;
        
        if (current_xarm_state != last_state) {
            RCLCPP_INFO(this->get_logger(), "Xarm state changed: %s -> %s", 
                        getXarmStateName(last_state).c_str(),
                        getXarmStateName(current_xarm_state).c_str());
            last_state = current_xarm_state;
        }

        if (xarm_error_code != last_error && xarm_error_code != 0) {
            RCLCPP_ERROR(this->get_logger(), "Xarm error detected: %d", xarm_error_code);
            last_error = xarm_error_code;
        }

        RCLCPP_DEBUG(this->get_logger(), "Xarm Status - State: %s, Mode: %s, Error: %d, Warning: %d",
                    getXarmStateName(current_xarm_state).c_str(),
                    getXarmModeName(current_xarm_mode).c_str(),
                    xarm_error_code, xarm_warning_code);
    }

    void PickAndPlaceStateMachine::stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg)
    {
        command_source = CommandSource::STATE_TOPIC;
        current_command = msg->robot_next_state;
        target_pose_1 = msg->target_pose_1;
        target_pose_2 = msg->target_pose_2;
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Received new command: %d (%s)", 
                    getXarmModeName(current_xarm_mode).c_str(),
                    getStateName(current_state).c_str(), 
                    current_command,
                    getStateName(static_cast<STATES>(current_command)).c_str());
    }

    void PickAndPlaceStateMachine::stopCallback(const xarm_msgs::msg::StopCommand::SharedPtr msg)
    {
        bool stop_command = msg->stop_command_state;
        if (stop_command){
            RCLCPP_ERROR(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: RECEIVED STOP COMMAND!!!", 
                        getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
            current_state = ERROR;
            current_command = (uint8_t)STATES::ERROR;

            xarm7_object->move_group->stop();
            xarm_gripper_object->move_group->stop();
        }
        else{
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Normal operation", 
                       getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
        }
    }

    void PickAndPlaceStateMachine::stateTransitionLogic()
    {
        switch (current_state)
        {
        case IDLE:
            // Only process commands from external sources (not just the timer loop)
            if (command_source == CommandSource::STATE_TOPIC) {
                idle(current_command);
                // Reset command source after processing
                command_source = CommandSource::INTERNAL;
            }
            break;

        case MOVING:
            moving(target_pose_1, target_pose_2);
            break;

        case PICKING:
            picking(target_pose_1, target_pose_2);
            break;

        case PLACING:
            placing(target_pose_2);
            break;

        case MANUAL_MODE:
            // Only process transition commands from external sources
            if (command_source == CommandSource::STATE_TOPIC) {
                manual_mode(current_command);
                // Reset command source after processing
                command_source = CommandSource::INTERNAL;
            }
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
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Operation completed successfully", 
                   getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
        gripper_state = "FREE";
        previous_state = IDLE;
        current_state = IDLE;
        current_command = 0;
        enterState(IDLE);
    }

    void PickAndPlaceStateMachine::handleError()
    {
        RCLCPP_ERROR(this->get_logger(), "Error encountered in state %s - Robot state: %s, Error code: %d",
                     getStateName(previous_state).c_str(),
                     getXarmStateName(current_xarm_state).c_str(),
                     xarm_error_code);

        // Stop all movements
        xarm7_object->move_group->stop();
        xarm_gripper_object->move_group->stop();
        
        // Reset internal state
        gripper_state = "FREE";
        previous_state = IDLE;
        current_state = IDLE;
        current_command = 0;
        
        // Try to set robot back to safe mode
        xarm_msgs::msg::RobotMode mode_msg;
        mode_msg.data = 0; // POSITION mode
        mode_publisher->publish(mode_msg);
        
        // Wait for robot state to potentially recover
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        
        // Only return to IDLE if robot has recovered
        if (hasRobotError()) {
            RCLCPP_WARN(this->get_logger(), "Robot still has error after recovery attempt");
            // Stay in error state, don't transition to IDLE
            current_state = ERROR;
        } else {
            RCLCPP_INFO(this->get_logger(), "Robot state recovered, transitioning to IDLE");
            enterState(IDLE);
        }
    }

    void PickAndPlaceStateMachine::idle(uint8_t next_state)
    {

        if(previous_state == MANUAL_MODE){
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: IDLE state entered from MANUAL_MODE", 
                       getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
            // Publish 0 for MOVEIT MODE to robot state 
            xarm_msgs::msg::RobotMode mode_msg;
            mode_msg.data = 0;
            mode_publisher->publish(mode_msg);
            enterState(IDLE);
            return;
        }

        // 2. Check robot state and handle errors
        if (hasRobotError()) {
            RCLCPP_ERROR(this->get_logger(), "Robot error detected in IDLE state (code: %d), transitioning to ERROR", xarm_error_code);
            enterState(ERROR);
            return;
        }

        // 3. Only process transitions if we have a valid command from external source
        // If next_state is IDLE (0), and we're already in IDLE, don't process transition
        if (next_state == static_cast<uint8_t>(IDLE) && current_state == IDLE) {
            // Stay in IDLE - no action needed
            return;
        }

        // 4. Validate Requested Transition
        STATES requested_state = static_cast<STATES>(next_state);

        // Check if the requested state is valid (not UNKNOWN)
        if (getStateName(requested_state) == "UNKNOWN") {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid command value: %d", next_state);
            return;
        }

        if (!isValidTransition(requested_state))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejected invalid transition from IDLE to %s",
                        getStateName(requested_state).c_str());
            return;
        }

        // 5. Check robot state compatibility before transitioning
        if (!shouldTransitionBasedOnRobotState(requested_state)) {
            RCLCPP_WARN(this->get_logger(), 
                        "Robot state (%s) not compatible with requested transition to %s",
                        getXarmStateName(current_xarm_state).c_str(),
                        getStateName(requested_state).c_str());
            return;
        }

        // 6. Handle Special Cases
        if (requested_state == FINAL)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Direct transition from IDLE to FINAL initiated");
            // Add any special finalization logic here
            enterState(FINAL);
            return;
        }

        // 7. Normal Transition Handling
        switch (requested_state)
        {
        case IDLE:
            // Already in IDLE - no action needed
            break;
        case MOVING:
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Starting normal operation sequence - Robot state: %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getXarmStateName(current_xarm_state).c_str());
            enterState(MOVING);
            break;

        case MANUAL_MODE:
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Entering manual control mode - Robot state: %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getXarmStateName(current_xarm_state).c_str());
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

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose_1, geometry_msgs::msg::Pose target_pose_2)
    {
        // 1. Check robot state before attempting movement
        if (!shouldTransitionBasedOnRobotState(MOVING)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot execute movement due to robot state");
            enterState(ERROR);
            return;
        }

        // 2. Entry Actions
        RCLCPP_INFO(this->get_logger(),
                    "MOVING to pose (x: %.3f, y: %.3f, z: %.3f) - Robot state: %s",
                    target_pose_1.position.x,
                    target_pose_1.position.y,
                    target_pose_1.position.z,
                    getXarmStateName(current_xarm_state).c_str());

        // 3. Execute Movement
        bool success = xarm7_object->planToTargetPose(target_pose_1, false);
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "MOVEMENT TO (x: %.3f, y: %.3f, z: %.3f) FAILED!!",
                         target_pose_1.position.x,
                         target_pose_1.position.y,
                         target_pose_1.position.z);
            enterState(ERROR);
            return;
        }else{

            // 4. Wait for robot to finish movement before determining next state
            // Give some time for the robot state to update to RUNNING
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            
            // Wait for movement completion (robot goes from RUNNING back to SLEEPING)
            auto start_time = this->now();
            auto timeout = std::chrono::seconds(10); // 10 second timeout
            
            while (isRobotMoving() && (this->now() - start_time) < rclcpp::Duration(timeout)) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                rclcpp::spin_some(shared_from_this());
            }
            
            // Check if movement completed successfully
            if (isRobotMoving()) {
                RCLCPP_WARN(this->get_logger(), "Movement timeout - robot still moving");
            } else if (hasRobotError()) {
                RCLCPP_ERROR(this->get_logger(), "Robot error detected after movement");
                enterState(ERROR);
                return;
            }

            // 5. Determine Next State based on movement completion
            STATES next = MOVING; // Default to state moving

            // Check for special pose conditions
            if (isPickPlacePose(target_pose_1)){
                // Transition based on gripper state
                next = (gripper_state == "FREE") ? PICKING : PLACING;
            }
            // Check for manual mode override
            else if (current_command == MANUAL_MODE && isValidTransition(MANUAL_MODE)){
                next = MANUAL_MODE;
            }
            // Check for return to idle
            else if (current_command == IDLE && isValidTransition(IDLE)){
                next = IDLE;
            }else if (isFinalPose(target_pose_1, target_pose_2)){
                next = FINAL;
            }else{
                next = IDLE;
                current_command = IDLE;
            }

            if (current_command == ERROR){
                next = ERROR;
            }

            enterState(next);
            previous_pose = target_pose_1;

        }

    }

    void PickAndPlaceStateMachine::moving(geometry_msgs::msg::Pose target_pose)
    {
        // Simplified version without state transition logic
        RCLCPP_DEBUG(this->get_logger(), "Executing movement only");

        if (current_command == ERROR)
        {
            RCLCPP_ERROR(this->get_logger(), "ROBOT MOVEMENT STOPPED!!!");
            return;
        }

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
                std::abs(pose.orientation.y) < tolerance &&
                std::abs(pose.orientation.z) < tolerance &&
                std::abs(pose.orientation.w) < tolerance);
    }

    bool PickAndPlaceStateMachine::isFinalPose(const geometry_msgs::msg::Pose &pose,
                                               const geometry_msgs::msg::Pose &previous_pose)
    {
        // Check if we've reached the final position
        const double position_tolerance = 0.005; // 5mm
        const double orientation_tolerance = 0.001;

        return (std::abs(pose.position.x - previous_pose.position.x)) < position_tolerance &&
               (std::abs(pose.position.y - previous_pose.position.y)) < position_tolerance &&
               (std::abs(pose.position.z - previous_pose.position.z)) < position_tolerance &&
               (std::abs(pose.orientation.x - previous_pose.orientation.x)) < orientation_tolerance &&
               (std::abs(pose.orientation.y - previous_pose.orientation.y)) < orientation_tolerance &&
               (std::abs(pose.orientation.z - previous_pose.orientation.z)) < orientation_tolerance &&
               (std::abs(pose.orientation.w - previous_pose.orientation.w)) < orientation_tolerance;
    }

    void PickAndPlaceStateMachine::picking(geometry_msgs::msg::Pose target_pose_1, geometry_msgs::msg::Pose target_pose_2)
    {
        // 1. Entry Actions
        RCLCPP_INFO(this->get_logger(),
                    "Starting PICK operation at (x: %.3f, y: %.3f, z: %.3f)",
                    target_pose_1.position.x,
                    target_pose_1.position.y,
                    target_pose_1.position.z);

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
        if (isValidTransition(static_cast<STATES>(current_command)))
        {
            next = static_cast<STATES>(current_command);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Requested transition to %s not allowed, defaulting to MOVING",
                        getStateName(static_cast<STATES>(current_command)).c_str());
        }

        if(isFinalPose(target_pose_1, target_pose_2))
        {
            next = FINAL; // Auto-transition if at final pose
        }

        enterState(next);
    }

    void PickAndPlaceStateMachine::placing(geometry_msgs::msg::Pose target_pose)
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
        else if (isValidTransition(static_cast<STATES>(current_command)))
        {
            next = static_cast<STATES>(current_command);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Requested transition to %s not allowed, using %s",
                        getStateName(static_cast<STATES>(current_command)).c_str(),
                        getStateName(next).c_str());
        }

        enterState(next);
    }

    void PickAndPlaceStateMachine::manual_mode(uint8_t next_state)
    {
        // 1. Entry Actions (only execute on first entry)
        if (previous_state != MANUAL_MODE)
        {
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Entered MANUAL_MODE from %s - Robot will be set to TEACHING_JOINT mode", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getStateName(previous_state).c_str());
            // Entry actions are handled in enterState() function
            return;
        }

        // 2. Monitor robot state while in manual mode
        if (hasRobotError()) {
            RCLCPP_ERROR(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Robot error in MANUAL_MODE (code: %d), transitioning to ERROR", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        xarm_error_code);
            enterState(ERROR);
            return;
        }

        // 3. Only process transitions if they come from external sources (state topic)
        STATES requested_state = static_cast<STATES>(next_state);
        
        // Check if the requested state is valid (not UNKNOWN)
        if (getStateName(requested_state) == "UNKNOWN") {
            RCLCPP_DEBUG(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Ignoring invalid command value: %d", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        next_state);
            return;
        }
        
        // If staying in manual mode, no action needed
        if(requested_state == MANUAL_MODE){
            RCLCPP_DEBUG(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Staying in MANUAL_MODE", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str());
            return;
        }
        
        // Validate transition
        if (!isValidTransition(requested_state)) {
            RCLCPP_WARN(this->get_logger(),
                        "MODE: %s -- XARM7-STATE: %s: Invalid transition from MANUAL_MODE to %s",
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getStateName(requested_state).c_str());
            return;
        }
        
        // Check robot state compatibility
        if (!shouldTransitionBasedOnRobotState(requested_state)) {
            RCLCPP_WARN(this->get_logger(),
                        "MODE: %s -- XARM7-STATE: %s: Robot state (%s) not compatible with transition to %s",
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getXarmStateName(current_xarm_state).c_str(),
                        getStateName(requested_state).c_str());
            return;
        }

        // When leaving manual mode, return to POSITION mode first
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Leaving MANUAL_MODE, transitioning to %s", 
                    getXarmModeName(current_xarm_mode).c_str(),
                    getStateName(current_state).c_str(),
                    getStateName(requested_state).c_str());
        
        xarm_msgs::msg::RobotMode mode_msg;
        mode_msg.data = 0; // POSITION mode
        mode_publisher->publish(mode_msg);
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Set robot back to POSITION mode", 
                    getXarmModeName(current_xarm_mode).c_str(),
                    getStateName(current_state).c_str());
        
        // Transition to the requested state
        enterState(requested_state);
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

    std::string PickAndPlaceStateMachine::getXarmStateName(XarmState state)
    {
        switch (state)
        {
        case XarmState::RUNNING:
            return "RUNNING";
        case XarmState::SLEEPING:
            return "SLEEPING";
        case XarmState::PAUSED:
            return "PAUSED";
        case XarmState::STOPPED:
            return "STOPPED";
        case XarmState::CONFIG_CHANGED:
            return "CONFIG_CHANGED";
        default:
            return "UNKNOWN";
        }
    }

    std::string PickAndPlaceStateMachine::getXarmModeName(XarmMode mode)
    {
        switch (mode)
        {
        case XarmMode::POSITION:
            return "POSITION";
        case XarmMode::SERVOJ:
            return "SERVOJ";
        case XarmMode::TEACHING_JOINT:
            return "TEACHING_JOINT";
        default:
            return "UNKNOWN";
        }
    }

    bool PickAndPlaceStateMachine::isRobotReady()
    {
        if (!robot_state_received) {
            RCLCPP_WARN(this->get_logger(), "No robot state received yet");
            return false;
        }
        
        // Robot is ready if it's in SLEEPING state with no errors
        return (current_xarm_state == XarmState::SLEEPING && xarm_error_code == 0);
    }

    bool PickAndPlaceStateMachine::isRobotMoving()
    {
        return (current_xarm_state == XarmState::RUNNING);
    }

    bool PickAndPlaceStateMachine::hasRobotError()
    {
        return (xarm_error_code != 0 || current_xarm_state == XarmState::STOPPED);
    }

    bool PickAndPlaceStateMachine::shouldTransitionBasedOnRobotState(STATES intended_state)
    {
        // Check for error conditions first
        if (hasRobotError()) {
            RCLCPP_ERROR(this->get_logger(), "Robot has error (code: %d) or is stopped, cannot transition to %s", 
                         xarm_error_code, getStateName(intended_state).c_str());
            return false;
        }

        // Check state-specific conditions
        switch (intended_state) {
            case MOVING:
            case PICKING:
            case PLACING:
                // Movement states require robot to be ready or already moving
                if (!isRobotReady() && !isRobotMoving()) {
                    RCLCPP_WARN(this->get_logger(), "Robot not ready for movement (state: %s), cannot transition to %s",
                                getXarmStateName(current_xarm_state).c_str(), getStateName(intended_state).c_str());
                    return false;
                }
                break;
                
            case MANUAL_MODE:
                // Manual mode needs the robot to be ready (not necessarily TEACHING_JOINT mode, we'll set that)
                if (current_xarm_state == XarmState::STOPPED || current_xarm_state == XarmState::CONFIG_CHANGED) {
                    RCLCPP_WARN(this->get_logger(), "Robot not available for manual mode (state: %s)",
                                getXarmStateName(current_xarm_state).c_str());
                    return false;
                }
                break;
                
            case IDLE:
            case FINAL:
            case ERROR:
                // These states can be entered regardless of robot state
                break;
        }

        return true;
    }

} // namespace simple_state_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_state_machine::PickAndPlaceStateMachine)