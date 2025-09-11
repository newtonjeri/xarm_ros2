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

    PickAndPlaceStateMachine::PickAndPlaceStateMachine() : 
        Node("pick_and_place_sm"),
        current_state(IDLE),
        previous_state(IDLE),
        gripper_state("FREE"),
        planned_sequence_(),
        sequence_index_(0),
        sequence_active_(false),
        movement_in_progress_(false), 
        gripper_operation_in_progress_(false)
    {
        initializeNode();
    }

    PickAndPlaceStateMachine::PickAndPlaceStateMachine(const rclcpp::NodeOptions &options) : 
        Node("pick_and_place_sm", options),
        current_state(IDLE),
        previous_state(IDLE),
        gripper_state("FREE"),
        planned_sequence_(),
        sequence_index_(0),
        sequence_active_(false),
        movement_in_progress_(false), 
        gripper_operation_in_progress_(false)
    {
        initializeNode();
    }

    void PickAndPlaceStateMachine::initializeNode()
    {
        // Initialize state and mode callbacks
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

        // Create recovery service
        recovery_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/xarm7_state_machine/recover",
            std::bind(&PickAndPlaceStateMachine::recoveryService, this, 
                     std::placeholders::_1, std::placeholders::_2));

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

        RCLCPP_INFO(this->get_logger(), "PickAndPlaceStateMachine initialized with callback groups for multithreading");
    }

    PickAndPlaceStateMachine::~PickAndPlaceStateMachine()
    {
        RCLCPP_INFO(this->get_logger(), "PickAndPlaceStateMachine destructor called");
        // Component nodes don't need to manage their own executor threads
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
        // Modal transition maps based on robot mode
        static const std::map<XarmMode, std::map<STATES, std::set<STATES>>> modal_transitions = {
            // MOVEIT-MODE (POSITION/SERVOJ): Full pick-and-place capability
            {XarmMode::POSITION, {
                {IDLE, {IDLE, MOVING, FINAL, ERROR}},
                {MOVING, {MOVING, PICKING, PLACING, IDLE, FINAL, ERROR}},
                {PICKING, {IDLE, MOVING, ERROR}},
                {PLACING, {IDLE, MOVING, FINAL, ERROR}},
                {FINAL, {IDLE, ERROR}},
                {ERROR, {IDLE}}
            }},
            {XarmMode::SERVOJ, {
                {IDLE, {IDLE, MOVING, FINAL, ERROR}},
                {MOVING, {MOVING, PICKING, PLACING, IDLE, FINAL, ERROR}},
                {PICKING, {IDLE, MOVING, ERROR}},
                {PLACING, {IDLE, MOVING, FINAL, ERROR}},
                {FINAL, {IDLE, ERROR}},
                {ERROR, {IDLE}}
            }},
            // MANUAL-MODE (TEACHING_JOINT): Limited to basic movement
            {XarmMode::TEACHING_JOINT, {
                {IDLE, {IDLE, MOVING, ERROR}},
                {MOVING, {MOVING, IDLE, ERROR}},
                {ERROR, {IDLE}}
            }}
        };

        // Special case: Allow forced ERROR transition from any state
        if (next_state == ERROR)
        {
            RCLCPP_DEBUG(this->get_logger(), "Allowing emergency transition to ERROR state");
            return true;
        }

        // Get the appropriate transition map for current mode
        auto mode_transitions = modal_transitions.find(current_xarm_mode);
        if (mode_transitions == modal_transitions.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "No transition map found for mode %s!",
                         getXarmModeName(current_xarm_mode).c_str());
            return false;
        }

        // Find current state in mode-specific transition map
        auto current_state_transitions = mode_transitions->second.find(current_state);
        if (current_state_transitions == mode_transitions->second.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Current state %s not valid for mode %s!",
                         getStateName(current_state).c_str(),
                         getXarmModeName(current_xarm_mode).c_str());
            return false;
        }

        // Check if requested transition exists in current mode
        bool valid = current_state_transitions->second.count(next_state) > 0;

        // Additional modal validation
        if (valid && !isStateValidForCurrentMode(next_state))
        {
            RCLCPP_WARN(this->get_logger(),
                        "State %s not valid for current mode %s",
                        getStateName(next_state).c_str(),
                        getXarmModeName(current_xarm_mode).c_str());
            valid = false;
        }

        if (!valid)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid transition: %s → %s (Mode: %s)",
                        getStateName(current_state).c_str(),
                        getStateName(next_state).c_str(),
                        getXarmModeName(current_xarm_mode).c_str());

            // Log available transitions for debugging
            std::string available;
            for (const auto &s : current_state_transitions->second)
            {
                available += getStateName(s) + " ";
            }
            RCLCPP_DEBUG(this->get_logger(),
                         "Available from %s in mode %s: %s",
                         getStateName(current_state).c_str(),
                         getXarmModeName(current_xarm_mode).c_str(),
                         available.c_str());
        }

        return valid;
    }

    void PickAndPlaceStateMachine::enterState(STATES new_state)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enterStateUnsafe(new_state);
    }

    void PickAndPlaceStateMachine::enterStateUnsafe(STATES new_state)
    {
        // This function assumes state_mutex_ is already locked by the caller
        
        // Check for no-op transitions
        if(new_state == current_state) {
            RCLCPP_DEBUG(this->get_logger(), "No state change needed - already in %s state", 
                        getStateName(new_state).c_str());
            return;
        }
        
        if(new_state == FINAL && current_state == IDLE) {
            RCLCPP_DEBUG(this->get_logger(), "No operation performed - already in IDLE state");
            return; // Stay in current state
        }
        
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Transitioning from %s to %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getStateName(current_state).c_str(),
                        getStateName(new_state).c_str());

        previous_state = current_state;
        current_state = new_state;

        // Publish state change for synchronization with other nodes (publish new state)
        std_msgs::msg::UInt8 state_msg;
        state_msg.data = static_cast<uint8_t>(current_state);
        state_publisher->publish(state_msg);

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
        
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s", 
                   getXarmModeName(current_xarm_mode).c_str(), getStateName(current_state).c_str());
    }

    void PickAndPlaceStateMachine::robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        current_xarm_state = static_cast<XarmState>(msg->state);
        current_xarm_mode = static_cast<XarmMode>(msg->mode);
        xarm_error_code = msg->err;
        xarm_warning_code = msg->warn;
        robot_state_received = true;

        // Handle CONFIG_CHANGED as mode switch indicator
        if (current_xarm_state == XarmState::CONFIG_CHANGED) {
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Mode switch detected (CONFIG_CHANGED)", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str());
            
            // Mode switch detected - validate current state for new mode
            if (!isStateValidForCurrentMode(current_state)) {
                RCLCPP_WARN(this->get_logger(), 
                           "MODE: %s -- XARM7-STATE: %s: Current state %s not valid for new mode %s, transitioning to IDLE",
                           getXarmModeName(current_xarm_mode).c_str(),
                           getStateName(current_state).c_str(),
                           getStateName(current_state).c_str(),
                           getXarmModeName(current_xarm_mode).c_str());
                
                // Force transition to safe state (IDLE) for the new mode
                if (isValidTransition(IDLE)) {
                    enterStateUnsafe(IDLE);  // Use unsafe version since we already hold the lock
                }
            }
        }

        // Log significant state changes
        static XarmState last_state = XarmState::SLEEPING;
        static XarmMode last_mode = XarmMode::POSITION;
        static int16_t last_error = 0;
        
        if (current_xarm_state != last_state) {
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Xarm state changed: %s -> %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getXarmStateName(last_state).c_str(),
                        getXarmStateName(current_xarm_state).c_str());
            last_state = current_xarm_state;
        }
        
        if (current_xarm_mode != last_mode) {
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Xarm mode changed: %s -> %s", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str(),
                        getXarmModeName(last_mode).c_str(),
                        getXarmModeName(current_xarm_mode).c_str());
            last_mode = current_xarm_mode;
        }

        if (xarm_error_code != last_error && xarm_error_code != 0) {
            RCLCPP_ERROR(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Xarm error detected: %d", 
                         getXarmModeName(current_xarm_mode).c_str(),
                         getStateName(current_state).c_str(),
                         xarm_error_code);
            last_error = xarm_error_code;
            
            // Error detected - transition to ERROR state
            if (isValidTransition(ERROR)) {
                enterStateUnsafe(ERROR);  // Use unsafe version since we already hold the lock
            }
        } else if (last_error != 0 && xarm_error_code == 0 && current_state == ERROR) {
            // Error has cleared while in ERROR state - attempt automatic recovery
            RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Error cleared, attempting automatic recovery", 
                        getXarmModeName(current_xarm_mode).c_str(),
                        getStateName(current_state).c_str());
            last_error = 0;
            
            if (isRobotReady()) {
                RCLCPP_INFO(this->get_logger(), "Robot ready - automatic recovery to IDLE successful");
                enterStateUnsafe(IDLE);  // Use unsafe version since we already hold the lock
            } else {
                RCLCPP_WARN(this->get_logger(), "Robot not ready yet - staying in ERROR state");
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Xarm Status - State: %s, Mode: %s, Error: %d, Warning: %d",
                    getXarmModeName(current_xarm_mode).c_str(),
                    getStateName(current_state).c_str(),
                    getXarmStateName(current_xarm_state).c_str(),
                    getXarmModeName(current_xarm_mode).c_str(),
                    xarm_error_code, xarm_warning_code);
    }

    void PickAndPlaceStateMachine::stateCallback(const xarm_msgs::msg::RobotStateAndTargetPose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> pose_lock(pose_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        
        command_source = CommandSource::STATE_TOPIC;
        current_command = msg->robot_next_state;
        target_pose_1 = msg->target_pose_1;
        target_pose_2 = msg->target_pose_2;
        
        RCLCPP_INFO(this->get_logger(), "MODE: %s -- XARM7-STATE: %s: Received new command: %d (%s)", 
                    getXarmModeName(current_xarm_mode).c_str(),
                    getStateName(current_state).c_str(), 
                    current_command,
                    getStateName(static_cast<STATES>(current_command)).c_str());

        // Process the external command using our new sequence planning logic
        if (current_state == IDLE) {
            bool success = processExternalCommand(msg->robot_next_state, msg->target_pose_1, msg->target_pose_2);
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to process external command");
                enterStateUnsafe(ERROR);  // Use unsafe version since we already hold the lock
            }
        } else if (current_state == ERROR) {
            // Allow recovery from ERROR state if robot is ready
            if (isRobotReady() && !hasRobotError()) {
                RCLCPP_INFO(this->get_logger(), "Command received while in ERROR state - attempting recovery");
                enterStateUnsafe(IDLE);  // Use unsafe version since we already hold the lock
                // Now process the command
                bool success = processExternalCommand(msg->robot_next_state, msg->target_pose_1, msg->target_pose_2);
                if (!success) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to process external command after recovery");
                    enterStateUnsafe(ERROR);  // Use unsafe version since we already hold the lock
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Command received while in ERROR state, but robot not ready for recovery");
                RCLCPP_INFO(this->get_logger(), "Robot state: %s, Error code: %d, Ready: %s", 
                           getXarmStateName(current_xarm_state).c_str(), 
                           xarm_error_code, 
                           isRobotReady() ? "true" : "false");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received command while in %s state, ignoring", getStateName(current_state).c_str());
        }
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

    void PickAndPlaceStateMachine::recoveryService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Suppress unused parameter warning
        
        RCLCPP_INFO(this->get_logger(), "Manual recovery service called");
        
        if (current_state != ERROR) {
            response->success = false;
            response->message = "Robot not in ERROR state - recovery not needed";
            RCLCPP_WARN(this->get_logger(), "Recovery service called but robot not in ERROR state");
            return;
        }
        
        // Cancel any ongoing operations
        cancelAllOperations();
        resetSequence();
        
        // Reset internal state
        gripper_state = "FREE";
        current_command = 0;
        
        // Check robot state
        if (hasRobotError()) {
            response->success = false;
            response->message = "Robot still has hardware error (code: " + std::to_string(xarm_error_code) + 
                              "). Fix hardware issue first, then retry recovery.";
            RCLCPP_ERROR(this->get_logger(), "Manual recovery failed - robot still has error: %d", xarm_error_code);
            return;
        }
        
        if (!isRobotReady()) {
            response->success = false;
            response->message = "Robot not ready (state: " + getXarmStateName(current_xarm_state) + 
                              "). Wait for robot to be in SLEEPING state, then retry recovery.";
            RCLCPP_WARN(this->get_logger(), "Manual recovery failed - robot not ready: %s", 
                       getXarmStateName(current_xarm_state).c_str());
            return;
        }
        
        // Force recovery to IDLE state
        previous_state = ERROR;
        current_state = IDLE;
        enterState(IDLE);
        
        response->success = true;
        response->message = "Recovery successful - robot returned to IDLE state";
        RCLCPP_INFO(this->get_logger(), "Manual recovery successful - robot returned to IDLE state");
    }

    void PickAndPlaceStateMachine::stateTransitionLogic()
    {
        // If we have an active sequence, execute it
        if (sequence_active_) {
            executeSequence();
            return;
        }

        // Otherwise execute current state logic
        switch (current_state)
        {
        case IDLE:
            executeIdleState();
            break;

        case MOVING:
            executeMovingState();
            break;

        case PICKING:
            executePickingState();
            break;

        case PLACING:
            executePlacingState();
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
        static auto last_error_time = std::chrono::steady_clock::now();
        static int error_recovery_attempts = 0;
        
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_error = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_error_time);
        
        // Reset attempt counter if enough time has passed since last error
        if (time_since_last_error.count() > 10) {
            error_recovery_attempts = 0;
        }
        
        last_error_time = current_time;
        error_recovery_attempts++;
        
        RCLCPP_ERROR(this->get_logger(), "Error encountered in state %s - Robot state: %s, Error code: %d (Attempt %d/3)",
                     getStateName(previous_state).c_str(),
                     getXarmStateName(current_xarm_state).c_str(),
                     xarm_error_code,
                     error_recovery_attempts);

        // Cancel any ongoing operations
        cancelAllOperations();
        resetSequence();
        
        // Reset internal state variables
        gripper_state = "FREE";
        
        // Try recovery based on number of attempts
        if (error_recovery_attempts <= 3) {
            RCLCPP_INFO(this->get_logger(), "Attempting error recovery (attempt %d/3)", error_recovery_attempts);
            
            // Try to set robot back to safe mode
            xarm_msgs::msg::RobotMode mode_msg;
            mode_msg.data = 0; // POSITION mode
            mode_publisher->publish(mode_msg);
            
            // Wait for robot state to potentially recover
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            
            // Check if robot has recovered
            if (!hasRobotError() && isRobotReady()) {
                RCLCPP_INFO(this->get_logger(), "Robot state recovered, transitioning to IDLE");
                error_recovery_attempts = 0; // Reset counter on successful recovery
                previous_state = ERROR;
                current_state = IDLE;
                current_command = 0;
                enterState(IDLE);
                return;
            } else {
                RCLCPP_WARN(this->get_logger(), "Robot still has error after recovery attempt %d", error_recovery_attempts);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Maximum recovery attempts reached. Manual intervention required.");
            RCLCPP_ERROR(this->get_logger(), "To recover: Fix robot error, then send any valid command to resume operation");
        }
        
        // Stay in error state - recovery only happens when:
        // 1. Robot error clears AND robot becomes ready
        // 2. New command is received (will be processed if robot is ready)
    }

    // ===== NEW SEQUENCE PLANNING AND EXECUTION FUNCTIONS =====
    
    bool PickAndPlaceStateMachine::processExternalCommand(int8_t robot_next_state, 
                                                         const geometry_msgs::msg::Pose& pose1,
                                                         const geometry_msgs::msg::Pose& pose2)
    {
        RCLCPP_INFO(this->get_logger(), "Processing external command: %d", robot_next_state);
        
        // Input validation
        STATES requested_state = static_cast<STATES>(robot_next_state);
        if (!isStateValidForCurrentMode(requested_state)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid state %s for current mode %s", 
                        getStateName(requested_state).c_str(),
                        getXarmModeName(current_xarm_mode).c_str());
            return false;
        }
        
        if (!isRobotReady()) {
            RCLCPP_ERROR(this->get_logger(), "Robot not ready for operation");
            return false;
        }

        // Plan the sequence
        std::vector<STATES> sequence = planSequence(robot_next_state, pose1, pose2);
        if (sequence.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No sequence needed for command %d - already in appropriate state", robot_next_state);
            return true; // Success - no work needed
        }

        // Start sequence execution
        planned_sequence_ = sequence;
        sequence_index_ = 0;
        sequence_active_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Starting sequence with %zu steps", sequence.size());
        
        // Log the planned sequence
        std::string sequence_str = "Planned sequence: ";
        for (size_t i = 0; i < sequence.size(); ++i) {
            sequence_str += getStateName(sequence[i]);
            if (i < sequence.size() - 1) sequence_str += " → ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", sequence_str.c_str());
        
        return true;
    }

    std::vector<STATES> PickAndPlaceStateMachine::planSequence(int8_t robot_next_state,
                                                              const geometry_msgs::msg::Pose& pose1,
                                                              const geometry_msgs::msg::Pose& pose2)
    {
        std::vector<STATES> sequence;
        
        // Don't create unnecessary sequences for IDLE→FINAL transitions
        if (current_state == IDLE && robot_next_state == FINAL) {
            RCLCPP_DEBUG(this->get_logger(), "Already in IDLE state, no need to transition to FINAL");
            return {}; // Return empty sequence to indicate no work needed
        }
        
        // Pose analysis
        bool same_poses = posesEqual(pose1, pose2);
        bool is_pick_pose = isPickPlacePose(pose1);
        bool is_place_pose = isPickPlacePose(pose2);
        bool gripper_open = (gripper_state == "FREE");
        bool gripper_closed = (gripper_state == "GRASPING");
        
        RCLCPP_DEBUG(this->get_logger(), "Pose analysis - same: %s, pick: %s, place: %s, gripper: %s",
                    same_poses ? "true" : "false",
                    is_pick_pose ? "true" : "false", 
                    is_place_pose ? "true" : "false",
                    gripper_open ? "open" : "closed");

        switch (robot_next_state) {
            case MOVING:
                if (same_poses) {
                    // Simple movement: IDLE->MOVING->FINAL
                    sequence = {MOVING, FINAL};
                }
                else if (is_pick_pose && gripper_open && same_poses) {
                    // Pick only: IDLE->MOVING->PICKING->FINAL  
                    sequence = {MOVING, PICKING, FINAL};
                }
                else if (is_place_pose && gripper_closed && same_poses) {
                    // Place only: IDLE->MOVING->PLACING->FINAL
                    sequence = {MOVING, PLACING, FINAL};
                }
                else if (is_pick_pose && gripper_open && is_place_pose) {
                    // Full pick-and-place: IDLE->MOVING->PICKING->MOVING->PLACING->FINAL
                    sequence = {MOVING, PICKING, MOVING, PLACING, FINAL};
                }
                else if (is_pick_pose && gripper_open) {
                    // Pick and move: IDLE->MOVING->PICKING->MOVING->FINAL
                    sequence = {MOVING, PICKING, MOVING, FINAL};
                }
                else if (is_place_pose && gripper_closed) {
                    // Move and place: IDLE->MOVING->PLACING->FINAL
                    sequence = {MOVING, PLACING, FINAL};
                }
                else {
                    // General movement: IDLE->MOVING->MOVING->FINAL
                    sequence = {MOVING, MOVING, FINAL};
                }
                break;
                
            case PICKING:
                if (!is_pick_pose) {
                    RCLCPP_ERROR(this->get_logger(), "Target pose not suitable for picking");
                    return {};
                }
                if (!gripper_open) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot pick - gripper not open");
                    return {};
                }
                if (same_poses) {
                    // Pick in place: IDLE->PICKING->FINAL
                    sequence = {PICKING, FINAL};
                } else {
                    // Pick and move: IDLE->PICKING->MOVING->FINAL
                    sequence = {PICKING, MOVING, FINAL};
                }
                break;
                
            case PLACING:
                if (!is_place_pose) {
                    RCLCPP_ERROR(this->get_logger(), "Target pose not suitable for placing");
                    return {};
                }
                if (!gripper_closed) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot place - gripper not holding object");
                    return {};
                }
                if (same_poses) {
                    // Place in place: IDLE->PLACING->FINAL
                    sequence = {PLACING, FINAL};
                } else {
                    // Move and place: IDLE->MOVING->PLACING->FINAL
                    sequence = {MOVING, PLACING, FINAL};
                }
                break;
                
            case FINAL:
                // Direct completion
                sequence = {FINAL};
                break;
                
            case ERROR:
                // Emergency stop
                sequence = {ERROR};
                break;
                
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown robot_next_state: %d", robot_next_state);
                return {};
        }
        
        return sequence;
    }

    // ===== SEQUENCE EXECUTION ENGINE =====
    
    void PickAndPlaceStateMachine::executeSequence()
    {
        STATES current_step;
        bool should_execute_state = false;
        
        {
            std::lock_guard<std::mutex> lock(sequence_mutex_);
            
            if (!sequence_active_ || sequence_index_ >= planned_sequence_.size()) {
                RCLCPP_DEBUG(this->get_logger(), "Sequence completed or not active");
                resetSequence();
                return;
            }

            current_step = planned_sequence_[sequence_index_];
            should_execute_state = (current_step == current_state);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Executing sequence step %zu/%zu: %s", 
                    sequence_index_ + 1, planned_sequence_.size(), getStateName(current_step).c_str());

        // Execute current step
        if (current_step != current_state) {
            enterState(current_step);
        } else if (should_execute_state) {
            // We're already in the target state, execute the state logic
            switch (current_step) {
                case MOVING:
                    executeMovingState();
                    break;
                case PICKING:
                    executePickingState();
                    break;
                case PLACING:
                    executePlacingState();
                    break;
                case FINAL:
                    handleCompletion();
                    break;
                case ERROR:
                    handleError();
                    break;
                case IDLE:
                    // For IDLE, just advance to next step if there is one
                    advanceSequence();
                    break;
            }
        }
    }

    bool PickAndPlaceStateMachine::advanceSequence()
    {
        if (!sequence_active_) return false;
        
        sequence_index_++;
        if (sequence_index_ >= planned_sequence_.size()) {
            RCLCPP_INFO(this->get_logger(), "Sequence completed successfully");
            resetSequence();
            return false;
        }
        return true;
    }

    void PickAndPlaceStateMachine::resetSequence()
    {
        std::lock_guard<std::mutex> lock(sequence_mutex_);
        
        sequence_active_ = false;
        sequence_index_ = 0;
        planned_sequence_.clear();
        movement_in_progress_ = false;
        gripper_operation_in_progress_ = false;
    }

    void PickAndPlaceStateMachine::cancelAllOperations()
    {
        if (xarm7_object && xarm7_object->move_group) {
            xarm7_object->move_group->stop();
        }
        if (xarm_gripper_object && xarm_gripper_object->move_group) {
            xarm_gripper_object->move_group->stop();
        }
        movement_in_progress_ = false;
        gripper_operation_in_progress_ = false;
    }

    // ===== NEW STATE EXECUTION FUNCTIONS =====
    
    void PickAndPlaceStateMachine::executeIdleState()
    {
        // IDLE state just waits for external commands (already handled in stateCallback)
        // No action needed here
    }

    void PickAndPlaceStateMachine::executeMovingState()
    {
        RCLCPP_INFO(this->get_logger(), "executeMovingState() called - sequence_index: %zu", sequence_index_);
        
        // Determine which pose to move to based on sequence context
        geometry_msgs::msg::Pose target_pose;
        
        if (sequence_index_ == 0 || 
            (sequence_index_ > 0 && planned_sequence_[sequence_index_-1] == PICKING)) {
            // First movement or after picking - move to target_pose_1
            target_pose = target_pose_1;
            RCLCPP_INFO(this->get_logger(), "Using target_pose_1: (%.3f, %.3f, %.3f)", 
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
        } else {
            // Later movement - move to target_pose_2  
            target_pose = target_pose_2;
            RCLCPP_INFO(this->get_logger(), "Using target_pose_2: (%.3f, %.3f, %.3f)", 
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
        }

        bool success = performMovement(target_pose);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Movement successful, advancing sequence");
            // Advance to next step in sequence
            advanceSequence();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Movement failed, transitioning to ERROR");
            resetSequence();
            enterState(ERROR);
        }
    }

    void PickAndPlaceStateMachine::executePickingState()
    {
        bool success = performPickingSequence(target_pose_1);
        if (success) {
            gripper_state = "GRASPING";
            advanceSequence();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Picking failed, transitioning to ERROR");
            resetSequence(); 
            enterState(ERROR);
        }
    }

    void PickAndPlaceStateMachine::executePlacingState()
    {
        bool success = performPlacingSequence(target_pose_2);
        if (success) {
            gripper_state = "FREE";
            advanceSequence();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Placing failed, transitioning to ERROR");
            resetSequence();
            enterState(ERROR);
        }
    }

    // ===== PURE OPERATION FUNCTIONS =====
    
    bool PickAndPlaceStateMachine::performMovement(const geometry_msgs::msg::Pose& target_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Performing movement to (%.3f, %.3f, %.3f)",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        if (!shouldTransitionBasedOnRobotState(MOVING)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot execute movement due to robot state");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Robot state check passed, calling planToTargetPose()");
        bool success = xarm7_object->planToTargetPose(target_pose, false);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Movement planning failed");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Movement planning successful, waiting for completion");
        
        // // Wait for movement completion 
        // auto start_time = std::chrono::steady_clock::now();
        // auto timeout = std::chrono::seconds(10);
        
        // rclcpp::sleep_for(std::chrono::milliseconds(100)); // Let movement start
        
        // int wait_cycles = 0;
        // while (isRobotMoving() && (std::chrono::steady_clock::now() - start_time) < timeout) {
        //     wait_cycles++;
        //     if (wait_cycles % 20 == 0) { // Log every second
        //         RCLCPP_INFO(this->get_logger(), "Waiting for movement completion... (%d cycles)", wait_cycles);
        //     }
        //     rclcpp::sleep_for(std::chrono::milliseconds(50));
        //     rclcpp::spin_some(shared_from_this());
        // }
        
        // if (isRobotMoving()) {
        //     RCLCPP_WARN(this->get_logger(), "Movement timeout - robot still moving after %d seconds", 10);
        //     return false;
        // } else if (hasRobotError()) {
        //     RCLCPP_ERROR(this->get_logger(), "Robot error detected after movement");
        //     return false;
        // }

        previous_pose = target_pose;
        RCLCPP_INFO(this->get_logger(), "Movement completed successfully");
        return true;
    }

    bool PickAndPlaceStateMachine::performPickingSequence(const geometry_msgs::msg::Pose& pick_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Performing picking sequence at (%.3f, %.3f, %.3f)",
                    pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);

        // 1. Open gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
            RCLCPP_ERROR(this->get_logger(), "Gripper open failed");
            return false;
        }

        // 2. Approach (5cm above)
        geometry_msgs::msg::Pose approach_pose = pick_pose;
        approach_pose.position.z -= 0.05;
        if (!performMovement(approach_pose)) {
            return false;
        }

        // 3. Close gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.84, 0.84, 0.84, 0.84, 0.84, 0.84})) {
            RCLCPP_ERROR(this->get_logger(), "Gripper close failed");
            return false;
        }

        // 4. Retreat
        if (!performMovement(pick_pose)) {
            return false;
        }

        return true;
    }

    bool PickAndPlaceStateMachine::performPlacingSequence(const geometry_msgs::msg::Pose& place_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Performing placing sequence at (%.3f, %.3f, %.3f)",
                    place_pose.position.x, place_pose.position.y, place_pose.position.z);

        if (gripper_state != "GRASPING") {
            RCLCPP_ERROR(this->get_logger(), "Cannot place - gripper not holding object");
            return false;
        }

        // 1. Approach (5cm above)

        // 1. Move to top (place position)
        if (!performMovement(place_pose)) {
            return false;
        }

        //. Approach the placing pose
        geometry_msgs::msg::Pose approach_pose = place_pose;
        approach_pose.position.z -= 0.05;
        if (!performMovement(approach_pose)) {
            return false;
        }

        // 3. Open gripper
        if (!xarm_gripper_object->gripperOpenAndClose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
            RCLCPP_ERROR(this->get_logger(), "Gripper open failed");
            return false;
        }

        // 4. Retreat
        if (!performMovement(place_pose)) {
            return false;
        }

        return true;
    }

    // ===== UTILITY AND HELPER FUNCTIONS =====
    
    std::string PickAndPlaceStateMachine::getStateName(STATES state)
    {
        switch (state) {
            case IDLE: return "IDLE";
            case MOVING: return "MOVING";
            case PICKING: return "PICKING";
            case PLACING: return "PLACING";
            case FINAL: return "FINAL";
            case ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    std::string PickAndPlaceStateMachine::getXarmStateName(XarmState state)
    {
        switch (state) {
            case XarmState::RUNNING: return "RUNNING";
            case XarmState::SLEEPING: return "SLEEPING";
            case XarmState::PAUSED: return "PAUSED";
            case XarmState::STOPPED: return "STOPPED";
            case XarmState::CONFIG_CHANGED: return "CONFIG_CHANGED";
            default: return "UNKNOWN";
        }
    }

    std::string PickAndPlaceStateMachine::getXarmModeName(XarmMode mode)
    {
        switch (mode) {
            case XarmMode::POSITION: return "POSITION";
            case XarmMode::SERVOJ: return "SERVOJ";
            case XarmMode::TEACHING_JOINT: return "TEACHING_JOINT";
            default: return "UNKNOWN";
        }
    }

    bool PickAndPlaceStateMachine::isRobotReady()
    {
        return (current_xarm_state == XarmState::SLEEPING) 
               && xarm_error_code == 0 && robot_state_received;
    }

    bool PickAndPlaceStateMachine::isRobotMoving()
    {
        return current_xarm_state == XarmState::RUNNING;
    }

    bool PickAndPlaceStateMachine::hasRobotError()
    {
        return xarm_error_code != 0 || current_xarm_state == XarmState::STOPPED;
    }

    bool PickAndPlaceStateMachine::shouldTransitionBasedOnRobotState(STATES intended_state)
    {
        // Check if robot is in error state
        if (hasRobotError()) {
            RCLCPP_ERROR(this->get_logger(), "Robot has error (code: %d), cannot transition to %s", 
                        xarm_error_code, getStateName(intended_state).c_str());
            return false;
        }

        // For MOVING state, we just need robot not to have errors
        // Robot can be SLEEPING (ready to move) or RUNNING (already moving)
        if (intended_state == MOVING) {
            RCLCPP_INFO(this->get_logger(), "Movement state check passed - robot state: %d, error: %d", 
                       static_cast<int>(current_xarm_state), xarm_error_code);
            return true; // Already checked for errors above
        }

        // For other states, check if robot is ready (SLEEPING state)
        if (!isRobotReady() && intended_state != ERROR && intended_state != IDLE) {
            RCLCPP_WARN(this->get_logger(), "Robot not ready, cannot transition to %s (state: %d, error: %d)", 
                       getStateName(intended_state).c_str(), 
                       static_cast<int>(current_xarm_state), xarm_error_code);
            return false;
        }

        return true;
    }

    bool PickAndPlaceStateMachine::isStateValidForCurrentMode(STATES state)
    {
        // Define which states are valid for each mode
        static const std::map<XarmMode, std::set<STATES>> valid_states_per_mode = {
            // MOVEIT-MODE (POSITION/SERVOJ): Full capability
            {XarmMode::POSITION, {IDLE, MOVING, PICKING, PLACING, FINAL, ERROR}},
            {XarmMode::SERVOJ, {IDLE, MOVING, PICKING, PLACING, FINAL, ERROR}},
            // MANUAL-MODE (TEACHING_JOINT): Limited capability
            {XarmMode::TEACHING_JOINT, {IDLE, MOVING, ERROR}}
        };

        auto it = valid_states_per_mode.find(current_xarm_mode);
        if (it == valid_states_per_mode.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown mode: %s", getXarmModeName(current_xarm_mode).c_str());
            return false;
        }

        return it->second.count(state) > 0;
    }

    bool PickAndPlaceStateMachine::isPickPlacePose(const geometry_msgs::msg::Pose& pose)
    {
        // Check for characteristic pick/place pose orientation
        const double tolerance = 0.001;
        return (std::abs(pose.orientation.x - 1.0) < tolerance &&
                std::abs(pose.orientation.y) < tolerance &&
                std::abs(pose.orientation.z) < tolerance &&
                std::abs(pose.orientation.w) < tolerance);
    }

    bool PickAndPlaceStateMachine::posesEqual(const geometry_msgs::msg::Pose& pose1, 
                                             const geometry_msgs::msg::Pose& pose2, 
                                             double position_tolerance,
                                             double orientation_tolerance)
    {
        // Check position
        double pos_diff = std::sqrt(
            std::pow(pose1.position.x - pose2.position.x, 2) +
            std::pow(pose1.position.y - pose2.position.y, 2) +
            std::pow(pose1.position.z - pose2.position.z, 2)
        );

        if (pos_diff > position_tolerance) {
            return false;
        }

        // Check orientation (quaternion difference)
        double orient_diff = std::sqrt(
            std::pow(pose1.orientation.x - pose2.orientation.x, 2) +
            std::pow(pose1.orientation.y - pose2.orientation.y, 2) +
            std::pow(pose1.orientation.z - pose2.orientation.z, 2) +
            std::pow(pose1.orientation.w - pose2.orientation.w, 2)
        );

        return orient_diff <= orientation_tolerance;
    }

} // namespace simple_state_machine

// Register the component with the ROS2 component system
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_state_machine::PickAndPlaceStateMachine)
