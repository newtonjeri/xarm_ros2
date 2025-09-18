/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/mtc_state_machine.hpp"
#include <thread>

namespace xarm7_mtc
{
    MTCStateMachine::MTCStateMachine(const rclcpp::NodeOptions& options)
        : Node("mtc_state_machine", options)
        , current_state_(StateMachineState::IDLE)
        , current_mode_(OperationMode::SERVOJ)
        , mode_change_requested_(false)
        , task_execution_active_(false)
        , shutdown_requested_(false)
        , robot_state_received_(false)
        , has_error_(false)
    {
        initializeNode();
    }

    MTCStateMachine::~MTCStateMachine()
    {
        shutdown_requested_ = true;
        if (execution_thread_ && execution_thread_->joinable()) {
            execution_thread_->join();
        }
        RCLCPP_INFO(this->get_logger(), "MTCStateMachine destroyed");
    }

    void MTCStateMachine::initializeNode()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MTC State Machine...");
        
        // Create task instances
        createTaskInstances();
        
        // Create ROS communication interfaces
        robot_state_sub_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            "/xarm/robot_states", 10,
            std::bind(&MTCStateMachine::robotStateCallback, this, std::placeholders::_1));
            
        state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/mtc_state_machine/state", 10);
        mode_request_pub_ = this->create_publisher<xarm_msgs::msg::RobotMode>("/mtc_mode_request", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/mtc_state_machine/status", 10);
        
        // Start execution thread
        execution_thread_ = std::make_unique<std::thread>(&MTCStateMachine::executionLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "MTC State Machine initialized successfully");
    }

    void MTCStateMachine::createTaskInstances()
    {
        auto node_ptr = shared_from_this();
        
        spindle_task_ = std::make_unique<SpindleTask>(node_ptr);
        pinion_task_ = std::make_unique<PinionTask>(node_ptr);
        idler_task_ = std::make_unique<IdlerTask>(node_ptr);
        cover_task_ = std::make_unique<CoverTask>(node_ptr);
        
        // Initialize all tasks
        spindle_task_->init();
        pinion_task_->init();
        idler_task_->init();
        cover_task_->init();
        
        RCLCPP_INFO(this->get_logger(), "All MTC task instances created and initialized");
    }

    bool MTCStateMachine::executeTask(TaskType task_type, 
                                    const geometry_msgs::msg::Pose* custom_pick_pose,
                                    const geometry_msgs::msg::Pose* custom_place_pose)
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        
        if (current_state_ != StateMachineState::IDLE) {
            setError("Cannot execute task: not in IDLE state");
            return false;
        }
        
        if (!isRobotReady()) {
            setError("Cannot execute task: robot not ready");
            return false;
        }
        
        return startTaskExecution(task_type, custom_pick_pose, custom_place_pose);
    }

    void MTCStateMachine::executionLoop()
    {
        auto rate = std::chrono::milliseconds(100); // 10Hz execution rate
        
        while (!shutdown_requested_) {
            try {
                processStateMachine();
                std::this_thread::sleep_for(rate);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in execution loop: %s", e.what());
                setError("Execution loop exception: " + std::string(e.what()));
            }
        }
    }

    void MTCStateMachine::processStateMachine()
    {
        // Publish current state
        std_msgs::msg::UInt8 state_msg;
        state_msg.data = static_cast<uint8_t>(current_state_.load());
        state_pub_->publish(state_msg);
        
        // Process based on current state
        switch (current_state_) {
            case StateMachineState::IDLE:
                handleIdleState();
                break;
            case StateMachineState::EXECUTING_TASK:
                handleExecutingTaskState();
                break;
            case StateMachineState::COMPLETED:
                handleCompletedState();
                break;
            case StateMachineState::ERROR:
                handleErrorState();
                break;
        }
        
        // Handle mode change requests
        if (mode_change_requested_.load()) {
            handleModeTransition();
        }
    }

    void MTCStateMachine::enterState(StateMachineState new_state)
    {
        if (current_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                       getStateName(current_state_).c_str(),
                       getStateName(new_state).c_str());
            current_state_ = new_state;
        }
    }

    void MTCStateMachine::handleIdleState()
    {
        // Ready for new tasks or mode changes
        // Check for hardware errors
        if (hasHardwareError()) {
            setError("Hardware error detected");
            enterState(StateMachineState::ERROR);
        }
    }

    void MTCStateMachine::handleExecutingTaskState()
    {
        // Monitor task execution
        monitorTaskExecution();
    }

    void MTCStateMachine::handleCompletedState()
    {
        // Task completed successfully
        RCLCPP_INFO(this->get_logger(), "Task completed successfully, returning to IDLE");
        task_execution_active_ = false;
        enterState(StateMachineState::IDLE);
    }

    void MTCStateMachine::handleErrorState()
    {
        // Stay in error state until manual intervention
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "State machine in ERROR state: %s", last_error_.c_str());
    }

    // Remaining methods are placeholder implementations
    bool MTCStateMachine::requestModeChange(OperationMode target_mode) { return false; }
    StateMachineState MTCStateMachine::getCurrentState() const { return current_state_; }
    OperationMode MTCStateMachine::getCurrentMode() const { return current_mode_; }
    std::string MTCStateMachine::getLastError() const { return last_error_; }
    std::vector<StageInfo> MTCStateMachine::getCurrentTaskStages() const { return {}; }
    bool MTCStateMachine::cancelCurrentTask() { return false; }
    bool MTCStateMachine::isTaskExecuting() const { return task_execution_active_; }
    double MTCStateMachine::getTaskProgress() const { return 0.0; }
    
    void MTCStateMachine::requestModeSwitch(OperationMode mode) { /* TODO */ }
    void MTCStateMachine::handleModeTransition() { /* TODO */ }
    void MTCStateMachine::robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg) { /* TODO */ }
    bool MTCStateMachine::isRobotReady() const { return true; }
    bool MTCStateMachine::hasHardwareError() const { return false; }
    
    bool MTCStateMachine::startTaskExecution(TaskType task_type,
                                           const geometry_msgs::msg::Pose* custom_pick_pose,
                                           const geometry_msgs::msg::Pose* custom_place_pose) { return false; }
    void MTCStateMachine::monitorTaskExecution() { /* TODO */ }
    BaseTask* MTCStateMachine::getTaskInstance(TaskType task_type) { return nullptr; }
    
    void MTCStateMachine::setError(const std::string& error_message)
    {
        last_error_ = error_message;
        has_error_ = true;
        enterState(StateMachineState::ERROR);
        RCLCPP_ERROR(this->get_logger(), "Error: %s", error_message.c_str());
    }
    
    void MTCStateMachine::clearError() { /* TODO */ }
    
    std::string MTCStateMachine::getStateName(StateMachineState state) const
    {
        switch (state) {
            case StateMachineState::IDLE: return "IDLE";
            case StateMachineState::EXECUTING_TASK: return "EXECUTING_TASK";
            case StateMachineState::COMPLETED: return "COMPLETED";
            case StateMachineState::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }
    
    std::string MTCStateMachine::getModeName(OperationMode mode) const { return "TODO"; }
    std::string MTCStateMachine::getTaskName(TaskType task_type) const { return "TODO"; }
    std::string MTCStateMachine::getXarmStateName(XarmState state) const { return "TODO"; }
    std::string MTCStateMachine::getXarmModeName(XarmMode mode) const { return "TODO"; }

} // namespace xarm7_mtc