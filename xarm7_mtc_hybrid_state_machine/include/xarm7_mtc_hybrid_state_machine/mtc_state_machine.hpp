/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/msg/robot_mode.hpp>
#include <xarm_msgs/srv/mtc_task_request.hpp>
#include "xarm7_mtc_hybrid_state_machine/base_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/spindle_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/pinion_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/idler_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/cover_task.hpp"

#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

namespace xarm7_mtc
{
    // Simplified state machine states for task-focused operation
    enum class StateMachineState : uint8_t
    {
        IDLE = 0,           // Ready for new tasks
        EXECUTING_TASK = 1, // Currently executing MTC task
        COMPLETED = 2,      // Task completed successfully
        ERROR = 3           // Task failed or hardware error
    };

    // Robot operation modes  
    enum class OperationMode : uint8_t
    {
        SERVOJ = 1,    // Precise task execution mode
        MANUAL = 2     // Manual control/teaching mode
    };

    // Task types matching service interface
    enum class TaskType : uint8_t
    {
        SPINDLE_2 = 1,
        PINION_GEAR = 2,
        IDLER_GEAR = 3, 
        COVER_PLATE = 4
    };

    // Xarm robot states (from RobotMsg)
    enum class XarmState : int16_t
    {
        RUNNING = 0,
        SLEEPING = 1,
        PAUSED = 2,
        STOPPED = 3,
        CONFIG_CHANGED = 4
    };

    // Xarm robot modes (from RobotMsg)
    enum class XarmMode : int16_t
    {
        POSITION = 0,       // Position control mode
        SERVOJ = 1,         // ServoJ mode for smooth motion
        TEACHING_JOINT = 2  // Teaching mode
    };

    /**
     * @brief Simplified hybrid state machine for MTC-based pick-and-place operations
     * 
     * This state machine:
     * - Manages task execution using MoveIt Task Constructor
     * - Tracks simplified states (IDLE, EXECUTING_TASK, COMPLETED, ERROR)
     * - Handles mode transitions between SERVOJ and MANUAL modes
     * - Monitors robot hardware state for error detection
     * - Provides stage-level progress tracking for MTC tasks
     */
    class MTCStateMachine : public rclcpp::Node
    {
    public:
        explicit MTCStateMachine(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~MTCStateMachine();

        // Main state machine interface
        bool executeTask(TaskType task_type, 
                        const geometry_msgs::msg::Pose* custom_pick_pose = nullptr,
                        const geometry_msgs::msg::Pose* custom_place_pose = nullptr);
        bool requestModeChange(OperationMode target_mode);
        
        // State and status queries
        StateMachineState getCurrentState() const;
        OperationMode getCurrentMode() const;
        std::string getLastError() const;
        std::vector<StageInfo> getCurrentTaskStages() const;
        
        // Task management
        bool cancelCurrentTask();
        bool isTaskExecuting() const;
        double getTaskProgress() const;

    private:
        // Initialization
        void initializeNode();
        void createTaskInstances();
        
        // Main execution loop
        void executionLoop();
        void processStateMachine();
        
        // State management
        void enterState(StateMachineState new_state);
        void handleIdleState();
        void handleExecutingTaskState();
        void handleCompletedState();
        void handleErrorState();
        
        // Mode management
        void requestModeSwitch(OperationMode mode);
        void handleModeTransition();
        
        // Robot state monitoring
        void robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg);
        bool isRobotReady() const;
        bool hasHardwareError() const;
        
        // Task execution
        bool startTaskExecution(TaskType task_type,
                              const geometry_msgs::msg::Pose* custom_pick_pose,
                              const geometry_msgs::msg::Pose* custom_place_pose);
        void monitorTaskExecution();
        BaseTask* getTaskInstance(TaskType task_type);
        
        // Error handling
        void setError(const std::string& error_message);
        void clearError();
        
        // Utility functions
        std::string getStateName(StateMachineState state) const;
        std::string getModeName(OperationMode mode) const;
        std::string getTaskName(TaskType task_type) const;
        std::string getXarmStateName(XarmState state) const;
        std::string getXarmModeName(XarmMode mode) const;

        // Node state
        rclcpp::Node::SharedPtr node_ptr_;
        
        // State machine variables
        std::atomic<StateMachineState> current_state_;
        std::atomic<OperationMode> current_mode_;
        std::atomic<bool> mode_change_requested_;
        OperationMode requested_mode_;
        
        // Task execution variables
        std::atomic<bool> task_execution_active_;
        TaskType current_task_type_;
        std::unique_ptr<BaseTask> current_task_;
        geometry_msgs::msg::Pose custom_pick_pose_;
        geometry_msgs::msg::Pose custom_place_pose_;
        bool use_custom_pick_pose_;
        bool use_custom_place_pose_;
        
        // Robot state tracking
        XarmState current_xarm_state_;
        XarmMode current_xarm_mode_;
        int16_t xarm_error_code_;
        int16_t xarm_warning_code_;
        bool robot_state_received_;
        
        // Error tracking
        std::string last_error_;
        std::atomic<bool> has_error_;
        
        // Task instances (pre-created for efficiency)
        std::unique_ptr<SpindleTask> spindle_task_;
        std::unique_ptr<PinionTask> pinion_task_;
        std::unique_ptr<IdlerTask> idler_task_;
        std::unique_ptr<CoverTask> cover_task_;
        
        // Threading and synchronization
        std::unique_ptr<std::thread> execution_thread_;
        std::atomic<bool> shutdown_requested_;
        mutable std::mutex state_mutex_;
        mutable std::mutex task_mutex_;
        std::condition_variable task_completion_cv_;
        
        // ROS communication
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_sub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::RobotMode>::SharedPtr mode_request_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        
        // Timing
        rclcpp::TimerBase::SharedPtr execution_timer_;
        std::chrono::steady_clock::time_point task_start_time_;
        std::chrono::steady_clock::time_point task_end_time_;
        
        // Configuration parameters
        double execution_loop_rate_;
        double task_timeout_;
        bool enable_stage_monitoring_;
        bool enable_collision_checking_;
    };

} // namespace xarm7_mtc