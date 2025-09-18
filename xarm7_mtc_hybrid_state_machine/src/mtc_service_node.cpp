/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/mtc_task_request.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "xarm7_mtc_hybrid_state_machine/base_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/spindle_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/pinion_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/idler_task.hpp"
#include "xarm7_mtc_hybrid_state_machine/cover_task.hpp"

using namespace xarm7_mtc;

// Simplified state machine states for task-focused operation
enum class StateMachineState : uint8_t
{
    IDLE = 0,           // Ready for new tasks
    EXECUTING_TASK = 1, // Currently executing MTC task
    COMPLETED = 2,      // Task completed successfully
    ERROR = 3           // Task failed or hardware error
};

class MTCServiceNode : public rclcpp::Node
{
public:
    MTCServiceNode() : Node("mtc_service_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MTC Service Node...");
        
        // Create service server
        service_ = this->create_service<xarm_msgs::srv::MTCTaskRequest>(
            "/mtc_task_service",
            std::bind(&MTCServiceNode::handleTaskRequest, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // Create status publishers
        state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/mtc_state_machine_state", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/mtc_state_machine/status", 10);
        
        // Create robot state subscriber
        robot_state_sub_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            "/xarm/robot_states", 10,
            std::bind(&MTCServiceNode::robotStateCallback, this, std::placeholders::_1));
        
        // Initialize state
        current_state_ = StateMachineState::IDLE;
        
        RCLCPP_INFO(this->get_logger(), "MTC Service Node initialized successfully");
    }
    
public:
    void initializeStateMachine()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MTC State Machine...");
        
        // Create task instances using this node's shared pointer
        auto node_ptr = this->shared_from_this();
        
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
    
private:
    
    void robotStateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr /*msg*/)
    {
        // Update robot state information
        robot_state_received_ = true;
        // Store relevant robot state data as needed
    }

private:
    void handleTaskRequest(
        const std::shared_ptr<xarm_msgs::srv::MTCTaskRequest::Request> request,
        std::shared_ptr<xarm_msgs::srv::MTCTaskRequest::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received MTC task request - Type: %d, ID: %d", 
                   request->request_type, request->task_id);
        
        if (request->request_type == xarm_msgs::srv::MTCTaskRequest::Request::REQUEST_TASK) {
            // Handle task execution request
            response->success = handleTaskExecution(request);
        }
        else if (request->request_type == xarm_msgs::srv::MTCTaskRequest::Request::REQUEST_MODE_CHANGE) {
            // Handle mode change request  
            response->success = handleModeChange(request);
        }
        else {
            response->success = false;
            response->message = "Invalid request type";
        }
        
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Task request completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task request failed: %s", response->message.c_str());
        }
    }
    
    bool handleTaskExecution(const std::shared_ptr<xarm_msgs::srv::MTCTaskRequest::Request> request)
    {
        if (current_state_ != StateMachineState::IDLE) {
            RCLCPP_WARN(this->get_logger(), "Cannot execute task: not in IDLE state");
            return false;
        }
        
        current_state_ = StateMachineState::EXECUTING_TASK;
        publishState();
        
        bool success = false;
        std::string task_name = request->task_name;
        
        try {
            if (task_name == "spindle_2") {
                // First create the task, then plan it, then execute it
                if (spindle_task_->createTask() && spindle_task_->planTask()) {
                    success = spindle_task_->executeTask();
                } else {
                    success = false;
                }
            }
            else if (task_name == "pinion_gear") {
                if (pinion_task_->createTask() && pinion_task_->planTask()) {
                    success = pinion_task_->executeTask();
                } else {
                    success = false;
                }
            }
            else if (task_name == "idler_gear") {
                if (idler_task_->createTask() && idler_task_->planTask()) {
                    success = idler_task_->executeTask();
                } else {
                    success = false;
                }
            }
            else if (task_name == "cover_plate") {
                if (cover_task_->createTask() && cover_task_->planTask()) {
                    success = cover_task_->executeTask();
                } else {
                    success = false;
                }
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Unknown task: %s", task_name.c_str());
                success = false;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Task execution failed with exception: %s", e.what());
            success = false;
        }
        
        if (success) {
            current_state_ = StateMachineState::COMPLETED;
            publishStatus("Task " + task_name + " completed successfully");
        } else {
            current_state_ = StateMachineState::ERROR;
            publishStatus("Task " + task_name + " failed");
        }
        
        publishState();
        return success;
    }
    
    bool handleModeChange(const std::shared_ptr<xarm_msgs::srv::MTCTaskRequest::Request> request)
    {
        RCLCPP_INFO(this->get_logger(), "Mode change request to mode: %d", request->mode_id);
        // For now, just acknowledge the mode change request
        // The actual mode switching is handled by the mode switcher node
        return true;
    }
    
    void publishState()
    {
        auto msg = std_msgs::msg::UInt8();
        msg.data = static_cast<uint8_t>(current_state_);
        state_pub_->publish(msg);
    }
    
    void publishStatus(const std::string& status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
    }

    // Member variables
    std::unique_ptr<SpindleTask> spindle_task_;
    std::unique_ptr<PinionTask> pinion_task_;
    std::unique_ptr<IdlerTask> idler_task_;
    std::unique_ptr<CoverTask> cover_task_;
    
    rclcpp::Service<xarm_msgs::srv::MTCTaskRequest>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_sub_;
    
    StateMachineState current_state_{StateMachineState::IDLE};
    bool robot_state_received_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MTCServiceNode>();
    
    // Initialize the state machine after the node is fully constructed
    node->initializeStateMachine();
    
    RCLCPP_INFO(node->get_logger(), "MTC Service Node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}