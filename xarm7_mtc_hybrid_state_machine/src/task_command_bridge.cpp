/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/task_command_bridge.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace xarm7_mtc
{
    TaskCommandBridge::TaskCommandBridge(const rclcpp::NodeOptions& options)
        : Node("task_command_bridge", options)
    {
        // Declare parameters
        this->declare_parameter("mtc_service_name", "/xarm/mtc_task_service");

        // Get parameters
        mtc_service_name_ = this->get_parameter("mtc_service_name").as_string();

        // Initialize command mapping
        initializeCommandMapping();

        // Create subscriber to task command topic
        task_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_command", 10,
            std::bind(&TaskCommandBridge::taskCommandCallback, this, std::placeholders::_1));

        // Create service client for MTC service
        mtc_service_client_ = this->create_client<xarm_msgs::srv::MTCTaskRequest>(mtc_service_name_);

        RCLCPP_INFO(this->get_logger(), "Task Command Bridge initialized");
        RCLCPP_INFO(this->get_logger(), "  - Listening on topic: /task_command");
        RCLCPP_INFO(this->get_logger(), "  - MTC service: %s", mtc_service_name_.c_str());
        
        // Log available commands
        RCLCPP_INFO(this->get_logger(), "Available commands:");
        for (const auto& [command, task_name] : command_mapping_) {
            RCLCPP_INFO(this->get_logger(), "  - '%s' -> task: '%s'", command.c_str(), task_name.c_str());
        }
    }

    void TaskCommandBridge::taskCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received task command: '%s'", command.c_str());

        // Parse the command to get the task name
        std::string task_name = parseCommand(command);
        
        if (task_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'. Ignoring.", command.c_str());
            return;
        }

        // Call the MTC service
        bool success = callMTCService(task_name);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Successfully processed command '%s' -> task '%s'", 
                       command.c_str(), task_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to process command '%s' -> task '%s'", 
                        command.c_str(), task_name.c_str());
        }
    }

    bool TaskCommandBridge::callMTCService(const std::string& task_name, int task_id)
    {
        // Wait for service to be available (no timeout, wait indefinitely)
        RCLCPP_INFO(this->get_logger(), "Waiting for MTC service '%s' to be available...", mtc_service_name_.c_str());
        mtc_service_client_->wait_for_service();

        // Create service request
        auto request = std::make_shared<xarm_msgs::srv::MTCTaskRequest::Request>();
        request->request_type = xarm_msgs::srv::MTCTaskRequest::Request::REQUEST_TASK;
        request->task_id = task_id;
        request->task_name = task_name;

        RCLCPP_INFO(this->get_logger(), "Calling MTC service with task: '%s', ID: %d", 
                   task_name.c_str(), task_id);

        try {
            // Call service and wait for response (no timeout)
            auto future = mtc_service_client_->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) 
                == rclcpp::FutureReturnCode::SUCCESS) {
                
                auto response = future.get();
                
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "MTC service call successful for task '%s'", task_name.c_str());
                    if (!response->message.empty()) {
                        RCLCPP_INFO(this->get_logger(), "Service response: %s", response->message.c_str());
                    }
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "MTC service call failed for task '%s': %s", 
                               task_name.c_str(), response->message.c_str());
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to complete MTC service call for task '%s'", task_name.c_str());
                return false;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during service call for task '%s': %s", 
                        task_name.c_str(), e.what());
            return false;
        }
    }

    std::string TaskCommandBridge::parseCommand(const std::string& command)
    {
        // Look up the command in the mapping
        auto it = command_mapping_.find(command);
        if (it != command_mapping_.end()) {
            return it->second;
        }

        // If not found, check for variations (case insensitive, with/without spaces)
        std::string normalized_command = command;
        std::transform(normalized_command.begin(), normalized_command.end(), 
                      normalized_command.begin(), ::tolower);
        
        // Remove spaces
        normalized_command.erase(std::remove(normalized_command.begin(), normalized_command.end(), ' '), 
                                normalized_command.end());

        // Try to find normalized version
        for (const auto& [cmd, task] : command_mapping_) {
            std::string normalized_mapped = cmd;
            std::transform(normalized_mapped.begin(), normalized_mapped.end(), 
                          normalized_mapped.begin(), ::tolower);
            normalized_mapped.erase(std::remove(normalized_mapped.begin(), normalized_mapped.end(), ' '), 
                                   normalized_mapped.end());
            
            if (normalized_command == normalized_mapped) {
                return task;
            }
        }

        return ""; // Unknown command
    }

    void TaskCommandBridge::initializeCommandMapping()
    {
        // Map Unity commands to MTC task names
        command_mapping_["spindle_2_task"] = "spindle_2";
        command_mapping_["pinion_gear_task"] = "pinion_gear";
        command_mapping_["idler_gear_task"] = "idler_gear";
        command_mapping_["cover_plate_task"] = "cover_plate";
        
        // Alternative command formats for flexibility
        command_mapping_["spindle_task"] = "spindle_2";
        command_mapping_["spindle"] = "spindle_2";
        command_mapping_["pinion_task"] = "pinion_gear";
        command_mapping_["pinion"] = "pinion_gear";
        command_mapping_["idler_task"] = "idler_gear";
        command_mapping_["idler"] = "idler_gear";
        command_mapping_["cover_task"] = "cover_plate";
        command_mapping_["cover"] = "cover_plate";
    }

} // namespace xarm7_mtc

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<xarm7_mtc::TaskCommandBridge>();
    
    RCLCPP_INFO(node->get_logger(), "Task Command Bridge node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}