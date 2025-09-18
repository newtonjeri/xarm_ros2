/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <xarm_msgs/srv/mtc_task_request.hpp>
#include <string>
#include <unordered_map>

namespace xarm7_mtc
{
    /**
     * @brief Task Command Bridge Node
     * 
     * This node subscribes to /task_command topic and converts string commands
     * from Unity into service calls to the MTC service node.
     * 
     * Supported commands:
     * - "spindle_2_task" -> calls MTC service with task_name: "spindle_2"
     * - "pinion_gear_task" -> calls MTC service with task_name: "pinion_gear"
     * - "idler_gear_task" -> calls MTC service with task_name: "idler_gear"
     * - "cover_plate_task" -> calls MTC service with task_name: "cover_plate"
     */
    class TaskCommandBridge : public rclcpp::Node
    {
    public:
        explicit TaskCommandBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~TaskCommandBridge() = default;

    private:
        /**
         * @brief Callback for task command messages
         * @param msg String message containing the task command
         */
        void taskCommandCallback(const std_msgs::msg::String::SharedPtr msg);

        /**
         * @brief Call the MTC service with the given task information
         * @param task_name Name of the task to execute
         * @param task_id ID of the task (defaults to 1)
         * @return true if service call was successful, false otherwise
         */
        bool callMTCService(const std::string& task_name, int task_id = 1);

        /**
         * @brief Parse the command string and extract task information
         * @param command The command string received from Unity
         * @return Task name to be sent to MTC service, empty if invalid command
         */
        std::string parseCommand(const std::string& command);

        /**
         * @brief Initialize the command mapping
         */
        void initializeCommandMapping();

        // ROS 2 interfaces
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_command_sub_;
        rclcpp::Client<xarm_msgs::srv::MTCTaskRequest>::SharedPtr mtc_service_client_;

        // Command mapping from Unity commands to MTC task names
        std::unordered_map<std::string, std::string> command_mapping_;

        // Parameters
        std::string mtc_service_name_;
    };

} // namespace xarm7_mtc