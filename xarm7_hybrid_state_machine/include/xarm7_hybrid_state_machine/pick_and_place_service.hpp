/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#ifndef XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SERVICE_HPP_
#define XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SERVICE_HPP_

#include <memory>
#include <string>
#include <map>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "xarm_msgs/srv/pick_and_place_service.hpp"
#include "xarm_msgs/msg/robot_state_and_target_pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

using namespace std::chrono_literals;

namespace pick_and_place_service
{
    enum PART_TYPES
    {
        SPINDLE_2 = 1,
        PINION_GEAR = 2,
        IDLER_GEAR = 3,
        COVER_PLATE = 4
    };

    class PickAndPlaceServiceNode : public rclcpp::Node
    {
    public:
        PickAndPlaceServiceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~PickAndPlaceServiceNode();

    private:
        // Service callback
        void pickAndPlaceService(
            const std::shared_ptr<xarm_msgs::srv::PickAndPlaceService::Request> request,
            std::shared_ptr<xarm_msgs::srv::PickAndPlaceService::Response> response);

        // State machine monitoring callback
        void stateMachineStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);

        // Helper functions
        bool isValidPartName(const std::string& part_name);
        int getPartId(const std::string& part_name);
        std::map<std::string, geometry_msgs::msg::Pose> getPartPoses(const std::string& part_name);
        void sendPickAndPlaceCommand(int part_id, const std::map<std::string, geometry_msgs::msg::Pose>& poses);
        bool waitForCompletion(const std::chrono::seconds& timeout);  // Will be simplified

        // Publishers and subscribers
        rclcpp::Service<xarm_msgs::srv::PickAndPlaceService>::SharedPtr pick_place_service_;
        rclcpp::Publisher<xarm_msgs::msg::RobotStateAndTargetPose>::SharedPtr state_topic_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_machine_subscriber_;

        // State monitoring (using atomic for thread-safe access)
        std::atomic<uint8_t> current_state_machine_state_;
        std::atomic<bool> operation_in_progress_;
        std::atomic<bool> operation_successful_;
        
        // Completion notification (replaces polling waitForCompletion)
        std::condition_variable completion_cv_;
        std::mutex completion_mutex_;
        bool operation_completed_;
        
        // Planning scene interface for collision objects
        moveit::planning_interface::PlanningSceneInterface psi_;

        // Timer for operation timeout
        rclcpp::TimerBase::SharedPtr timeout_timer_;

        // Predefined poses from experiment_002.cpp
        void initializePoses();
        std::map<std::string, std::map<std::string, geometry_msgs::msg::Pose>> part_poses_;
    };

} // namespace pick_and_place_service

#endif // XARM7_HYBRID_STATE_MACHINE__PICK_AND_PLACE_SERVICE_HPP_
