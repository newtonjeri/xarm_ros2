/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_hybrid_state_machine/pick_and_place_service.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

namespace pick_and_place_service
{
    PickAndPlaceServiceNode::PickAndPlaceServiceNode(const rclcpp::NodeOptions &options) 
        : Node("pick_and_place_service_node", options),
          current_state_machine_state_(0), // IDLE
          operation_in_progress_(false),
          operation_completed_(false),
          operation_successful_(false)
    {
        // Initialize poses from experiment_002.cpp
        initializePoses();

        // Create service server
        pick_place_service_ = this->create_service<xarm_msgs::srv::PickAndPlaceService>(
            "/xarm7/pick_and_place_service",
            std::bind(&PickAndPlaceServiceNode::pickAndPlaceService, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Create publisher to communicate with state machine
        state_topic_publisher_ = this->create_publisher<xarm_msgs::msg::RobotStateAndTargetPose>(
            "/xarm7_state_topic", 10);

        // Subscribe to state machine state for monitoring
        state_machine_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/xarm7_state_machine_state", 10,
            std::bind(&PickAndPlaceServiceNode::stateMachineStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pick and Place Service Node initialized");
        RCLCPP_INFO(this->get_logger(), "Service available at: /xarm7/pick_and_place_service");
        RCLCPP_INFO(this->get_logger(), "Supported parts: spindle_2, pinion_gear, idler_gear, cover_plate");
    }

    PickAndPlaceServiceNode::~PickAndPlaceServiceNode()
    {
        RCLCPP_INFO(this->get_logger(), "Pick and Place Service Node shutting down");
    }

    void PickAndPlaceServiceNode::pickAndPlaceService(
        const std::shared_ptr<xarm_msgs::srv::PickAndPlaceService::Request> request,
        std::shared_ptr<xarm_msgs::srv::PickAndPlaceService::Response> response)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        RCLCPP_INFO(this->get_logger(), "Received pick and place request for part: %s", request->part_name.c_str());

        // Check if operation is already in progress
        if (operation_in_progress_) {
            response->success = false;
            response->message = "Another pick and place operation is already in progress";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Validate part name
        if (!isValidPartName(request->part_name)) {
            response->success = false;
            response->message = "Invalid part name: " + request->part_name + 
                               ". Supported parts: spindle_2, pinion_gear, idler_gear, cover_plate";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Get part ID and poses
        int part_id = getPartId(request->part_name);
        auto poses = getPartPoses(request->part_name);

        // Set operation flags
        operation_in_progress_ = true;
        operation_completed_ = false;
        operation_successful_ = false;

        // Send command to state machine
        sendPickAndPlaceCommand(part_id, poses);

        // Wait for completion with timeout (60 seconds)
        bool completed = waitForCompletion(std::chrono::seconds(60));

        // Reset operation flags
        operation_in_progress_ = false;

        if (completed && operation_successful_) {
            response->success = true;
            response->message = "Pick and place operation for " + request->part_name + " completed successfully";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            
            // Remove collision object after successful operation
            psi_.removeCollisionObjects({request->part_name});
        } else if (completed && !operation_successful_) {
            response->success = false;
            response->message = "Pick and place operation for " + request->part_name + " failed";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "Pick and place operation for " + request->part_name + " timed out";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void PickAndPlaceServiceNode::stateMachineStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_machine_state_ = msg->data;

        // Check if operation completed (FINAL state = 4, ERROR state = 5)
        if (operation_in_progress_) {
            if (current_state_machine_state_ == 4) { // FINAL
                operation_completed_ = true;
                operation_successful_ = true;
                RCLCPP_INFO(this->get_logger(), "State machine reached FINAL state - operation successful");
            } else if (current_state_machine_state_ == 5) { // ERROR
                operation_completed_ = true;
                operation_successful_ = false;
                RCLCPP_WARN(this->get_logger(), "State machine reached ERROR state - operation failed");
            }
        }
    }

    bool PickAndPlaceServiceNode::isValidPartName(const std::string& part_name)
    {
        return part_poses_.find(part_name) != part_poses_.end();
    }

    int PickAndPlaceServiceNode::getPartId(const std::string& part_name)
    {
        if (part_name == "spindle_2") return SPINDLE_2;
        if (part_name == "pinion_gear") return PINION_GEAR;
        if (part_name == "idler_gear") return IDLER_GEAR;
        if (part_name == "cover_plate") return COVER_PLATE;
        return -1;
    }

    std::map<std::string, geometry_msgs::msg::Pose> PickAndPlaceServiceNode::getPartPoses(const std::string& part_name)
    {
        if (part_poses_.find(part_name) != part_poses_.end()) {
            return part_poses_[part_name];
        }
        return {};
    }

    void PickAndPlaceServiceNode::sendPickAndPlaceCommand(int part_id, const std::map<std::string, geometry_msgs::msg::Pose>& poses)
    {
        auto command = xarm_msgs::msg::RobotStateAndTargetPose();
        command.robot_next_state = 1;

        // Set target poses based on part type
        if (poses.find("pick") != poses.end()) {
            command.target_pose_1 = poses.at("pick");
        }
        if (poses.find("place") != poses.end()) {
            command.target_pose_2 = poses.at("place");
        }

        state_topic_publisher_->publish(command);
        RCLCPP_INFO(this->get_logger(), "Sent pick and place command for part ID: %d", part_id);
    }

    bool PickAndPlaceServiceNode::waitForCompletion(const std::chrono::seconds& timeout)
    {
        auto start_time = std::chrono::steady_clock::now();
        
        while (rclcpp::ok()) {
            // Check timeout
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout) {
                return false;
            }

            // Check if operation completed
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (operation_completed_) {
                    return true;
                }
            }

            // Sleep briefly to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(shared_from_this());
        }
        
        return false;
    }

    void PickAndPlaceServiceNode::initializePoses()
    {
        // Initialize spindle_2 poses (from experiment_002.cpp)
        geometry_msgs::msg::Pose spindle_pick_pose;
        spindle_pick_pose.position.x = 0.3285;
        spindle_pick_pose.position.y = -0.08823;
        spindle_pick_pose.position.z = 0.1261;
        spindle_pick_pose.orientation.x = 1.0;
        spindle_pick_pose.orientation.y = 0.0;
        spindle_pick_pose.orientation.z = 0.0;
        spindle_pick_pose.orientation.w = 0.0;

        geometry_msgs::msg::Pose spindle_place_pose;
        spindle_place_pose.position.x = 0.3665;
        spindle_place_pose.position.y = -0.3788;
        spindle_place_pose.position.z = 0.15;
        spindle_place_pose.orientation.x = 1.0;
        spindle_place_pose.orientation.y = 0.0;
        spindle_place_pose.orientation.z = 0.0;
        spindle_place_pose.orientation.w = 0.0;

        part_poses_["spindle_2"]["pick"] = spindle_pick_pose;
        part_poses_["spindle_2"]["place"] = spindle_place_pose;

        // Initialize pinion_gear poses
        geometry_msgs::msg::Pose pinion_pick_pose;
        pinion_pick_pose.position.x = 0.4912;
        pinion_pick_pose.position.y = 0.0661;
        pinion_pick_pose.position.z = 0.0650;
        pinion_pick_pose.orientation.x = 1.0;
        pinion_pick_pose.orientation.y = 0.0;
        pinion_pick_pose.orientation.z = 0.0;
        pinion_pick_pose.orientation.w = 0.0;

        geometry_msgs::msg::Pose pinion_place_pose;
        pinion_place_pose.position.x = 0.4612;
        pinion_place_pose.position.y = -0.4039;
        pinion_place_pose.position.z = 0.15;
        pinion_place_pose.orientation.x = 1.0;
        pinion_place_pose.orientation.y = 0.0;
        pinion_place_pose.orientation.z = 0.0;
        pinion_place_pose.orientation.w = 0.0;

        part_poses_["pinion_gear"]["pick"] = pinion_pick_pose;
        part_poses_["pinion_gear"]["place"] = pinion_place_pose;

        // Initialize idler_gear poses
        geometry_msgs::msg::Pose idler_pick_pose;
        idler_pick_pose.position.x = 0.3357;
        idler_pick_pose.position.y = 0.0295;
        idler_pick_pose.position.z = 0.0558;
        idler_pick_pose.orientation.x = 1.0;
        idler_pick_pose.orientation.y = 0.0;
        idler_pick_pose.orientation.z = 0.0;
        idler_pick_pose.orientation.w = 0.0;

        geometry_msgs::msg::Pose idler_place_pose;
        idler_place_pose.position.x = 0.4506;
        idler_place_pose.position.y = -0.3707;
        idler_place_pose.position.z = 0.1407;
        idler_place_pose.orientation.x = 1.0;
        idler_place_pose.orientation.y = 0.0;
        idler_place_pose.orientation.z = 0.0;
        idler_place_pose.orientation.w = 0.0;

        part_poses_["idler_gear"]["pick"] = idler_pick_pose;
        part_poses_["idler_gear"]["place"] = idler_place_pose;

        // Initialize cover_plate poses (only pick for cover)
        geometry_msgs::msg::Pose cover_pick_pose;
        cover_pick_pose.position.x = 0.5052;
        cover_pick_pose.position.y = -0.08395;
        cover_pick_pose.position.z = 0.06;
        cover_pick_pose.orientation.x = 1.0;
        cover_pick_pose.orientation.y = 0.0;
        cover_pick_pose.orientation.z = 0.0;
        cover_pick_pose.orientation.w = 0.0;

        part_poses_["cover_plate"]["pick"] = cover_pick_pose;
        // Note: cover_plate only has pick operation, no place operation

        RCLCPP_INFO(this->get_logger(), "Part poses initialized for all supported parts");
    }

} // namespace pick_and_place_service

RCLCPP_COMPONENTS_REGISTER_NODE(pick_and_place_service::PickAndPlaceServiceNode)
