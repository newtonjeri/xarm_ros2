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
          operation_successful_(false),
          operation_completed_(false)
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
        RCLCPP_INFO(this->get_logger(), "Publishing commands to: /xarm7_state_topic");
        RCLCPP_INFO(this->get_logger(), "Monitoring state machine via: /xarm7_state_machine_state");
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
        RCLCPP_INFO(this->get_logger(), "=== NEW PICK AND PLACE REQUEST ===");
        RCLCPP_INFO(this->get_logger(), "Requested part: %s", request->part_name.c_str());

        // Check if operation is already in progress
        if (operation_in_progress_.load()) {
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

        // Reset and set operation flags atomically
        RCLCPP_INFO(this->get_logger(), "Starting operation for part: %s", request->part_name.c_str());
        operation_in_progress_.store(true);
        operation_successful_.store(false);
        
        // Reset completion state (protected by mutex)
        {
            std::lock_guard<std::mutex> lock(completion_mutex_);
            operation_completed_ = false;
        }

        // Send command to state machine
        sendPickAndPlaceCommand(part_id, poses);

        // Wait for completion with timeout (60 seconds)
        RCLCPP_INFO(this->get_logger(), "Waiting for operation completion (timeout: 60s)...");
        bool completed = waitForCompletion(std::chrono::seconds(60));

        // Reset operation flag atomically
        operation_in_progress_.store(false);

        // Process results
        if (completed && operation_successful_.load()) {
            response->success = true;
            response->message = "Pick and place operation for " + request->part_name + " completed successfully";
            RCLCPP_INFO(this->get_logger(), "=== OPERATION SUCCESS ===");
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            
            // Remove collision object after successful operation
            psi_.removeCollisionObjects({request->part_name});
        } else if (completed && !operation_successful_.load()) {
            response->success = false;
            response->message = "Pick and place operation for " + request->part_name + " failed";
            RCLCPP_ERROR(this->get_logger(), "=== OPERATION FAILED ===");
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "Pick and place operation for " + request->part_name + " timed out";
            RCLCPP_ERROR(this->get_logger(), "=== OPERATION TIMEOUT ===");
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Service ready for next request");
    }

    void PickAndPlaceServiceNode::stateMachineStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        uint8_t new_state = msg->data;
        uint8_t previous_state = current_state_machine_state_.load();
        
        // Update current state
        current_state_machine_state_.store(new_state);
        
        // Log state changes for debugging
        if (new_state != previous_state) {
            const char* state_names[] = {"IDLE", "MOVING", "PICKING", "PLACING", "FINAL", "ERROR"};
            const char* current_name = (new_state < 6) ? state_names[new_state] : "UNKNOWN";
            const char* previous_name = (previous_state < 6) ? state_names[previous_state] : "UNKNOWN";
            
            RCLCPP_INFO(this->get_logger(), "State machine transition: %s (%d) -> %s (%d)", 
                       previous_name, previous_state, current_name, new_state);
        }

        // Check if operation completed (FINAL state = 4, ERROR state = 5)
        if (operation_in_progress_.load()) {
            if (new_state == 4) { // FINAL
                RCLCPP_INFO(this->get_logger(), "State machine reached FINAL state - operation successful");
                operation_successful_.store(true);
                
                // Notify waiting service callback
                {
                    std::lock_guard<std::mutex> lock(completion_mutex_);
                    operation_completed_ = true;
                }
                completion_cv_.notify_one();
                
            } else if (new_state == 5) { // ERROR
                RCLCPP_WARN(this->get_logger(), "State machine reached ERROR state - operation failed");
                operation_successful_.store(false);
                
                // Notify waiting service callback
                {
                    std::lock_guard<std::mutex> lock(completion_mutex_);
                    operation_completed_ = true;
                }
                completion_cv_.notify_one();
            }
            // For other states (IDLE, MOVING, PICKING, PLACING), continue waiting
        } else {
            // Log state changes even when no operation is in progress (for debugging)
            RCLCPP_DEBUG(this->get_logger(), "Received state update but no operation in progress");
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
        command.robot_next_state = 1; // MOVING state
        
        // Log the command details
        RCLCPP_INFO(this->get_logger(), "Preparing pick and place command for part ID: %d", part_id);

        // Set target poses based on part type
        if (poses.find("pick") != poses.end()) {
            command.target_pose_1 = poses.at("pick");
            RCLCPP_INFO(this->get_logger(), "Pick pose: [%.3f, %.3f, %.3f]", 
                       command.target_pose_1.position.x,
                       command.target_pose_1.position.y,
                       command.target_pose_1.position.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "No pick pose found for part!");
        }
        
        if (poses.find("place") != poses.end()) {
            command.target_pose_2 = poses.at("place");
            RCLCPP_INFO(this->get_logger(), "Place pose: [%.3f, %.3f, %.3f]", 
                       command.target_pose_2.position.x,
                       command.target_pose_2.position.y,
                       command.target_pose_2.position.z);
        } else {
            RCLCPP_INFO(this->get_logger(), "No place pose (pick-only operation)");
        }

        // Publish command
        state_topic_publisher_->publish(command);
        RCLCPP_INFO(this->get_logger(), "Command sent to state machine via /xarm7_state_topic");
        
        // Give a brief moment for the command to be processed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    bool PickAndPlaceServiceNode::waitForCompletion(const std::chrono::seconds& timeout)
    {
        // Much simpler approach: just wait for the state machine callback to notify us
        std::unique_lock<std::mutex> lock(completion_mutex_);
        
        auto start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Waiting for state machine to complete operation...");
        
        // Wait for completion or timeout
        bool completed = completion_cv_.wait_for(lock, timeout, [this] { 
            return operation_completed_; 
        });
        
        if (completed) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto completion_time = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            RCLCPP_INFO(this->get_logger(), "Operation completed in %ld ms", completion_time);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Operation timed out after %ld seconds", timeout.count());
            RCLCPP_WARN(this->get_logger(), "Final state was: %d", current_state_machine_state_.load());
            return false;
        }
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
