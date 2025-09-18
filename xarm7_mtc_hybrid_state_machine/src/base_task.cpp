/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/base_task.hpp"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/properties.h>

namespace xarm7_mtc
{
    namespace utils
    {
        // Pre-defined approach and retreat vectors
        const geometry_msgs::msg::Vector3 APPROACH_VECTOR = createVector3(0.0, 0.0, -1.0);
        const geometry_msgs::msg::Vector3 RETREAT_VECTOR = createVector3(0.0, 0.0, -1.0);
        const double APPROACH_DISTANCE = 0.05;  // 5cm
        const double RETREAT_DISTANCE = 0.05;   // 5cm
        const double APPROACH_HEIGHT = 0.1;     // 10cm
        const double SPINDLE_APPROACH_HEIGHT = -0.075;     // 7.5cm
        const double SPINDLE_RETREAT_HEIGHT = 0.075;
        const double SPINDLE_PLACE_APPROACH_HEIGHT = -0.08;  // 8cm for placing
        const double PINION_APPROACH_HEIGHT = 0.1;     // 10cm
        const double IDLER_APPROACH_HEIGHT = 0.1;     // 10cm
        const double COVER_APPROACH_HEIGHT = 0.1;     // 10cm

        geometry_msgs::msg::Pose createPose(double x, double y, double z, 
                                           double qx, double qy, double qz, double qw)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = qx;
            pose.orientation.y = qy;
            pose.orientation.z = qz;
            pose.orientation.w = qw;
            return pose;
        }

        geometry_msgs::msg::PoseStamped createPoseStamped(const geometry_msgs::msg::Pose& pose,
                                                         const std::string& frame_id)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header.frame_id = frame_id;
            pose_stamped.header.stamp = rclcpp::Clock().now();
            return pose_stamped;
        }

        geometry_msgs::msg::Vector3 createVector3(double x, double y, double z)
        {
            geometry_msgs::msg::Vector3 vector;
            vector.x = x;
            vector.y = y;
            vector.z = z;
            return vector;
        }
    } // namespace utils

    BaseTask::BaseTask(const std::string& task_name, rclcpp::Node::SharedPtr node)
        : node_(node)
        , task_name_(task_name)
        , arm_group_name_("xarm7")
        , gripper_group_name_("xarm_gripper")
        , base_frame_("link_base")
        , tool_frame_("link_tcp")
        , has_pick_pose_(false)
        , has_place_pose_(false)
        , current_state_(TaskState::IDLE)
        , last_error_("")
    {
        RCLCPP_INFO(node_->get_logger(), "BaseTask '%s' initialized", task_name_.c_str());
    }

    bool BaseTask::init()
    {
        try {
            // Reset all state
            reset();
            
            // Validate configuration
            if (!validateGroupNames()) {
                setError("Invalid group names configuration");
                return false;
            }

            setState(TaskState::IDLE);
            RCLCPP_INFO(node_->get_logger(), "BaseTask '%s' initialization completed", task_name_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            setError("Exception during initialization: " + std::string(e.what()));
            return false;
        }
    }

    bool BaseTask::reset()
    {
        try {
            // Reset task object
            task_.reset();
            
            // Reset state tracking
            stage_info_.clear();
            current_state_ = TaskState::IDLE;
            last_error_.clear();
            
            // Reset timing
            start_time_ = std::chrono::steady_clock::time_point{};
            end_time_ = std::chrono::steady_clock::time_point{};
            
            RCLCPP_DEBUG(node_->get_logger(), "BaseTask '%s' reset completed", task_name_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            setError("Exception during reset: " + std::string(e.what()));
            return false;
        }
    }

    std::vector<StageInfo> BaseTask::getStageInfo() const
    {
        return stage_info_;
    }

    TaskState BaseTask::getCurrentState() const
    {
        return current_state_;
    }

    std::string BaseTask::getLastError() const
    {
        return last_error_;
    }

    void BaseTask::setPickPose(const geometry_msgs::msg::Pose& pose)
    {
        pick_pose_ = pose;
        has_pick_pose_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "Pick pose set for task '%s'", task_name_.c_str());
    }

    void BaseTask::setPlacePose(const geometry_msgs::msg::Pose& pose)
    {
        place_pose_ = pose;
        has_place_pose_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "Place pose set for task '%s'", task_name_.c_str());
    }

    geometry_msgs::msg::Pose BaseTask::getPickPose() const
    {
        return pick_pose_;
    }

    geometry_msgs::msg::Pose BaseTask::getPlacePose() const
    {
        return place_pose_;
    }

    std::string BaseTask::getTaskName() const
    {
        return task_name_;
    }

    double BaseTask::getTotalExecutionTime() const
    {
        if (start_time_.time_since_epoch().count() == 0) {
            return 0.0;
        }
        
        auto end = (end_time_.time_since_epoch().count() == 0) ? 
                   std::chrono::steady_clock::now() : end_time_;
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time_);
        return duration.count() / 1000.0;
    }

    void BaseTask::updateStageInfo(const std::string& stage_name, TaskState state, 
                                  const std::string& error_msg)
    {
        StageInfo info;
        info.stage_name = stage_name;
        info.state = state;
        info.error_message = error_msg;
        info.execution_time = getTotalExecutionTime();
        
        // Update existing stage or add new one
        bool found = false;
        for (auto& existing : stage_info_) {
            if (existing.stage_name == stage_name) {
                existing = info;
                found = true;
                break;
            }
        }
        
        if (!found) {
            stage_info_.push_back(info);
        }
        
        RCLCPP_DEBUG(node_->get_logger(), "Stage '%s' updated: state=%d, error='%s'",
                    stage_name.c_str(), static_cast<int>(state), error_msg.c_str());
    }

    void BaseTask::setState(TaskState state)
    {
        if (current_state_ != state) {
            RCLCPP_DEBUG(node_->get_logger(), "Task '%s' state change: %d -> %d",
                        task_name_.c_str(), static_cast<int>(current_state_), static_cast<int>(state));
            current_state_ = state;
            
            // Update timing
            if (state == TaskState::PLANNING || state == TaskState::EXECUTING) {
                if (start_time_.time_since_epoch().count() == 0) {
                    start_time_ = std::chrono::steady_clock::now();
                }
            } else if (state == TaskState::COMPLETED || state == TaskState::FAILED) {
                end_time_ = std::chrono::steady_clock::now();
            }
        }
    }

    void BaseTask::setError(const std::string& error_message)
    {
        last_error_ = error_message;
        setState(TaskState::FAILED);
        RCLCPP_ERROR(node_->get_logger(), "Task '%s' error: %s", task_name_.c_str(), error_message.c_str());
    }

    moveit::task_constructor::stages::CurrentState* BaseTask::createCurrentState(const std::string& name)
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::CurrentState>(name);
        
        // Configure current state stage
        stage->setTimeout(10.0);  // 10 second timeout
        
        updateStageInfo(name, TaskState::PLANNING);
        return stage.release();
    }

    moveit::task_constructor::stages::MoveTo* BaseTask::createMoveTo(const std::string& name, 
                                                                   const geometry_msgs::msg::Pose& pose)
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(name, 
            std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>());
        
        // Configure MoveTo stage
        stage->setGroup(arm_group_name_);
        stage->setTimeout(30.0);  // 30 second timeout
        stage->setGoal(utils::createPoseStamped(pose, base_frame_));
        
        updateStageInfo(name, TaskState::PLANNING);
        return stage.release();
    }

    moveit::task_constructor::stages::MoveRelative* BaseTask::createMoveRelative(const std::string& name,
                                                                               const geometry_msgs::msg::Vector3& direction,
                                                                               double distance)
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(name,
            std::make_shared<moveit::task_constructor::solvers::CartesianPath>());
        
        // Configure MoveRelative stage
        stage->setGroup(arm_group_name_);
        stage->setTimeout(20.0);  // 20 second timeout
        
        // Set relative motion
        geometry_msgs::msg::Vector3Stamped direction_stamped;
        direction_stamped.header.frame_id = tool_frame_;
        direction_stamped.vector = direction;
        
        // Scale direction by distance
        direction_stamped.vector.x *= distance;
        direction_stamped.vector.y *= distance;
        direction_stamped.vector.z *= distance;
        
        stage->setDirection(direction_stamped);
        
        updateStageInfo(name, TaskState::PLANNING);
        return stage.release();
    }

    moveit::task_constructor::stages::MoveTo* BaseTask::createOpenGripper(const std::string& name)
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(name,
            std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>());
        
        // Configure gripper opening stage
        stage->setGroup(gripper_group_name_);
        stage->setTimeout(10.0);  // 10 second timeout
        
        // Set goal to open position (according to SRDF: open = 0.0)
        std::map<std::string, double> gripper_open_values;
        gripper_open_values["drive_joint"] = 0.0;  // Open gripper position for xArm gripper
        stage->setGoal(gripper_open_values);
        
        updateStageInfo(name, TaskState::PLANNING);
        return stage.release();
    }

    moveit::task_constructor::stages::MoveTo* BaseTask::createCloseGripper(const std::string& name)
    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(name,
            std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>());
        
        // Configure gripper closing stage  
        stage->setGroup(gripper_group_name_);
        stage->setTimeout(10.0);  // 10 second timeout
        
        // Set goal to close position (according to SRDF: close = 0.85)
        std::map<std::string, double> gripper_close_values;
        gripper_close_values["drive_joint"] = 0.85;  // Closed gripper position for xArm gripper
        stage->setGoal(gripper_close_values);
        
        updateStageInfo(name, TaskState::PLANNING);
        return stage.release();
    }

    void BaseTask::setupCommonProperties()
    {
        if (!task_) {
            setError("Cannot setup properties: task not created");
            return;
        }
        
        // Load robot model from robot_description parameter
        if (!loadRobotModel()) {
            setError("Failed to load robot model");
            return;
        }
        
        // Set planning scene
        task_->stages()->setName(task_name_);
        
        // Set common task properties
        task_->setProperty("group", arm_group_name_);
        task_->setProperty("eef", gripper_group_name_);
        task_->setProperty("ik_frame", tool_frame_);
        
        // Set trajectory execution info with controller names
        // This is critical for task execution to work properly
        std::vector<std::string> controller_names = {"xarm7_traj_controller", "xarm_gripper_traj_controller"};
        task_->setProperty("trajectory_execution_info", 
                          moveit::task_constructor::TrajectoryExecutionInfo().set__controller_names(controller_names));
        
        RCLCPP_INFO(node_->get_logger(), "Common properties set for task '%s' with controllers: [%s, %s]", 
                   task_name_.c_str(), controller_names[0].c_str(), controller_names[1].c_str());
    }

    bool BaseTask::loadRobotModel()
    {
        try {
            // Load robot model using the standard robot_description parameter
            task_->loadRobotModel(node_);
            
            RCLCPP_INFO(node_->get_logger(), "Robot model loaded successfully for task '%s'", task_name_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model for task '%s': %s", 
                        task_name_.c_str(), e.what());
            return false;
        }
    }

    bool BaseTask::validatePoses() const
    {
        if (!has_pick_pose_) {
            RCLCPP_ERROR(node_->get_logger(), "Task '%s': Pick pose not set", task_name_.c_str());
            return false;
        }
        
        // Additional pose validation can be added here
        // Check for reasonable pose values, workspace limits, etc.
        
        return true;
    }

    bool BaseTask::validateGroupNames() const
    {
        if (arm_group_name_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Task '%s': Arm group name not set", task_name_.c_str());
            return false;
        }
        
        if (gripper_group_name_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Task '%s': Gripper group name not set", task_name_.c_str());
            return false;
        }
        
        return true;
    }

} // namespace xarm7_mtc