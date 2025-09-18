/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#pragma once

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/properties.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace xarm7_mtc
{
    // Task execution states for stage tracking
    enum class TaskState
    {
        IDLE = 0,
        PLANNING = 1,
        EXECUTING = 2,
        COMPLETED = 3,
        FAILED = 4
    };

    // Stage tracking information
    struct StageInfo
    {
        std::string stage_name;
        TaskState state;
        std::string error_message;
        double execution_time;
    };

    /**
     * @brief Base class for all xarm7 MTC tasks
     * 
     * Provides common functionality for pick-and-place operations including:
     * - Stage tracking and monitoring
     * - Common pose definitions and utilities
     * - Error handling and reporting
     * - Planning scene integration
     */
    class BaseTask
    {
    public:
        BaseTask(const std::string& task_name, rclcpp::Node::SharedPtr node);
        virtual ~BaseTask() = default;

        // Pure virtual methods that derived classes must implement
        virtual bool createTask() = 0;
        virtual bool planTask() = 0;
        virtual bool executeTask() = 0;
        
        // Common interface methods
        bool init();
        bool reset();
        
        // Stage tracking methods
        std::vector<StageInfo> getStageInfo() const;
        TaskState getCurrentState() const;
        std::string getLastError() const;
        
        // Pose management
        void setPickPose(const geometry_msgs::msg::Pose& pose);
        void setPlacePose(const geometry_msgs::msg::Pose& pose);
        geometry_msgs::msg::Pose getPickPose() const;
        geometry_msgs::msg::Pose getPlacePose() const;
        
        // Task management
        std::string getTaskName() const;
        double getTotalExecutionTime() const;

    protected:
        // MTC task object
        std::unique_ptr<moveit::task_constructor::Task> task_;
        
        // Node reference
        rclcpp::Node::SharedPtr node_;
        
        // Task configuration
        std::string task_name_;
        std::string arm_group_name_;
        std::string gripper_group_name_;
        std::string base_frame_;
        std::string tool_frame_;
        
        // Poses
        geometry_msgs::msg::Pose pick_pose_;
        geometry_msgs::msg::Pose place_pose_;
        bool has_pick_pose_;
        bool has_place_pose_;
        
        // Stage tracking
        std::vector<StageInfo> stage_info_;
        TaskState current_state_;
        std::string last_error_;
        
        // Timing
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::steady_clock::time_point end_time_;
        
        // Planning scene interface
        moveit::planning_interface::PlanningSceneInterface psi_;
        
        // Common utility methods
        void updateStageInfo(const std::string& stage_name, TaskState state, 
                           const std::string& error_msg = "");
        void setState(TaskState state);
        void setError(const std::string& error_message);
        
        // Common stage creation helpers
        moveit::task_constructor::stages::CurrentState* createCurrentState(const std::string& name);
        moveit::task_constructor::stages::MoveTo* createMoveTo(const std::string& name, 
                                                             const geometry_msgs::msg::Pose& pose);
        moveit::task_constructor::stages::MoveRelative* createMoveRelative(const std::string& name,
                                                                          const geometry_msgs::msg::Vector3& direction,
                                                                          double distance);
        
        // Gripper control helpers
        moveit::task_constructor::stages::MoveTo* createOpenGripper(const std::string& name);
        moveit::task_constructor::stages::MoveTo* createCloseGripper(const std::string& name);
        
        // Common properties setup
        void setupCommonProperties();
        bool loadRobotModel();
        
        // Validation helpers
        bool validatePoses() const;
        bool validateGroupNames() const;
    };

    /**
     * @brief Utility functions for pose manipulation
     */
    namespace utils
    {
        geometry_msgs::msg::Pose createPose(double x, double y, double z, 
                                           double qx, double qy, double qz, double qw);
        geometry_msgs::msg::PoseStamped createPoseStamped(const geometry_msgs::msg::Pose& pose,
                                                         const std::string& frame_id);
        geometry_msgs::msg::Vector3 createVector3(double x, double y, double z);
        
        // Pre-defined approach and retreat vectors
        extern const geometry_msgs::msg::Vector3 APPROACH_VECTOR;
        extern const geometry_msgs::msg::Vector3 RETREAT_VECTOR;
        extern const double SPINDLE_APPROACH_HEIGHT;
        extern const double SPINDLE_RETREAT_HEIGHT;
        extern const double SPINDLE_PLACE_APPROACH_HEIGHT;
        extern const double PINION_APPROACH_HEIGHT;
        extern const double IDLER_APPROACH_HEIGHT;
        extern const double COVER_APPROACH_HEIGHT;
        extern const double APPROACH_DISTANCE;
        extern const double RETREAT_DISTANCE;
        extern const double APPROACH_HEIGHT;
    }

} // namespace xarm7_mtc