/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#pragma once

#include "xarm7_mtc_hybrid_state_machine/base_task.hpp"

namespace xarm7_mtc
{
    /**
     * @brief MTC Task for spindle_2 pick-and-place operation
     * 
     * This task implements a complete pick-and-place sequence for the spindle_2 part:
     * 1. CurrentState - Get initial robot state
     * 2. MoveTo(approach_pick) - Move to approach position above pick location  
     * 3. MoveRelative(descend) - Descend to pick height
     * 4. Pick(gripper_close) - Close gripper to grasp object
     * 5. MoveRelative(ascend) - Lift object from pick location
     * 6. MoveTo(approach_place) - Move to approach position above place location
     * 7. MoveRelative(descend) - Descend to place height
     * 8. Place(gripper_open) - Open gripper to release object
     * 9. MoveRelative(ascend) - Retreat from place location
     * 10. MoveTo(home) - Return to safe home position
     */
    class SpindleTask : public BaseTask
    {
    public:
        explicit SpindleTask(rclcpp::Node::SharedPtr node);
        ~SpindleTask() override = default;

        // BaseTask interface implementation
        bool createTask() override;
        bool planTask() override; 
        bool executeTask() override;

        // Spindle-specific configuration
        void setSpindleSpecificPoses();
        
        // Pre-defined poses for spindle_2 operation
        static geometry_msgs::msg::Pose getDefaultPickPose();
        static geometry_msgs::msg::Pose getDefaultPlacePose();
        static geometry_msgs::msg::Pose getApproachPickPose();
        static geometry_msgs::msg::Pose getApproachPlacePose();
        static geometry_msgs::msg::Pose getHomePose();

    private:
        // Spindle-specific configuration
        static constexpr double PICK_HEIGHT_OFFSET = 0.05;  // 5cm above pick surface
        static constexpr double PLACE_HEIGHT_OFFSET = 0.05; // 5cm above place surface
        static constexpr double APPROACH_HEIGHT = 0.10;     // 10cm approach height
        static constexpr double SPINDLE_APPROACH_HEIGHT = -0.075; // 7.5cm approach height for spindle
        static constexpr double SPINDLE_RETREAT_HEIGHT = 0.075;  // 7.5cm retreat height for spindle
        static constexpr double GRIPPER_CLOSE_DISTANCE = 0.02; // Gripper closing distance
        


        // Object information
        static constexpr const char* OBJECT_NAME = "spindle_2";
        static constexpr double OBJECT_HEIGHT = 0.03;       // Object height for collision planning
        static constexpr double OBJECT_RADIUS = 0.015;      // Object radius for collision planning

        // Stage creation helpers specific to spindle task
        void createPickingStages();
        void createPlacingStages();
        void createReturnToHomeStage();
        
        // Pose calculation helpers
        geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target_pose);
        
        // Validation helpers
        bool validateSpindlePoses() const;
        
        // Collision object management
        void addSpindleCollisionObject();
        void removeSpindleCollisionObject();
        void updateSpindleCollisionObject();
    };

} // namespace xarm7_mtc