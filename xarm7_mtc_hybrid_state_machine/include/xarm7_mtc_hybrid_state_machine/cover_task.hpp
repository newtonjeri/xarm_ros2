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
     * @brief MTC Task for cover_plate pick-only operation
     * 
     * This task implements a pick-only sequence for the cover_plate:
     * 1. CurrentState - Get initial robot state
     * 2. MoveTo(approach_pick) - Move to approach position above pick location  
     * 3. MoveRelative(descend) - Descend to pick height
     * 4. Pick(gripper_close) - Close gripper to grasp object
     * 5. MoveRelative(ascend) - Lift object from pick location
     * 6. MoveTo(hold_position) - Move to safe holding position
     * 
     * Note: This is a pick-only operation - no place sequence
     */
    class CoverTask : public BaseTask
    {
    public:
        explicit CoverTask(rclcpp::Node::SharedPtr node);
        ~CoverTask() override = default;

        bool createTask() override;
        bool planTask() override; 
        bool executeTask() override;

        void setCoverSpecificPoses();
        
        static geometry_msgs::msg::Pose getDefaultPickPose();
        static geometry_msgs::msg::Pose getApproachPickPose();
        static geometry_msgs::msg::Pose getHoldPosition();
        static geometry_msgs::msg::Pose getHomePose();

    private:
        static constexpr double PICK_HEIGHT_OFFSET = 0.05;
        static constexpr double APPROACH_HEIGHT = 0.0435;
        static constexpr double GRIPPER_CLOSE_DISTANCE = 0.02;
        static constexpr double HOLD_HEIGHT = 0.15;        // Height for holding position
        
        static constexpr const char* OBJECT_NAME = "cover_plate";
        static constexpr double OBJECT_HEIGHT = 0.01;      // Thin cover plate
        static constexpr double OBJECT_LENGTH = 0.08;      // Cover plate dimensions
        static constexpr double OBJECT_WIDTH = 0.06;

        // Cover-specific stages (pick-only operation)
        void createPickingStages();
        void createHoldingStage();
        void createReturnToHomeStage();
        
        geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target_pose);
        geometry_msgs::msg::Pose calculateHoldPose(const geometry_msgs::msg::Pose& pick_pose);
        
        bool validateCoverPoses() const;
        
        void addCoverCollisionObject();
        void removeCoverCollisionObject();
        void updateCoverCollisionObject();
    };

} // namespace xarm7_mtc