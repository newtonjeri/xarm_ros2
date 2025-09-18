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
     * @brief MTC Task for pinion_gear pick-and-place operation
     * 
     * Similar structure to SpindleTask but with pinion-specific poses and constraints
     */
    class PinionTask : public BaseTask
    {
    public:
        explicit PinionTask(rclcpp::Node::SharedPtr node);
        ~PinionTask() override = default;

        bool createTask() override;
        bool planTask() override; 
        bool executeTask() override;

        void setPinionSpecificPoses();
        
        static geometry_msgs::msg::Pose getDefaultPickPose();
        static geometry_msgs::msg::Pose getDefaultPlacePose();
        static geometry_msgs::msg::Pose getApproachPickPose();
        static geometry_msgs::msg::Pose getApproachPlacePose();
        static geometry_msgs::msg::Pose getHomePose();

    private:
        static constexpr double PICK_HEIGHT_OFFSET = 0.05;
        static constexpr double PLACE_HEIGHT_OFFSET = 0.05;
        static constexpr double APPROACH_HEIGHT = 0.10;
        static constexpr double GRIPPER_CLOSE_DISTANCE = 0.02;
        
        static constexpr const char* OBJECT_NAME = "pinion_gear";
        static constexpr double OBJECT_HEIGHT = 0.025;
        static constexpr double OBJECT_RADIUS = 0.02;

        void createPickingStages();
        void createPlacingStages();
        void createReturnToHomeStage();
        
        geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target_pose);
        bool validatePinionPoses() const;
        
        void addPinionCollisionObject();
        void removePinionCollisionObject();
        void updatePinionCollisionObject();
    };

} // namespace xarm7_mtc