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
     * @brief MTC Task for idler_gear pick-and-place operation
     * 
     * Similar structure to other gear tasks but with idler-specific poses and constraints
     */
    class IdlerTask : public BaseTask
    {
    public:
        explicit IdlerTask(rclcpp::Node::SharedPtr node);
        ~IdlerTask() override = default;

        bool createTask() override;
        bool planTask() override; 
        bool executeTask() override;

        void setIdlerSpecificPoses();
        
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
        
        static constexpr const char* OBJECT_NAME = "idler_gear";
        static constexpr double OBJECT_HEIGHT = 0.02;
        static constexpr double OBJECT_RADIUS = 0.018;

        void createPickingStages();
        void createPlacingStages();
        void createReturnToHomeStage();
        
        geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target_pose);
        bool validateIdlerPoses() const;
        
        void addIdlerCollisionObject();
        void removeIdlerCollisionObject();
        void updateIdlerCollisionObject();
    };

} // namespace xarm7_mtc