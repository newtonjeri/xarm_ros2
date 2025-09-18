/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/cover_task.hpp"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace xarm7_mtc
{
    CoverTask::CoverTask(rclcpp::Node::SharedPtr node)
        : BaseTask("cover_task", node)
    {
        setCoverSpecificPoses();
        RCLCPP_INFO(node_->get_logger(), "CoverTask initialized with default poses");
    }

    bool CoverTask::createTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("create_task", TaskState::PLANNING);

            // Create new task
            task_ = std::make_unique<moveit::task_constructor::Task>("cover_pick_task");
            
            if (!validateCoverPoses()) {
                setError("Invalid cover poses");
                return false;
            }

            // Setup common properties
            setupCommonProperties();

            // Add collision object for cover
            // addCoverCollisionObject();

            // Create task stages (pick-only operation)
            createPickingStages();

            updateStageInfo("create_task", TaskState::COMPLETED);
            RCLCPP_INFO(node_->get_logger(), "CoverTask created successfully with %zu stages", 
                       task_->stages()->numChildren());
            return true;
        }
        catch (const std::exception& e) {
            setError("Failed to create cover task: " + std::string(e.what()));
            return false;
        }
    }

    bool CoverTask::planTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("plan_task", TaskState::PLANNING);

            if (!task_) {
                setError("Cannot plan: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Planning cover task...");
            
            // Plan the task
            auto result = task_->plan();
            bool success = (result.val == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                updateStageInfo("plan_task", TaskState::COMPLETED);
                RCLCPP_INFO(node_->get_logger(), "Cover task planning successful");
                setState(TaskState::IDLE); // Ready for execution
                return true;
            } else {
                setError("Cover task planning failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during cover task planning: " + std::string(e.what()));
            return false;
        }
    }

    bool CoverTask::executeTask()
    {
        try {
            setState(TaskState::EXECUTING);
            updateStageInfo("execute_task", TaskState::EXECUTING);

            if (!task_) {
                setError("Cannot execute: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Executing cover task...");
            
            // Execute the planned task using first solution
            if (task_->numSolutions() == 0) {
                setError("No solutions available for execution");
                return false;
            }
            
            auto solution = task_->solutions().front();
            auto result = task_->execute(*solution);
            bool success = (result.val == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                updateStageInfo("execute_task", TaskState::COMPLETED);
                setState(TaskState::COMPLETED);
                RCLCPP_INFO(node_->get_logger(), "Cover task execution successful");
                
                // Update collision object after successful pick
                // updateCoverCollisionObject();
                return true;
            } else {
                setError("Cover task execution failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during cover task execution: " + std::string(e.what()));
            return false;
        }
    }

    void CoverTask::setCoverSpecificPoses()
    {
        // Set default poses from previous implementation
        setPickPose(getDefaultPickPose());
        // Note: Cover task is pick-only, so no place pose set
        
        RCLCPP_DEBUG(node_->get_logger(), "Cover-specific poses configured");
    }

    geometry_msgs::msg::Pose CoverTask::getDefaultPickPose()
    {
        return utils::createPose(0.5052, -0.08395, 0.0165, 1.0, 0.0, 0.0, 0.0);
    }

    geometry_msgs::msg::Pose CoverTask::getApproachPickPose()
    {
        auto pose = getDefaultPickPose();
        pose.position.z += APPROACH_HEIGHT;
        return pose;
    }

    geometry_msgs::msg::Pose CoverTask::getHoldPosition()
    {
        auto pose = getDefaultPickPose();
        pose.position.z = HOLD_HEIGHT;
        return pose;
    }

    geometry_msgs::msg::Pose CoverTask::getHomePose()
    {
        return utils::createPose(0.4912, 0.0661, 0.3, 1.0, 0.0, 0.0, 0.0);
    }

    void CoverTask::createPickingStages()
    {
        updateStageInfo("picking_stages", TaskState::PLANNING);

        // Stage 1: Current State
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCurrentState("current_state")));

        // Stage 2: Move to approach pick position
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("approach_pick", getApproachPickPose())));

        // Stage 3: Open gripper (prepare to pick)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createOpenGripper("open_gripper_for_pick")));

        // Stage 4: Descend to pick position (based on experiment_002: -0.0435)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("descend_to_pick", utils::APPROACH_VECTOR, -0.0435)));

        // Stage 5: Allow collisions before closing gripper
        auto allow_collision_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow_collisions_for_grasp");
        allow_collision_stage->allowCollisions(OBJECT_NAME, "right_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "left_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "base_plate", true);
        task_->add(std::move(allow_collision_stage));

        // Stage 6: Close gripper (pick object)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCloseGripper("close_gripper_pick_object")));
        
        // Stage 7: Attach object to gripper
        auto pick_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("pick_object");
        pick_stage->attachObject(OBJECT_NAME, tool_frame_);
        task_->add(std::move(pick_stage));

        // Stage 8: Ascend with object (based on experiment_002: +0.19)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("ascend_with_object", utils::RETREAT_VECTOR, 0.19)));

        updateStageInfo("picking_stages", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Picking stages created for cover task");
    }

    geometry_msgs::msg::Pose CoverTask::calculateApproachPose(const geometry_msgs::msg::Pose& target_pose)
    {
        auto approach_pose = target_pose;
        approach_pose.position.z += APPROACH_HEIGHT;
        return approach_pose;
    }

    geometry_msgs::msg::Pose CoverTask::calculateHoldPose(const geometry_msgs::msg::Pose& pick_pose)
    {
        auto hold_pose = pick_pose;
        hold_pose.position.z = HOLD_HEIGHT;
        return hold_pose;
    }

    bool CoverTask::validateCoverPoses() const
    {
        if (!validatePoses()) {
            return false;
        }

        // Additional cover-specific validation
        const auto& pick_pose = getPickPose();

        // Check workspace limits
        if (pick_pose.position.x < 0.3 || pick_pose.position.x > 0.8) {
            RCLCPP_ERROR(node_->get_logger(), "Cover pick pose X out of workspace limits");
            return false;
        }

        if (pick_pose.position.y < -0.5 || pick_pose.position.y > 0.5) {
            RCLCPP_ERROR(node_->get_logger(), "Cover pick pose Y out of workspace limits");
            return false;
        }

        return true;
    }

    void CoverTask::addCoverCollisionObject()
    {
        try {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = base_frame_;
            collision_object.id = OBJECT_NAME;

            // Define cover as box
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = OBJECT_LENGTH;  // length
            primitive.dimensions[1] = OBJECT_WIDTH;   // width
            primitive.dimensions[2] = OBJECT_HEIGHT;  // height

            collision_object.primitives.push_back(primitive);

            // Set pose at pick location
            geometry_msgs::msg::Pose object_pose = getPickPose();
            collision_object.primitive_poses.push_back(object_pose);

            collision_object.operation = collision_object.ADD;

            // Add to planning scene
            psi_.applyCollisionObject(collision_object);

            RCLCPP_DEBUG(node_->get_logger(), "Cover collision object added to planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to add cover collision object: %s", e.what());
        }
    }

    void CoverTask::removeCoverCollisionObject()
    {
        try {
            psi_.removeCollisionObjects({OBJECT_NAME});
            RCLCPP_DEBUG(node_->get_logger(), "Cover collision object removed from planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to remove cover collision object: %s", e.what());
        }
    }

    void CoverTask::updateCoverCollisionObject()
    {
        try {
            // For cover task, object is just picked and held
            // No need to move collision object to different location
            RCLCPP_DEBUG(node_->get_logger(), "Cover held in gripper - collision object attached");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to update cover collision object: %s", e.what());
        }
    }

} // namespace xarm7_mtc