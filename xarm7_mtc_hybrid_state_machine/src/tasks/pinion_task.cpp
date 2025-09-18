/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/pinion_task.hpp"
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
    PinionTask::PinionTask(rclcpp::Node::SharedPtr node)
        : BaseTask("pinion_task", node)
    {
        setPinionSpecificPoses();
        RCLCPP_INFO(node_->get_logger(), "PinionTask initialized with default poses");
    }

    bool PinionTask::createTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("create_task", TaskState::PLANNING);

            // Create new task
            task_ = std::make_unique<moveit::task_constructor::Task>("pinion_pick_and_place");
            
            if (!validatePinionPoses()) {
                setError("Invalid pinion poses");
                return false;
            }

            // Setup common properties
            setupCommonProperties();

            // Add collision object for pinion
            // addPinionCollisionObject();

            // Create task stages
            createPickingStages();
            createPlacingStages();
            createReturnToHomeStage();

            updateStageInfo("create_task", TaskState::COMPLETED);
            RCLCPP_INFO(node_->get_logger(), "PinionTask created successfully with %zu stages", 
                       task_->stages()->numChildren());
            return true;
        }
        catch (const std::exception& e) {
            setError("Failed to create pinion task: " + std::string(e.what()));
            return false;
        }
    }

    bool PinionTask::planTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("plan_task", TaskState::PLANNING);

            if (!task_) {
                setError("Cannot plan: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Planning pinion task...");
            
            // Plan the task
            auto result = task_->plan();
            bool success = (result.val == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                updateStageInfo("plan_task", TaskState::COMPLETED);
                RCLCPP_INFO(node_->get_logger(), "Pinion task planning successful");
                setState(TaskState::IDLE); // Ready for execution
                return true;
            } else {
                setError("Pinion task planning failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during pinion task planning: " + std::string(e.what()));
            return false;
        }
    }

    bool PinionTask::executeTask()
    {
        try {
            setState(TaskState::EXECUTING);
            updateStageInfo("execute_task", TaskState::EXECUTING);

            if (!task_) {
                setError("Cannot execute: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Executing pinion task...");
            
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
                RCLCPP_INFO(node_->get_logger(), "Pinion task execution successful");
                
                // Update collision object after successful pick
                // updatePinionCollisionObject();
                return true;
            } else {
                setError("Pinion task execution failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during pinion task execution: " + std::string(e.what()));
            return false;
        }
    }

    void PinionTask::setPinionSpecificPoses()
    {
        // Set default poses from experiment_002.cpp
        setPickPose(getDefaultPickPose());
        setPlacePose(getDefaultPlacePose());
        
        RCLCPP_DEBUG(node_->get_logger(), "Pinion-specific poses configured");
    }

    geometry_msgs::msg::Pose PinionTask::getDefaultPickPose()
    {
        return utils::createPose(0.4912, 0.0661, 0.0650, 1.0, 0.0, 0.0, 0.0);
    }

    geometry_msgs::msg::Pose PinionTask::getDefaultPlacePose()
    {
        return utils::createPose(0.4612, -0.4039, 0.15, 1.0, 0.0, 0.0, 0.0);
    }

    geometry_msgs::msg::Pose PinionTask::getApproachPickPose()
    {
        auto pose = getDefaultPickPose();
        pose.position.z += APPROACH_HEIGHT;
        return pose;
    }

    geometry_msgs::msg::Pose PinionTask::getApproachPlacePose()
    {
        auto pose = getDefaultPlacePose();
        pose.position.z += APPROACH_HEIGHT;
        return pose;
    }

    geometry_msgs::msg::Pose PinionTask::getHomePose()
    {
        return utils::createPose(0.4912, 0.0661, 0.3, 1.0, 0.0, 0.0, 0.0);
    }

    void PinionTask::createPickingStages()
    {
        updateStageInfo("picking_stages", TaskState::PLANNING);

        // Stage 1: Current State
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCurrentState("current_state")));

        // Stage 2: Move to pick pose
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("move_to_pick_pose", getDefaultPickPose())));

        // Stage 3: Open gripper (prepare to pick)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createOpenGripper("open_gripper_for_pick")));

        // Stage 4: Descend to pick position (based on experiment_002: -0.055)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("descend_to_pick", utils::APPROACH_VECTOR, -0.055)));

        // Stage 5: Allow collisions before closing gripper (pinion needs to allow collision with spindle and base_plate)
        auto allow_collision_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow_collisions_for_grasp");
        allow_collision_stage->allowCollisions(OBJECT_NAME, "right_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "left_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "base_plate", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "spindle_2", true); // Allow collision with spindle
        task_->add(std::move(allow_collision_stage));

        // Stage 6: Close gripper (pick object)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCloseGripper("close_gripper_pick_object")));
        
        // Stage 7: Attach object to gripper
        auto pick_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("pick_object");
        pick_stage->attachObject(OBJECT_NAME, tool_frame_);
        task_->add(std::move(pick_stage));

        // Stage 8: Ascend with object (based on experiment_002: +0.055)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("ascend_with_object", utils::RETREAT_VECTOR, 0.055)));

        updateStageInfo("picking_stages", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Picking stages created for pinion task");
    }

    void PinionTask::createPlacingStages()
    {
        updateStageInfo("placing_stages", TaskState::PLANNING);

        // Stage 9: Move to place pose
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("move_to_place_pose", getDefaultPlacePose())));

        // Stage 10: Descend to place position (based on experiment_002: -0.08)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("descend_to_place", utils::APPROACH_VECTOR, -0.08)));

        // Stage 11: Open gripper (place object) - use halfway position for pinion
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createOpenGripper("open_gripper_place_object")));
        
        // Stage 12: Detach object from gripper
        auto place_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("place_object");
        place_stage->detachObject(OBJECT_NAME, tool_frame_);
        task_->add(std::move(place_stage));

        // Stage 13: Ascend after placing (based on experiment_002: +0.08)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("ascend_after_place", utils::RETREAT_VECTOR, 0.08)));

        updateStageInfo("placing_stages", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Placing stages created for pinion task");
    }

    void PinionTask::createReturnToHomeStage()
    {
        updateStageInfo("return_home", TaskState::PLANNING);

        // Stage 14: Return to home position
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("return_home", getHomePose())));

        updateStageInfo("return_home", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Return home stage created for pinion task");
    }

    geometry_msgs::msg::Pose PinionTask::calculateApproachPose(const geometry_msgs::msg::Pose& target_pose)
    {
        auto approach_pose = target_pose;
        approach_pose.position.z += APPROACH_HEIGHT;
        return approach_pose;
    }

    bool PinionTask::validatePinionPoses() const
    {
        if (!validatePoses()) {
            return false;
        }

        // Additional pinion-specific validation
        const auto& pick_pose = getPickPose();
        const auto& place_pose = getPlacePose();

        // Check workspace limits
        if (pick_pose.position.x < 0.2 || pick_pose.position.x > 0.8) {
            RCLCPP_ERROR(node_->get_logger(), "Pinion pick pose X out of workspace limits");
            return false;
        }

        if (place_pose.position.x < 0.2 || place_pose.position.x > 0.8) {
            RCLCPP_ERROR(node_->get_logger(), "Pinion place pose X out of workspace limits");
            return false;
        }

        return true;
    }

    void PinionTask::addPinionCollisionObject()
    {
        try {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = base_frame_;
            collision_object.id = OBJECT_NAME;

            // Define pinion as cylinder
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = OBJECT_HEIGHT;  // height
            primitive.dimensions[1] = OBJECT_RADIUS;  // radius

            collision_object.primitives.push_back(primitive);

            // Set pose at pick location
            geometry_msgs::msg::Pose object_pose = getPickPose();
            collision_object.primitive_poses.push_back(object_pose);

            collision_object.operation = collision_object.ADD;

            // Add to planning scene
            psi_.applyCollisionObject(collision_object);

            RCLCPP_DEBUG(node_->get_logger(), "Pinion collision object added to planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to add pinion collision object: %s", e.what());
        }
    }

    void PinionTask::removePinionCollisionObject()
    {
        try {
            psi_.removeCollisionObjects({OBJECT_NAME});
            RCLCPP_DEBUG(node_->get_logger(), "Pinion collision object removed from planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to remove pinion collision object: %s", e.what());
        }
    }

    void PinionTask::updatePinionCollisionObject()
    {
        try {
            // Remove from pick location
            removePinionCollisionObject();
            
            // Add at place location
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = base_frame_;
            collision_object.id = OBJECT_NAME;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = OBJECT_HEIGHT;
            primitive.dimensions[1] = OBJECT_RADIUS;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(getPlacePose());
            collision_object.operation = collision_object.ADD;

            psi_.applyCollisionObject(collision_object);

            RCLCPP_DEBUG(node_->get_logger(), "Pinion collision object updated to place location");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to update pinion collision object: %s", e.what());
        }
    }

} // namespace xarm7_mtc