/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_mtc_hybrid_state_machine/spindle_task.hpp"
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
    SpindleTask::SpindleTask(rclcpp::Node::SharedPtr node)
        : BaseTask("spindle_task", node)
    {
        setSpindleSpecificPoses();
        RCLCPP_INFO(node_->get_logger(), "SpindleTask initialized with default poses");
    }

    bool SpindleTask::createTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("create_task", TaskState::PLANNING);

            // Create new task
            task_ = std::make_unique<moveit::task_constructor::Task>("spindle_pick_and_place");
            
            if (!validateSpindlePoses()) {
                setError("Invalid spindle poses");
                return false;
            }

            // Setup common properties
            setupCommonProperties();

            // Add collision object for spindle
            // addSpindleCollisionObject();

            // Create task stages
            createPickingStages();
            createPlacingStages();
            createReturnToHomeStage();

            updateStageInfo("create_task", TaskState::COMPLETED);
            RCLCPP_INFO(node_->get_logger(), "SpindleTask created successfully with %zu stages", 
                       task_->stages()->numChildren());
            return true;
        }
        catch (const std::exception& e) {
            setError("Failed to create spindle task: " + std::string(e.what()));
            return false;
        }
    }

    bool SpindleTask::planTask()
    {
        try {
            setState(TaskState::PLANNING);
            updateStageInfo("plan_task", TaskState::PLANNING);

            if (!task_) {
                setError("Cannot plan: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Planning spindle task...");
            
            // Plan the task
            auto result = task_->plan();
            bool success = (result.val == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                updateStageInfo("plan_task", TaskState::COMPLETED);
                RCLCPP_INFO(node_->get_logger(), "Spindle task planning successful");
                setState(TaskState::IDLE); // Ready for execution
                return true;
            } else {
                setError("Spindle task planning failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during spindle task planning: " + std::string(e.what()));
            return false;
        }
    }

    bool SpindleTask::executeTask()
    {
        try {
            setState(TaskState::EXECUTING);
            updateStageInfo("execute_task", TaskState::EXECUTING);

            if (!task_) {
                setError("Cannot execute: task not created");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Executing spindle task...");
            
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
                RCLCPP_INFO(node_->get_logger(), "Spindle task execution successful");
                
                // Update collision object after successful pick
                // updateSpindleCollisionObject();
                return true;
            } else {
                setError("Spindle task execution failed: " + std::to_string(result.val));
                return false;
            }
        }
        catch (const std::exception& e) {
            setError("Exception during spindle task execution: " + std::string(e.what()));
            return false;
        }
    }

    void SpindleTask::setSpindleSpecificPoses()
    {
        // Set default poses from previous implementation
        setPickPose(getDefaultPickPose());
        setPlacePose(getDefaultPlacePose());
        
        RCLCPP_DEBUG(node_->get_logger(), "Spindle-specific poses configured");
    }

    geometry_msgs::msg::Pose SpindleTask::getDefaultPickPose()
    {
        return utils::createPose(0.3285, -0.08823, 0.1261, 1.0, 0.0, 0.0, 0.0);
    }

    geometry_msgs::msg::Pose SpindleTask::getDefaultPlacePose()
    {
        return utils::createPose(0.3665, -0.3788, 0.15, 1.0, 0.0, 0.0, 0.0);
    }

    geometry_msgs::msg::Pose SpindleTask::getApproachPickPose()
    {
        auto pose = getDefaultPickPose();
        pose.position.z -= SPINDLE_APPROACH_HEIGHT;
        return pose;
    }

    geometry_msgs::msg::Pose SpindleTask::getApproachPlacePose()
    {
        // Return the place pose directly - no approach offset
        return getDefaultPlacePose();
    }

    geometry_msgs::msg::Pose SpindleTask::getHomePose()
    {

        // TODO: Update home pose using joint interpolation, already set up in the sdf as a group state, (hold up)
        return utils::createPose(0.4912, 0.0661, 0.3, 1.0, 0.0, 0.0, 0.0);
    }

    void SpindleTask::createPickingStages()
    {
        updateStageInfo("picking_stages", TaskState::PLANNING);

        // Stage 1: Current State
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCurrentState("current_state")));

        // Stage 2: Move to pick pose
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("move_to_pick_pose", getDefaultPickPose())));

        // Stage 3: Open gripper (prepare to pick)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createOpenGripper("open_gripper_for_pick")));

        // Stage 4: Move to approach pick position
        // task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("approach_pick", getApproachPickPose())));

        // Stage 5: Descend to pick position
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("descend_to_pick", utils::APPROACH_VECTOR, utils::SPINDLE_APPROACH_HEIGHT)));

        // Stage 6: Allow collisions before closing gripper
        auto allow_collision_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow_collisions_for_grasp");
        allow_collision_stage->allowCollisions(OBJECT_NAME, "right_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "left_finger", true);
        allow_collision_stage->allowCollisions(OBJECT_NAME, "base_plate", true);
        task_->add(std::move(allow_collision_stage));

        // Stage 7: Close gripper (pick object)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCloseGripper("close_gripper_pick_object")));
        
        // Stage 8: Attach object to gripper
        auto pick_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("pick_object");
        pick_stage->attachObject(OBJECT_NAME, tool_frame_);
        task_->add(std::move(pick_stage));

        // Stage 9: Ascend with object
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("ascend_with_object", utils::RETREAT_VECTOR, utils::SPINDLE_RETREAT_HEIGHT)));

        updateStageInfo("picking_stages", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Picking stages created for spindle task");
    }

    void SpindleTask::createPlacingStages()
    {
        updateStageInfo("placing_stages", TaskState::PLANNING);

        // Stage 10: Move to approach place position
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("approach_place", getApproachPlacePose())));

        // Stage 11: Descend to place position (using different approach height for placing)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("descend_to_place", utils::APPROACH_VECTOR, utils::SPINDLE_PLACE_APPROACH_HEIGHT)));

        // Stage 12: Open gripper (place object)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createOpenGripper("open_gripper_place_object")));
        
        // Stage 13: Detach object from gripper
        auto place_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("place_object");
        place_stage->detachObject(OBJECT_NAME, tool_frame_);
        // Disable collision allowances when placing object
        // place_stage->allowCollisions(OBJECT_NAME, "right_finger", false);
        // place_stage->allowCollisions(OBJECT_NAME, "left_finger", false);
        // place_stage->allowCollisions(OBJECT_NAME, "base_plate", false);
        task_->add(std::move(place_stage));

        // Stage 14: Ascend after placing (using place approach height to restore position)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveRelative("ascend_after_place", utils::RETREAT_VECTOR, -utils::SPINDLE_PLACE_APPROACH_HEIGHT)));

        // Stage 15: Close gripper (after placing)
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createCloseGripper("close_gripper_after_place")));

        updateStageInfo("placing_stages", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Placing stages created for spindle task");
    }

    void SpindleTask::createReturnToHomeStage()
    {
        updateStageInfo("return_home", TaskState::PLANNING);

        // Stage 16: Return to home position
        task_->add(std::unique_ptr<moveit::task_constructor::Stage>(createMoveTo("return_home", getHomePose())));

        updateStageInfo("return_home", TaskState::COMPLETED);
        RCLCPP_DEBUG(node_->get_logger(), "Return home stage created for spindle task");
    }

    geometry_msgs::msg::Pose SpindleTask::calculateApproachPose(const geometry_msgs::msg::Pose& target_pose)
    {
        auto approach_pose = target_pose;
        approach_pose.position.z += utils::SPINDLE_APPROACH_HEIGHT;
        return approach_pose;
    }

    bool SpindleTask::validateSpindlePoses() const
    {
        if (!validatePoses()) {
            return false;
        }

        // Additional spindle-specific validation
        const auto& pick_pose = getPickPose();
        const auto& place_pose = getPlacePose();

        // Check workspace limits (example values)
        if (pick_pose.position.x < 0.2 || pick_pose.position.x > 0.8) {
            RCLCPP_ERROR(node_->get_logger(), "Spindle pick pose X out of workspace limits");
            return false;
        }

        if (place_pose.position.x < 0.2 || place_pose.position.x > 0.8) {
            RCLCPP_ERROR(node_->get_logger(), "Spindle place pose X out of workspace limits");
            return false;
        }

        return true;
    }

    void SpindleTask::addSpindleCollisionObject()
    {
        try {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = base_frame_;
            collision_object.id = OBJECT_NAME;

            // Define spindle as cylinder
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

            RCLCPP_DEBUG(node_->get_logger(), "Spindle collision object added to planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to add spindle collision object: %s", e.what());
        }
    }

    void SpindleTask::removeSpindleCollisionObject()
    {
        try {
            psi_.removeCollisionObjects({OBJECT_NAME});
            RCLCPP_DEBUG(node_->get_logger(), "Spindle collision object removed from planning scene");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to remove spindle collision object: %s", e.what());
        }
    }

    void SpindleTask::updateSpindleCollisionObject()
    {
        try {
            // Remove from pick location
            removeSpindleCollisionObject();
            
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

            RCLCPP_DEBUG(node_->get_logger(), "Spindle collision object updated to place location");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to update spindle collision object: %s", e.what());
        }
    }

} // namespace xarm7_mtc