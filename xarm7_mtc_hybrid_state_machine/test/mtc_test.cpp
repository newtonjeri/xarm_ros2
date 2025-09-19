/**
 * @file mtc_test.cpp
 * @brief Demonstrates the use of MoveIt Task Constructor for robot motion planning.
 *
 * This program creates a reusable task for a robot arm using MoveIt Task Constructor.
 * It defines a series of movements including Cartesian paths and joint space motions.
 *
 * Key Concept:
 *   SerialContainer: This is a type of container in MoveIt Task Constructor that holds
 *     multiple movement stages. These stages are executed in sequence, one after another.
 *     Think of it like a to-do list for the robot, where each item must be completed
 *     before moving on to the next one.
 *
 * @author Addison Sears-Collins
 * @date December 19, 2024
 */

// Include necessary headers
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/container.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <thread>
#include <chrono>

// Use the moveit::task_constructor namespace for convenience
using namespace moveit::task_constructor;

/**
 * @brief Creates a reusable module for robot movement.
 *
 * @param arm_group The name of the robot group to move.
 * @return std::unique_ptr<SerialContainer> A container with a series of movement stages.
 */
std::unique_ptr<SerialContainer> createModule(const std::string& arm_group) {
  // Create a new SerialContainer to hold our movement stages
  auto c = std::make_unique<SerialContainer>("Cartesian Path");
  c->setProperty("group", arm_group);

  RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Creating module for arm_group: %s", arm_group.c_str());

  // Create solvers for Cartesian and joint space planning
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  // Stage 1: Move 5 cm in the positive X direction
  // {
  //   auto stage = std::make_unique<stages::MoveRelative>("x +0.05", cartesian);
  //   stage->properties().configureInitFrom(Stage::PARENT, { "group" });
  //   geometry_msgs::msg::Vector3Stamped direction;
  //   direction.header.frame_id = "link_base";
  //   direction.vector.x = 0.05;
  //   stage->setDirection(direction);
  //   c->insert(std::move(stage));
  //   RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Added stage: Move 5 cm in +X direction");
  // }

   // Add a stage to move to the "Joint1" position
  {
    auto stage = std::make_unique<stages::MoveTo>("move to joint_1", joint_interpolation);
    stage->setGroup(arm_group);
    stage->setGoal("joint_1");
    c->insert(std::move(stage));
    RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Added stage: Move to 'joint_1' position");
  }

  // Stage 2: Move 2 cm in the negative Y direction
  // {
  //   auto stage = std::make_unique<stages::MoveRelative>("y -0.02", cartesian);
  //   stage->properties().configureInitFrom(Stage::PARENT);
  //   geometry_msgs::msg::Vector3Stamped direction;
  //   direction.header.frame_id = "link_base";
  //   direction.vector.y = -0.02;
  //   stage->setDirection(direction);
  //   c->insert(std::move(stage));
  //   RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Added stage: Move 2 cm in -Y direction");
  // }

  

//   // Stage 3: Rotate -18 degrees around the Z axis
//   {
//     auto stage = std::make_unique<stages::MoveRelative>("rz -18°", cartesian);
//     stage->properties().configureInitFrom(Stage::PARENT);
//     geometry_msgs::msg::TwistStamped twist;
//     twist.header.frame_id = "link_base";
//     twist.twist.angular.z = -M_PI / 10.; // 18 degrees in radians
//     stage->setDirection(twist);
//     c->insert(std::move(stage));
//     RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Added stage: Rotate -18 degrees around Z axis");
//   }

//   // Stage 4: Move to the "hold-up" position
//   {
//     auto stage = std::make_unique<stages::MoveTo>("moveTo hold-up", joint_interpolation);
//     stage->properties().configureInitFrom(Stage::PARENT);
//     stage->setGoal("hold-up");
//     c->insert(std::move(stage));
//     RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Added stage: Move to 'hold-up' position");
//   }

  RCLCPP_INFO(rclcpp::get_logger("xarm7_mtc_test"), "Module creation completed with 4 stages");
  return c;
}

/**
 * @brief Creates the main task for robot movement.
 *
 * @param node The ROS2 node to use for loading the robot model.
 * @return Task The complete task for robot movement.
 */
Task createTask(const rclcpp::Node::SharedPtr& node) {
  Task t;
  t.loadRobotModel(node);
  // t.setConrollers
  t.stages()->setName("Reusable Containers");

  RCLCPP_INFO(node->get_logger(), "Creating task: %s", t.stages()->name().c_str());

  // Add the current state as the starting point
  t.add(std::make_unique<stages::CurrentState>("current"));
  RCLCPP_INFO(node->get_logger(), "Added current state as starting point");

  // Define the robot arm_group to move
  const std::string arm_group = "xarm7";

  // Add a stage to move to the "hold-up" position
  // {
  //   auto stage = std::make_unique<stages::MoveTo>("move to hold-up", std::make_shared<solvers::JointInterpolationPlanner>());
  //   stage->setGroup(arm_group);
  //   stage->setGoal("hold-up");
  //   t.add(std::move(stage));
  //   RCLCPP_INFO(node->get_logger(), "Added stage: Move to 'hold-up' position");
  // }

  // Add five instances of our reusable module
  // This creates a sequence of movements that the robot will perform,
  // repeating the same set of actions five times in a row.
//   RCLCPP_INFO(node->get_logger(), "Adding 5 instances of the reusable module");
//   for (int i = 1; i <= 5; ++i) {
    t.add(createModule(arm_group));
//     RCLCPP_INFO(node->get_logger(), "Added module instance %d", i);
//   }

  // Add a stage to move to the "home" position
  {
    auto stage = std::make_unique<stages::MoveTo>("move to home", std::make_shared<solvers::JointInterpolationPlanner>());
    stage->setGroup(arm_group);
    stage->setGoal("hold-up");
    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added stage: Move to 'home' position");
  }

  RCLCPP_INFO(node->get_logger(), "Task creation completed with 5 module instances");
  return t;
}

/**
 * @brief Main function to set up and execute the robot task.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */

 /**
 * @brief Main function to set up and execute the robot task.
 */
int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("xarm7_mtc_test");
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Starting xarm7 MTC test");

  // Create and plan the task
  auto task = createTask(node);
  try {
    RCLCPP_INFO(logger, "Starting task planning");

    if (task.plan()) {
      RCLCPP_INFO(logger, "Task planning completed successfully. Found %zu solutions.", task.numSolutions());
      task.introspection().publishSolution(*task.solutions().front());

      // RCLCPP_INFO(logger, "=== READY FOR EXECUTION ===");
      // RCLCPP_INFO(logger, "Press Enter to execute the planned motion...");
      // std::cin.get(); // Wait for user confirmation

      RCLCPP_INFO(logger, "🚀 Starting MTC task execution now!");
      
      try {
        // Execute and get the result directly
        RCLCPP_INFO(logger, "About to execute MTC task...");
        auto execution_result = task.execute(*task.solutions().front());

        // =================================================================
        // THIS IS THE CRITICAL FIX: Check the result
        // =================================================================
        RCLCPP_INFO(logger, "MTC task execution completed.");
        
        // Check if execution was successful
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(logger, "🎉 MTC Task execution completed successfully!");
        } else {
          RCLCPP_ERROR(logger, "❌ MTC Task execution failed with error code: %d", execution_result.val);
        }
        
        // Add a small delay to let move_group finish cleanup
        RCLCPP_INFO(logger, "Waiting for move_group cleanup...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
      } catch (const std::exception& ex) {
        RCLCPP_ERROR(logger, "Exception during task execution: %s", ex.what());
      } catch (...) {
        RCLCPP_ERROR(logger, "Unknown exception during task execution");
      }
      
    } else {
      RCLCPP_ERROR(logger, "Task planning failed.");
      task.explainFailure(std::cout);
    }

  } catch (const InitStageException& ex) {
    RCLCPP_ERROR(logger, "InitStageException caught: %s", ex.what());
  }

  RCLCPP_INFO(logger, "xarm7 MTC test shutting down.");
  rclcpp::shutdown();
  return 0;
}