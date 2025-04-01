// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");



// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::NodeOptions options;
//     options.automatically_declare_parameters_from_overrides(true);

//     auto node = std::make_shared<rclcpp::Node>("cartesian_path_node", options);

//     const std::string GROUP_NAME = "xarm7";
//     auto arm_group = moveit::planning_interface::MoveGroupInterface(node, GROUP_NAME);

//     RCLCPP_INFO(LOGGER, "Planning frame: %s", arm_group.getPlanningFrame().c_str());
//     RCLCPP_INFO(LOGGER, "End effector link: %s", arm_group.getEndEffectorLink().c_str());

//     std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};

// }

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <rclcpp/rclcpp.hpp>

using namespace moveit::task_constructor;

Task createTask(const rclcpp::Node::SharedPtr& node) {
  Task t;
  t.stages()->setName("Move along X axis");

  const std::string group = "xarm7";
  auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();  
  
  // Load the robot model before creating the task
  t.loadRobotModel(node);

  // Define a Cartesian motion along the x-axis
  auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian_interpolation);
  stage->setGroup(group);

  // Set the direction for x-axis motion
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = "world";
  direction.vector.x = 0.2;  // Move by +0.2m along x-axis

  stage->setDirection(direction);
  t.add(std::move(stage));

  return t;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("xarm_move_x_axis");
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  auto task = createTask(node);
  try {
    if (task.plan())
      task.introspection().publishSolution(*task.solutions().front());
  } catch (const InitStageException& ex) {
    std::cerr << "Planning failed: " << ex.what() << std::endl;
  }

  spinning_thread.join();
  return 0;
}
