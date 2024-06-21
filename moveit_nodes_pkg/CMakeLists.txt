cmake_minimum_required(VERSION 3.8)
project(moveit_nodes_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



add_executable(goal_pose_planner_and_executor src/goal_pose_planner_and_executor.cpp)
ament_target_dependencies(
  goal_pose_planner_and_executor
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "xarm_msgs"
)

add_executable(joints_position_planner_and_executor src/joints_position_planner_and_executor.cpp)
ament_target_dependencies(
  joints_position_planner_and_executor
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "xarm_msgs"
)

add_executable(move_to_A_B_C_points src/move_to_A_B_C_points.cpp)
ament_target_dependencies(
  move_to_A_B_C_points
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
)


add_executable(mover_node src/mover.cpp)
ament_target_dependencies(
  mover_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "sensor_msgs"
  "xarm_msgs"
)

add_executable(unity_subscriber_cpp_node src/unity_subscriber.cpp)
ament_target_dependencies(
  unity_subscriber_cpp_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "sensor_msgs"
  "xarm_msgs"
)

add_executable(task_plan_node src/task_node.cpp)
ament_target_dependencies(
  task_plan_node
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(
  TARGETS 
    goal_pose_planner_and_executor
    joints_position_planner_and_executor
    move_to_A_B_C_points
    mover_node
    unity_subscriber_cpp_node
    task_plan_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
