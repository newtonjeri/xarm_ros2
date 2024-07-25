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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(boost_filesystem REQUIRED)


find_package(Boost REQUIRED COMPONENTS filesystem)


# include directories
include_directories(${Boost_RERUIRED_DIRS})

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

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
  "moveit_visual_tools"
  "rclcpp"
)

target_link_libraries(task_plan_node ${Boost_LIBRARIES})

add_executable(gripper_node src/gripper_node.cpp)
ament_target_dependencies(
  gripper_node
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(cartesian_path_node src/cartesian_path_planning_test.cpp)
ament_target_dependencies(
  cartesian_path_node
  "moveit_ros_planning_interface"
  "rclcpp"
)


# Server installation Setup
add_library(voice_command_server SHARED src/voice_command_server.cpp)
target_include_directories(
  voice_command_server 
  PRIVATE "voice_command_server_CPP_BUILDING_DLL"
)
ament_target_dependencies(
  voice_command_server
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "xarm_msgs"
    "moveit_ros_planning_interface"
  )

rclcpp_components_register_node(
  voice_command_server
    PLUGIN "voice_command::VoiceCommandServer"
    EXECUTABLE voice_command_server_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
    goal_pose_planner_and_executor
    joints_position_planner_and_executor
    move_to_A_B_C_points
    mover_node
    unity_subscriber_cpp_node
    task_plan_node
    gripper_node
    cartesian_path_node
  DESTINATION 
    lib/${PROJECT_NAME}
  )

install(
  TARGETS
    voice_command_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Move folders to share
install(
  DIRECTORY
    launch
  DESTINATION 
    share/${PROJECT_NAME} 
)



ament_package()
