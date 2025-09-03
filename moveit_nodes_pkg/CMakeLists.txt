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

#hrc
find_package(jsoncpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)


# include directories
include_directories(${Boost_REQUIRED_DIRS})

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

add_executable(xarm7_mover_node src/xarm7_mover.cpp)
ament_target_dependencies(
  xarm7_mover_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "xarm_msgs"
)
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(
  test_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "xarm_msgs"
)

target_link_libraries(task_plan_node ${Boost_LIBRARIES})

add_executable(xarm_gripper_node src/gripper_node.cpp)
ament_target_dependencies(
  xarm_gripper_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "xarm_msgs"
)

add_executable(update_planning_scene_node src/update_planning_scene.cpp)
ament_target_dependencies(
  update_planning_scene_node
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "moveit_msgs"
  "rclcpp"
)

add_executable(experiment_node src/experiment_node.cpp)
ament_target_dependencies(
  experiment_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
)

add_executable(pick_and_place_gear_node src/pick_and_place_gear.cpp)
ament_target_dependencies(
  pick_and_place_gear_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
)


#hrc
add_executable(hrc_node src/hrc.cpp)
ament_target_dependencies(
hrc_node rclcpp geometry_msgs jsoncpp std_msgs moveit_ros_planning_interface
)

target_link_libraries(hrc_node jsoncpp_lib)




# Experiment002 node
add_library(experiment_002 SHARED src/experiment_002.cpp)
target_include_directories(
  experiment_002
  PRIVATE "experiment_002_node_CPP_BUILD_DLL"
)
ament_target_dependencies(
  experiment_002
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
  "xarm_msgs"
)

rclcpp_components_register_node(
  experiment_002
    PLUGIN "Experiment002Node"
    EXECUTABLE experiment_002_node
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
    xarm_gripper_node
    update_planning_scene_node
    xarm7_mover_node
    experiment_node
    test_node
    pick_and_place_gear_node
  DESTINATION 
    lib/${PROJECT_NAME}
  )

install(
  TARGETS
    experiment_002
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Move folders to share
install(
  DIRECTORY
    launch gear_box
  DESTINATION 
    share/${PROJECT_NAME} 
)


#hrc
install(TARGETS
  hrc_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
