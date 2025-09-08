cmake_minimum_required(VERSION 3.8)
project(xarm7_hybrid_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(pick_and_place_sm SHARED src/pick_and_place_sm.cpp)
target_include_directories(
  pick_and_place_sm
  PRIVATE "pick_and_place_sm_node_CPP_BUILD_DLL"
)

ament_target_dependencies(
  pick_and_place_sm
    "rclcpp"
    "rclcpp_components"
    "xarm_msgs"
    "geometry_msgs"
    "std_msgs"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
)

rclcpp_components_register_node(
  pick_and_place_sm
    PLUGIN "simple_state_machine::PickAndPlaceStateMachine"
    EXECUTABLE pick_and_place_sm_node
)

# Create standalone executable with multithreaded executor
add_executable(pick_and_place_sm_standalone src/pick_and_place_sm_node.cpp)
target_link_libraries(pick_and_place_sm_standalone pick_and_place_sm)
ament_target_dependencies(
  pick_and_place_sm_standalone
    "rclcpp"
    "rclcpp_components"
    "xarm_msgs"
    "geometry_msgs"
    "std_msgs"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
)

install(
  TARGETS 
    pick_and_place_sm
    pick_and_place_sm_standalone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
