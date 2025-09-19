cmake_minimum_required(VERSION 3.8)

project(xarm7_mtc_hybrid_state_machine)

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
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)



# Create MTC tasks library
add_library(mtc_tasks SHARED 
  src/base_task.cpp
  src/tasks/spindle_task.cpp
  src/tasks/pinion_task.cpp
  src/tasks/idler_task.cpp
  src/tasks/cover_task.cpp

)

target_include_directories(
  mtc_tasks
  PRIVATE "mtc_tasks_CPP_BUILD_DLL"
)

ament_target_dependencies(
  mtc_tasks
    "rclcpp"
    "rclcpp_components"
    "xarm_msgs"
    "geometry_msgs"
    "std_msgs"
    "std_srvs"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "moveit_task_constructor_core"
    "moveit_task_constructor_msgs"
)


# Create MTC state machine library
add_library(mtc_state_machine SHARED src/mtc_state_machine.cpp)
  
target_include_directories(
  mtc_state_machine
  PRIVATE "mtc_state_machine_CPP_BUILD_DLL"
)


ament_target_dependencies(
  mtc_state_machine
    "rclcpp"
    "rclcpp_components"
    "xarm_msgs"
    "geometry_msgs"
    "std_msgs"
    "std_srvs"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "moveit_task_constructor_core"
    "moveit_task_constructor_msgs"
)


target_link_libraries(mtc_state_machine mtc_tasks)

rclcpp_components_register_node(
  mtc_state_machine
    PLUGIN "xarm7_mtc::MTCStateMachine"
    EXECUTABLE mtc_state_machine_node
)  


# Create standalone MTC service executable

add_executable(mtc_service_standalone src/mtc_service_node.cpp)
target_link_libraries(mtc_service_standalone mtc_state_machine)  
ament_target_dependencies(
  mtc_service_standalone
    "rclcpp"
    "rclcpp_components"
    "xarm_msgs"
    "geometry_msgs"
    "std_msgs"
    "std_srvs"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "moveit_task_constructor_core"
    "moveit_task_constructor_msgs" 
)

# Create task command bridge executable
add_executable(task_command_bridge src/task_command_bridge.cpp)
ament_target_dependencies(
  task_command_bridge
    "rclcpp"
    "xarm_msgs"
    "std_msgs"
)

# Create mtc_test executable
add_executable(mtc_test test/mtc_test.cpp)
ament_target_dependencies(
  mtc_test
    "rclcpp"
    "moveit_task_constructor_core"
    "moveit_task_constructor_msgs"
    "geometry_msgs"
)


install(
  TARGETS
    mtc_tasks
    mtc_state_machine
    mtc_service_standalone
    task_command_bridge
    mtc_test
  ARCHIVE DESTINATION lib  
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.

install(
  DIRECTORY
   launch
   config
  DESTINATION share/${PROJECT_NAME}  
)

ament_package()
