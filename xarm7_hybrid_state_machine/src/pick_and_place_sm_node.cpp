/* Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include "xarm7_hybrid_state_machine/pick_and_place_sm.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<simple_state_machine::PickAndPlaceStateMachine>();
    
    // Create multithreaded executor with 4 threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting PickAndPlaceStateMachine with MultiThreadedExecutor (4 threads)");
    
    // Spin until shutdown
    executor.spin();
    
    RCLCPP_INFO(node->get_logger(), "PickAndPlaceStateMachine shutting down");
    rclcpp::shutdown();
    
    return 0;
}
