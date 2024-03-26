#include <thread>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "xarm_msgs/srv/gripper_pose.hpp"

using GripperPoseSrv = xarm_msgs::srv::GripperPose;



/*
Function: 
*/

void saveToCsv(const std::vector<double> points, const std::string& filename) {
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    csvFile << "x,y,z,qx,qy,qz,qw" << std::endl;
    for (const auto point : points) {
        csvFile << point << ","
                << point << ","
                << point << ","
                << point << ","
                << point << ","
                << point << ","
                << point << std::endl;
    }

    std::cout << "Interpolation points saved to " << filename << std::endl;
}

void gripperPoseCallback(const std::shared_ptr<GripperPoseSrv::Request> request,
                         std::shared_ptr<GripperPoseSrv::Response> response,
                         const rclcpp::Logger& logger,
                         rclcpp::Node::SharedPtr node) {
    // Get the request
    double x = request->x;
    double y = request->y;
    double z = request->z;
    double w = request->w;

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "xarm7");

    // Construct and initialize MoveItVisualTools
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools(
        node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel());
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = w;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group_interface.setPoseTarget(target_pose);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface.plan(plan));

    saveToCsv(move_group_interface.getCurrentJointValues(), "interpolation_points.csv");
    // Execute the plan
    if (success) {
        move_group_interface.execute(plan);
        response->success = true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        response->success = false;
    }
}

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("goal_pose_planner_and_executor");

    // Create a service
    auto logger = rclcpp::get_logger("goal_pose_planner_and_executor");
    auto service = node->create_service<GripperPoseSrv>(
        "gripper_pose",
        std::bind(gripperPoseCallback, std::placeholders::_1, std::placeholders::_2, logger, node)
    );

    // Spin
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}
