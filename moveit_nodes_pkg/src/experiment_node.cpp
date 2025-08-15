/* Copyright 2024 Virtual Mechatronics Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

 #include "moveit_nodes_pkg/experiment_node.hpp"


using namespace std::chrono_literals;


 int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("experiment_node", node_options);
   
    // Spinning up a SingleThreadedExecutor for  the current state monitor to get information about the robot's state
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

     // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    std::string GRIPPER_GROUP = "xarm_gripper";


    // Arm group object
    auto arm_node = std::make_shared<experiment::ExperimentNode>(node, ARM_GROUP);
    // gripper group object
    auto gripper_node = std::make_shared<experiment::ExperimentNode>(node, GRIPPER_GROUP);

    // home joints positions
    std::vector<double> home_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, -1.571, 0.0};
    // Open gripper
    std::vector<double> open_gripper_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Close gripper
    std::vector<double> close_gripper_positions = {0.84, 0.84, 0.84, 0.84, 0.84, 0.84};

    // Spindle pick pose
    const auto spindle_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.326;
        msg.position.y = -0.084;
        msg.position.z = 0.15;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Spur pick pose
    const auto spur_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.491;
        msg.position.y = 0.07;
        msg.position.z = 0.15;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Idler pick pose
    const auto idler_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3356;
        msg.position.y = 0.0288;
        msg.position.z = 0.05;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Cover pick pose
    const auto cover_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.5014;
        msg.position.y = -0.085;
        msg.position.z = 0.07;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Spindle place pose
    const auto spindle_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3726;
        msg.position.y = -0.3771;
        msg.position.z = 0.2;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Idler place pose
    const auto idler_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.4462;
        msg.position.y = -0.3659;
        msg.position.z = 0.15;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Spur place pose
    const auto spur_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3726;
        msg.position.y = -0.3429;
        msg.position.z = 0.15;
        msg.orientation.x = 0.7071;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.7071;
        return msg;
    }();

    const auto move_up_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3726;
        msg.position.y = -0.3;
        msg.position.z = 0.2;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();


    arm_node->prompt("Press 'Next' in the RvizVisualToolsGui window to execute next step");
    arm_node->moveit_visual_tools->trigger();
    // Pick Spindle
    arm_node->planToTargetPose(spindle_pick_pose);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(spindle_pick_pose, 1, {0, 0, -0.1});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(close_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.1});
    // Place spindle
    arm_node->planToTargetPose(spindle_place_pose);
    arm_node->planCartesianPath(spindle_place_pose, 1, {0, 0, -0.05});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.2});
    // Move to home pose
    arm_node->planToJointSpaceTarget(home_joint_positions);
    arm_node->prompt("Press 'Next' in the RvizVisualToolsGui window to execute next step");
    arm_node->moveit_visual_tools->trigger();

    // // Pick spur gear
    arm_node->planToTargetPose(spur_pick_pose);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(spur_pick_pose, 1, {0, 0, -0.07});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(close_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.07});
    // Place spur gear
    arm_node->planToTargetPose(spur_place_pose);
    // arm_node->planCartesianPath(spur_place_pose, 1, {0, 0, -0.05});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planToTargetPose(move_up_pose);
    rclcpp::sleep_for(2s);
    // Move to home pose
    arm_node->planToJointSpaceTarget(home_joint_positions);
    arm_node->prompt("Press 'Next' in the RvizVisualToolsGui window to execute next step");
    arm_node->moveit_visual_tools->trigger();

    // Pick idler gear
    arm_node->planToTargetPose(idler_pick_pose);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(idler_pick_pose, 1, {0, 0, -0.04});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(close_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.04});
    // Place idler gear
    arm_node->planToTargetPose(idler_place_pose);
    arm_node->planCartesianPath(idler_place_pose, 1, {0, 0, -0.04});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.12});
    // Move to home pose
    arm_node->planToJointSpaceTarget(home_joint_positions);
    arm_node->prompt("Press 'Next' in the RvizVisualToolsGui window to execute next step");
    arm_node->moveit_visual_tools->trigger();

    // Pick cover
    arm_node->planToTargetPose(cover_pick_pose);
    gripper_node->planToJointSpaceTarget(open_gripper_positions);
    arm_node->planCartesianPath(cover_pick_pose, 1, {0, 0, -0.055});
    rclcpp::sleep_for(2s);
    gripper_node->planToJointSpaceTarget(close_gripper_positions);
    arm_node->planCartesianPath(arm_node->current_pose, 1, {0, 0, 0.15});
    arm_node->prompt("Press 'Next' in the RvizVisualToolsGui window to execute next step");
    arm_node->moveit_visual_tools->trigger();

    
    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
 }