#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Motion_Planner");

void generate_and_execute_joint_plan(moveit::planning_interface::MoveGroupInterface &move_group, const std::vector<double> &target_joint_values) {
    move_group.setJointValueTarget(target_joint_values);

    // Generate plan to the target joint values
    auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute plan
    if (success) {
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to execute");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto motion_planner = rclcpp::Node::make_shared("motion_planner_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_planner);
    std::thread([&executor] {
        executor.spin();
    }).detach();

    const std::string PLANNING_GROUP_NAME = "arm";

    moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP_NAME, "robot_description");
    moveit::planning_interface::MoveGroupInterface move_group(motion_planner, options);

    // Set the planner ID
    // move_group.setPlannerId("LBKPIECEkConfigDefault");

    // Define target joint values
    std::vector<double> pregrasp_joint_values = {0.4, 0.0, -1.5708, 0.0, -1.5708, 0.0};  // Modify these values based on your robot's joint limits
    std::vector<double> dry_wall2_pick_joints = {0.298, 0.0, -1.5708, 0.0, -1.5708, 0.0}; // Modify these values based on your robot's
    std::vector<double> dry_wall2_preplace_joints = {0.0, 3.1416, 0.0, -1.5708, 0.0, 0.0};

    // std::vector<double> dry_wall2_preplace_joints = {0.8, 0.0, -1.5708, 0.0, -1.5708, 0.0};
    std::vector<double> dry_wall2_place_joints = {0.0, 3.1416, -1.367199, 0.0, -0.203596, 0.0}; //


    // Define joint targets for picking and placing drywall_1
    std::vector<double> drywall_1_pick_joints = {0.02, 0.0, -1.5708, 0.0, -1.5708, 0.0};
    std::vector<double> drywall_1_preplace_joints = {1.3716, 3.1416, 0.0, -1.5708, 0.0, 0.0};
    std::vector<double> drywall_1_place_joints = {1.3716, 3.1416, -1.367199, 0.0, -0.203596, 0.0};

    // Generate and execute plan to pick and place drywall_2
    generate_and_execute_joint_plan(move_group, pregrasp_joint_values);
    generate_and_execute_joint_plan(move_group, dry_wall2_pick_joints);
    move_group.setSupportSurfaceName("world");
    // Grasp the object/ (attach object to robot)
    move_group.attachObject("drywall_2");
    generate_and_execute_joint_plan(move_group, dry_wall2_preplace_joints);
    generate_and_execute_joint_plan(move_group, dry_wall2_place_joints);
    // Place the object/ (detach object from robot)
    move_group.detachObject("drywall_2");

    generate_and_execute_joint_plan(move_group, pregrasp_joint_values);

    // Generate and execute plan to pick and place drywal_1
    generate_and_execute_joint_plan(move_group, drywall_1_pick_joints);
    move_group.setSupportSurfaceName("world");
    // Grasp the object/ (attach object to robot)
    move_group.attachObject("drywall_1");
    // generate_and_execute_joint_plan(move_group, drywall_1_preplace_joints);
    generate_and_execute_joint_plan(move_group, drywall_1_place_joints);
    // Place the object/ (detach object from robot)
    move_group.detachObject("drywall_1");
    generate_and_execute_joint_plan(move_group, pregrasp_joint_values);


    rclcpp::shutdown();
    return 0;
}
