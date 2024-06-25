#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    //Logger
    auto const LOGGER = rclcpp::get_logger("Gripper Logeer");

    auto const node = std::make_shared<rclcpp::Node>("gripper_node", node_options);
    std::string group_name = "xarm_gripper";
    // Move group interface 
    auto gripper_group = moveit::planning_interface::MoveGroupInterface(node, group_name);


    // Open gripper
    gripper_group.setJointValueTarget({0.84});

    auto const [success, plan] = [&gripper_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(gripper_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success){
        gripper_group.execute(plan);
    }

    // Close gripper
    gripper_group.setJointValueTarget({0.84});

    auto const [success_, plan_] = [&gripper_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(gripper_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success_){
        gripper_group.execute(plan_);
    }

    rclcpp::shutdown();
    return 0;

}