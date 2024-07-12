#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

    //Logger
    auto const LOGGER = rclcpp::get_logger("Gripper Logeer");

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);



    auto const node = std::make_shared<rclcpp::Node>("gripper_node", node_options);
    std::string group_name = "xarm_gripper";
    // Move group interface 
    auto gripper_group = moveit::planning_interface::MoveGroupInterface(node, group_name);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", gripper_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", gripper_group.getEndEffectorLink().c_str());

    double gripper_open_joint_value = 0;
    double gripper_close_joint_value = 0.8;

    // Open gripper
    gripper_group.setJointValueTarget({gripper_open_joint_value});

    auto const [success, plan] = [&gripper_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(gripper_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success){
        gripper_group.move();
    }

    // Close gripper
    gripper_group.setJointValueTarget({gripper_close_joint_value});

    auto const [success_, plan_] = [&gripper_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(gripper_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success_){
        gripper_group.move();
    }

    rclcpp::shutdown();
    return 0;

}