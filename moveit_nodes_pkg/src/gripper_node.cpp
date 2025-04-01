#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "xarm_msgs/msg/gripper_state.hpp"

class GripperNode
{
public:
    GripperNode(
        rclcpp::Node::SharedPtr &node) : node_(node)
    {
        const std::string group_name = "xarm_gripper";
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

        gripper_state_subscriber = node_->create_subscription<xarm_msgs::msg::GripperState>(
            GRIPPER_TOPIC, 10, std::bind(&GripperNode::gripperCallback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("xarm_griper_node"), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(rclcpp::get_logger("xarm_griper_node"), "End effector link: %s", move_group->getEndEffectorLink().c_str());
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

    const std::string GRIPPER_TOPIC = "/gripper_state";
    std::vector<double> gripper_open_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> gripper_close_joint_values = {0.84, 0.84, 0.84, 0.84, 0.84, 0.84};

    rclcpp::Subscription<xarm_msgs::msg::GripperState>::SharedPtr gripper_state_subscriber;

    rclcpp::Node::SharedPtr &node_;

    void moveGripper(const std::vector<double> joints_values)
    {
        move_group->setJointValueTarget(joints_values);
        move_group->move();
    }

    void gripperCallback(const xarm_msgs::msg::GripperState &msg)
    {
        if (msg.gripper_state)
        {
            moveGripper(gripper_close_joint_values);
        }
        else
        {
            moveGripper(gripper_open_joint_values);
        }
    }
};

void exit_sig_handler([[maybe_unused]] int signum)
{
    fprintf(stderr, "[xarm_gripper_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

// Main Function

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_gripper_node", node_options);

    // gripper group object
    auto gripper_node = std::make_shared<GripperNode>(node);

    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    // rclcpp::spin(gripper_node);
    rclcpp::shutdown();
}