/* Copyright 2024 Virtual Reality Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <signal.h>
#include <iostream>

#include "xarm_msgs/msg/gripper_state.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm_gripper_node");

class GripperNode
{
public:
    GripperNode(rclcpp::Node::SharedPtr &node) : node_(node)
    {
        // Declare parameter only if not already declared
        if (!node_->has_parameter("group_name")) {
            node_->declare_parameter<std::string>("group_name", "xarm_gripper");
        }

        std::string group_name;
        node_->get_parameter("group_name", group_name);

        init(group_name);

        gripper_state_subscriber = node_->create_subscription<xarm_msgs::msg::GripperState>(
            GRIPPER_TOPIC, 10, std::bind(&GripperNode::gripperCallback, this, std::placeholders::_1));

        RCLCPP_INFO(LOGGER, "Gripper node initialized and ready");
    }

    void init(const std::string &group_name)
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
        RCLCPP_INFO(LOGGER, "Available Planning Groups:");
        std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(), 
                  std::ostream_iterator<std::string>(std::cout, ", "));
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

    const std::string GRIPPER_TOPIC = "/gripper_state";
    std::vector<double> gripper_open_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> gripper_close_joint_values = {0.84, 0.84, 0.84, 0.84, 0.84, 0.84};

    rclcpp::Subscription<xarm_msgs::msg::GripperState>::SharedPtr gripper_state_subscriber;

    rclcpp::Node::SharedPtr &node_;

    void moveGripper(const std::vector<double> &joints_values)
    {
        try 
        {
            bool success = move_group->setJointValueTarget(joints_values);
            if (!success)
            {
                RCLCPP_WARN(LOGGER, "setJointValueTarget: out of bounds");
                return;
            }
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (!success)
            {
                RCLCPP_ERROR(LOGGER, "Gripper trajectory could not be planned");
                return;
            }
            
            success = (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                RCLCPP_INFO(LOGGER, "Gripper movement executed successfully");
            }
            else
            {
                RCLCPP_ERROR(LOGGER, "Gripper movement execution failed");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(LOGGER, "Exception in gripper movement: %s", ex.what());
        }
    }

    void gripperCallback(const xarm_msgs::msg::GripperState::SharedPtr msg)
    {
        RCLCPP_INFO(LOGGER, "Received gripper command: %s", msg->gripper_state ? "CLOSE" : "OPEN");
        
        if (msg->gripper_state)
        {
            RCLCPP_INFO(LOGGER, "Closing gripper");
            moveGripper(gripper_close_joint_values);
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Opening gripper");
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
    auto gripper_node = std::make_shared<GripperNode>(node);
    
    RCLCPP_INFO(node->get_logger(), "xarm_gripper_node started");
    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    RCLCPP_INFO(node->get_logger(), "xarm_gripper_node shutdown");
    return 0;
}