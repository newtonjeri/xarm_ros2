
/* Copyright 2024 Virtual Reality Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <xarm_msgs/msg/move_next_command.hpp>
#include <xarm_msgs/msg/stop_command.hpp>


#include "moveit_nodes_pkg/experiment_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// name spaces

using namespace std::placeholders;
using namespace std::chrono_literals;

class Experiment002Node : public rclcpp::Node
{
public:
    Experiment002Node(
        const rclcpp::NodeOptions &options)
        : Node("experiment_002_node", options)
    {
        // Create a reentrant callback group
        reentrant_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions subscriptions_options1;
        subscriptions_options1.callback_group = reentrant_callback_group_;

        move_next_subscription = this->create_subscription<xarm_msgs::msg::MoveNextCommand>(
                                "/is_next_topic",
                                 10,
                                std::bind(&Experiment002Node::isNextCallback, this, _1),
                                subscriptions_options1);


        rclcpp::SubscriptionOptions subscriptions_options2;
        subscriptions_options2.callback_group = reentrant_callback_group_;
        stop_subscription = this->create_subscription<xarm_msgs::msg::StopCommand>(
                                "/stop_command_topic", 
                                10, 
                                std::bind(&Experiment002Node::stopCommandCallback, this, _1),
                                subscriptions_options2);

        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Experiment002Node created ");

        // Initialize MoveIt interface in a separate method using a timer
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Experiment002Node::initialize_moveit, this));
    }

private:
    void initialize_moveit()
    {
        // Set up the MoveIt 2 Interface here
        auto node_ptr_ = shared_from_this();
        experiment_node_arm = std::make_shared<experiment::ExperimentNode>(node_ptr_, ARM_GROUP);
        experiment_node_gripper = std::make_shared<experiment::ExperimentNode>(node_ptr_, GRIPPER_GROUP);

        // experiment_node_arm->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        // experiment_node_arm->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");
        timer_->cancel(); // Stop the timer after initialization
    }

    void isNextCallback(const xarm_msgs::msg::MoveNextCommand::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "is_next = %d", msg->is_next);
        executeTaskBasedOnId((msg->is_next));
    }

    void stopCommandCallback(const xarm_msgs::msg::StopCommand::SharedPtr msg){
        if (msg->stop_command_state){
            stop_moving = msg->stop_command_state;
            experiment_node_arm->move_group->stop();
            RCLCPP_WARN(rclcpp::get_logger("Experiment 002 node"), "Received message to stop the robot: %d", stop_moving);
        }
    }

    void pickAndPlaceSpindle2(const std::string object_name = "spindle_2")
    {
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Picking spindle 2");
        // Pick Spindle
        experiment_node_arm->planToTargetPose(spindle_pick_pose, stop_moving);
        experiment_node_gripper->gripperOpenAndClose(open_halfway_gripper_positions);
        // experiment_node_arm->planCartesianPath(spindle_pick_pose, 1, {0, 0, -0.1});
        spindle_pick_pose.position.z += -0.075;
        experiment_node_arm->planToTargetPose(spindle_pick_pose, stop_moving);
        experiment_node_arm->move_group->attachObject(object_name, "link_tcp", {"right_finger", "left_finger", "base_plate"});
        experiment_node_gripper->gripperOpenAndClose(close_gripper_positions);
        rclcpp::sleep_for(1s);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.1});
        spindle_pick_pose.position.z += 0.075;
        experiment_node_arm->planToTargetPose(spindle_pick_pose, stop_moving);
        // Place spindle
        experiment_node_arm->planToTargetPose(spindle_place_pose, stop_moving);
        // experiment_node_arm->planCartesianPath(spindle_place_pose, 1, {0, 0, -0.05});
        spindle_place_pose.position.z += -0.08;
        experiment_node_arm->planToTargetPose(spindle_place_pose, stop_moving);
        rclcpp::sleep_for(1s);
        experiment_node_gripper->gripperOpenAndClose(open_halfway_gripper_positions);
        experiment_node_arm->move_group->detachObject(object_name);
        rclcpp::sleep_for(1s);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.2});
        spindle_place_pose.position.z += 0.08;
        experiment_node_arm->planToTargetPose(spindle_place_pose, stop_moving);
        // Move to home pose
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        if (stop_moving){
            stop_moving = false;
            return;
        }
        stop_moving = false;
        task_id = 2;
        psi.removeCollisionObjects({"spindle_2"});
    }

    void pickAndPlacePinionGear(std::string object_name = "pinion_gear")
    {      
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Picking pinion gear");
        experiment_node_arm->planToTargetPose(pinion_pick_pose, stop_moving);
        experiment_node_gripper->gripperOpenAndClose(open_gripper_positions);
        // experiment_node_arm->planCartesianPath(pinion_pick_pose, 1, {0, 0, -0.07});
        pinion_pick_pose.position.z += -0.055;
        experiment_node_arm->planToTargetPose(pinion_pick_pose, stop_moving);
        experiment_node_arm->move_group->attachObject(object_name, "link_tcp", {"left_finger", "right_finger", "spindle_2", "base_plate"});
        rclcpp::sleep_for(1s);
        experiment_node_gripper->gripperOpenAndClose(close_gripper_positions);
        rclcpp::sleep_for(1s);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.07});;
        pinion_pick_pose.position.z += 0.055;
        experiment_node_arm->planToTargetPose(pinion_pick_pose, stop_moving);
        // Place pinion gear
        experiment_node_arm->planToTargetPose(pinion_place_pose, stop_moving);
        // experiment_node_arm->planCartesianPath(pinion_place_pose, 1, {0, 0, -0.05});
        pinion_place_pose.position.z += -0.08;
        experiment_node_arm->planToTargetPose(pinion_place_pose, stop_moving);
        rclcpp::sleep_for(1s);
        experiment_node_gripper->gripperOpenAndClose(open_halfway_gripper_positions);
        experiment_node_arm->move_group->detachObject(object_name);
        pinion_place_pose.position.z += 0.08;
        experiment_node_arm->planToTargetPose(pinion_place_pose, stop_moving);
        rclcpp::sleep_for(1s);
        // pinion_place_pose.position.z += 0.264;
        // experiment_node_arm->planToTargetPose(pinion_place_pose, stop_moving);
        // Move to home pose
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        if (stop_moving){
            stop_moving = false;
            return;
        }
        stop_moving = false;
        task_id = 4;
        psi.removeCollisionObjects({"pinion_gear"});
    }

    void pickAndPlaceIdlerGear(std::string object_name = "idler_gear")
    {
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Picking idler gear");
        experiment_node_arm->planToTargetPose(idler_pick_pose, stop_moving);
        experiment_node_gripper->gripperOpenAndClose(open_gripper_positions);
        // experiment_node_arm->planCartesianPath(idler_pick_pose, 1, {0, 0, -0.04});
        idler_pick_pose.position.z += -0.05;
        experiment_node_arm->planToTargetPose(idler_pick_pose, stop_moving);
        experiment_node_arm->move_group->attachObject(object_name, "link_tcp", {"right_finger", "left_finger", "base_plate", "pinion_gear"});
        experiment_node_gripper->gripperOpenAndClose(close_gripper_positions);
        rclcpp::sleep_for(1s);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.04});
        idler_pick_pose.position.z += 0.1;
        experiment_node_arm->planToTargetPose(idler_pick_pose, stop_moving);
        // Place idler gear
        experiment_node_arm->planToTargetPose(idler_place_pose, stop_moving);
        // experiment_node_arm->planCartesianPath(idler_place_pose, 1, {0, 0, -0.04});
        // idler_place_pose.position.z += -0.0908;
        idler_place_pose.position.z += -0.0808;
        experiment_node_arm->planToTargetPose(idler_place_pose, stop_moving);
        rclcpp::sleep_for(1s);
        experiment_node_gripper->gripperOpenAndClose(open_gripper_positions);
        experiment_node_arm->move_group->detachObject(object_name);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.12});
        idler_place_pose.position.z += 0.093;
        experiment_node_arm->planToTargetPose(idler_place_pose, stop_moving);
        // Move to home pose
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        if (stop_moving){
            stop_moving = false;
            return;
        }
        stop_moving = false;
        task_id = 3;
        psi.removeCollisionObjects({"idler_gear"});
    }

    void pickCover(std::string object_name = "cover_plate")
    {
        experiment_node_arm->planToJointSpaceTarget(home_joint_positions, stop_moving);
        RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Picking cover");
        experiment_node_arm->planToTargetPose(cover_pick_pose, stop_moving);
        experiment_node_gripper->gripperOpenAndClose(open_gripper_positions);
        // experiment_node_arm->planCartesianPath(cover_pick_pose, 1, {0, 0, -0.055});
        cover_pick_pose.position.z += -0.0435;
        experiment_node_arm->planToTargetPose(cover_pick_pose, stop_moving);
        experiment_node_arm->move_group->attachObject(object_name, "link_tcp", gripper_fingers);
        experiment_node_gripper->gripperOpenAndClose(close_gripper_positions);
        rclcpp::sleep_for(1s);
        // experiment_node_arm->planCartesianPath(experiment_node_arm->current_pose, 1, {0, 0, 0.15});
        cover_pick_pose.position.z += 0.19;
        experiment_node_arm->planToTargetPose(cover_pick_pose, stop_moving);
        if (stop_moving){
            stop_moving = false;
            return;
        }
        stop_moving = false;
        task_id = 0;
    }

    void executeTaskBasedOnId(bool is_next)
    {
        // experiment_node_arm->move_group->setSupportSurfaceName("table_plane");
        if (is_next == true)
        {
            if (task_id == 1)
            {
                pickAndPlaceSpindle2();
            }
            else if (task_id == 2)
            {
                // pickAndPlacePinionGear();
                pickAndPlaceIdlerGear();
            }
            else if (task_id == 3)
            {
                // pickAndPlaceIdlerGear();
                pickAndPlacePinionGear();
            }
            else if (task_id == 4)
            {
                pickCover();
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "Task not found");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("Experiment 002 node"), "No next task");
            return;
        }
    }

    // Variables
    moveit::planning_interface::PlanningSceneInterface psi;
    // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    std::string GRIPPER_GROUP = "xarm_gripper";

    const double max_velocity_scaling_factor = 0.1;     // [move_group_interface] default is 0.1
    const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

    // Subscriptions 
    rclcpp::Subscription<xarm_msgs::msg::MoveNextCommand>::SharedPtr move_next_subscription;
    rclcpp::Subscription<xarm_msgs::msg::StopCommand>::SharedPtr stop_subscription;

    // Callback group member
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Experiment node object
    std::shared_ptr<experiment::ExperimentNode> experiment_node_arm;
    std::shared_ptr<experiment::ExperimentNode> experiment_node_gripper;

    // Gripper fingers
    std::vector<std::string> gripper_fingers = {"left_finger", "right_finger"};

    std::int16_t task_id = 1;
    // std::int16_t task_id = 2;
    // std::int16_t task_id = 3;
    bool stop_moving = false;

    // home joints positions
    std::vector<double> home_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, -1.571, 0.0};
    // Open gripper
    std::vector<double> open_gripper_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> open_halfway_gripper_positions = {0.40, 0.40, 0.40, 0.40, 0.40, 0.40};
    // Close gripper
    std::vector<double> close_gripper_positions = {0.75, 0.75, 0.75, 0.75, 0.75, 0.75};

    // Spindle pick pose
    geometry_msgs::msg::Pose spindle_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3285;
        msg.position.y = -0.08823;
        msg.position.z = 0.1261;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Pinion pick pose
    geometry_msgs::msg::Pose pinion_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.4912;
        msg.position.y = 0.0661;
        msg.position.z = 0.0650;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Idler pick pose
    geometry_msgs::msg::Pose idler_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3357;
        msg.position.y = 0.0295;
        msg.position.z = 0.0558;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Cover pick pose
    geometry_msgs::msg::Pose cover_pick_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.5052;
        msg.position.y = -0.08395;
        msg.position.z = 0.06;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Spindle place pose
    geometry_msgs::msg::Pose spindle_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3665;
        msg.position.y = -0.3788;
        // msg.position.z = 0.2174
        msg.position.z = 0.15;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // geometry_msgs::msg::Pose spindle_place_pose = []
    // {
    //     geometry_msgs::msg::Pose msg;
    //     msg.position.x = 0.4592;
    //     msg.position.y = -0.4074;
    //     msg.position.z = 0.15;
    //     msg.orientation.x = 1.0;
    //     msg.orientation.y = 0.0;
    //     msg.orientation.z = 0.0;
    //     msg.orientation.w = 0.0;
    //     return msg;
    // }();

    // Idler place pose
    geometry_msgs::msg::Pose idler_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.4506;
        msg.position.y = -0.3707;
        msg.position.z = 0.1407;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    // Pinion place pose
    geometry_msgs::msg::Pose pinion_place_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.4612;
        msg.position.y = -0.4039;
        msg.position.z = 0.15;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

};

RCLCPP_COMPONENTS_REGISTER_NODE(Experiment002Node)