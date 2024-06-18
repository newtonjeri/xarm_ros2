#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <xarm_msgs/srv/mover_service.hpp>
#include <xarm_msgs/msg/xarm7_unity_target_pose_to_ros.hpp>
#include <xarm_msgs/msg/xarm7_ros_joints_to_unity.hpp>
#include <xarm_msgs/msg/xarm7_unity_robot_joints.hpp>
#include <xarm_msgs/msg/joint_names_and_angles.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>


using namespace std::chrono_literals;
using MoverService = xarm_msgs::srv::MoverService;
using MoverServiceRequest = xarm_msgs::srv::MoverService::Request;
using MoverServiceResponse = xarm_msgs::srv::MoverService::Response;
using Xarm7UnityTargetPoseToROS = xarm_msgs::msg::Xarm7UnityTargetPoseToROS;
using Xarm7ROSJointsToUnity = xarm_msgs::msg::Xarm7ROSJointsToUnity;
using Xarm7UnityRobotJoints = xarm_msgs::msg::Xarm7UnityRobotJoints;
using Xarm7JointNamesAndAngle = xarm_msgs::msg::JointNamesAndAngles;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("UnitySubscriberCppNode");

class UnitySubscriberCppNode
{
public:
  UnitySubscriberCppNode(rclcpp::Node::SharedPtr &node) : node_(node)
  {
    node_->declare_parameter<std::string>("group_name", "xarm7");

    std::string group_name;
    node_->get_parameter("group_name", group_name);

    init(group_name);

    // pub_ = node_->create_publisher<Xarm7ROSJointsToUnity>("xarm_ros_joints_to_unity", 10);
    sub_ = node_->create_subscription<Xarm7JointNamesAndAngle>("/joint_info_fromUnity", 10,
                                                                std::bind(&UnitySubscriberCppNode::xarm7UnityCallback, this, std::placeholders::_1));
    // srv_ = node_->create_service<MoverService>("xarm_moveit", std::bind(&UnitySubscriberCppNode::planPickAndPlace, this,
    //                                                                    std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(LOGGER, "Ready to plan");
  }

  void init(const std::string &group_name)
  {
    is_trajectory_ = false;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  }

private:

  void xarm7UnityCallback(const Xarm7JointNamesAndAngle::SharedPtr msg)
  {
      std::vector<double> joint_positions(msg->positions.begin(), msg->positions.end());
      try
      {
        planTrajectory(joint_positions);
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(LOGGER, "Exception in planning: %s", ex.what());
      }
  }



    void planTrajectory(const std::vector<double>& destination_joints_deg)
  {
   std::vector<double> destination_joints(destination_joints_deg.size());
    // Convert each angle from degrees to radians
    std::transform(destination_joints_deg.begin(), destination_joints_deg.end(), destination_joints.begin(),
                   [](double degrees) {
                       return degrees * M_PI / 180.0;
                   });

    
    RCLCPP_INFO(LOGGER, "Joint Values Joint1: %f, Joint2: %f, Joint3: %f, Joint4: %f, Joint5: %f, Joint6: %f, Joint7: %f", destination_joints[0],
                              destination_joints[1], destination_joints[2], destination_joints[3], destination_joints[4], destination_joints[5],
                              destination_joints[6]);
                              
    bool success = move_group_->setJointValueTarget(destination_joints);

    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
    if (!success)
    {
      throw std::runtime_error("Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
    }else{
      move_group_->execute(plan);
      is_trajectory_ = true;
    }
  }


  //  Move group
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Xarm7JointNamesAndAngle>::SharedPtr sub_;
  std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  bool is_trajectory_;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
};

void exit_sig_handler([[maybe_unused]] int signum)
{
  fprintf(stderr, "[xarm_planner_node] Ctrl-C caught, exit process...\n");
  exit(-1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("unitysubscribercppnode", node_options);
  auto unitysubscribercppnode = std::make_shared<UnitySubscriberCppNode>(node);
  RCLCPP_INFO(node->get_logger(), "unitysubscribercppnode start");
  signal(SIGINT, exit_sig_handler);

  rclcpp::spin(node);
  rclcpp::shutdown();

  RCLCPP_INFO(node->get_logger(), "unitysubscribercppnode over");
  return 0;
}