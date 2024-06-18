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
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>



using namespace std::chrono_literals;
using MoverService = xarm_msgs::srv::MoverService;
using MoverServiceRequest = xarm_msgs::srv::MoverService::Request;
using MoverServiceResponse = xarm_msgs::srv::MoverService::Response;
using Xarm7UnityTargetPoseToROS = xarm_msgs::msg::Xarm7UnityTargetPoseToROS;
using Xarm7ROSJointsToUnity = xarm_msgs::msg::Xarm7ROSJointsToUnity;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MoverNode");

class MoverNode
{
public:
  MoverNode(rclcpp::Node::SharedPtr &node) : node_(node)
  {
    node_->declare_parameter<std::string>("group_name", "xarm7");

    std::string group_name;
    node_->get_parameter("group_name", group_name);

    init(group_name);

    pub_ = node_->create_publisher<Xarm7ROSJointsToUnity>("xarm_ros_joints_to_unity", 10);
    sub_ = node_->create_subscription<Xarm7UnityTargetPoseToROS>("xarm_unity_target_pose_to_ros", 10,
                                                                std::bind(&MoverNode::xarm7UnityCallback, this, std::placeholders::_1));
    srv_ = node_->create_service<MoverService>("xarm_moveit", std::bind(&MoverNode::planPickAndPlace, this,
                                                                       std::placeholders::_1, std::placeholders::_2));

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
  void xarm7UnityCallback(const Xarm7UnityTargetPoseToROS::SharedPtr msg)
  {
    if (!is_trajectory_)
    {
      is_trajectory_ = true;
      try
      {
        motion_plan_ = planTrajectory(msg->target_pose, msg->robot_arm_rotation);
        sendPlanToUnity(motion_plan_.trajectory_);
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(LOGGER, "Exception in planning: %s", ex.what());
        is_trajectory_ = false;
      }
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan planTrajectory(const geometry_msgs::msg::Pose &destination_pose,
                                                                      const geometry_msgs::msg::Vector3 &robot_arm_rotation)
  {
    // geometry_msgs::msg::Pose new_destination_pose = rotatePose(destination_pose, robot_arm_rotation.x, robot_arm_rotation.y, robot_arm_rotation.z);

    // move_group_->setPoseTarget(new_destination_pose);


    RCLCPP_INFO(LOGGER, "Destination Pose Position = [x: %f, y: %f z: %f]", 
                        destination_pose.position.x,
                        destination_pose.position.y,
                        destination_pose.position.z);
    move_group_->setPoseTarget(destination_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
    if (!success)
    {
      throw std::runtime_error("Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
    }else{
      // move_group_->execute(plan);
  }

    return plan;
  }

  

geometry_msgs::msg::Pose rotatePose(const geometry_msgs::msg::Pose &pose, double roll, double pitch, double yaw) {
  // Convert degrees to radians
  roll = roll * M_PI / 180.0;
  pitch = pitch * M_PI / 180.0;
  yaw = yaw * M_PI / 180.0;

  // Create the rotation matrix from roll, pitch, and yaw
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  // Convert the current orientation (quaternion) to a rotation matrix
  Eigen::Quaterniond current_orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d current_rotation_matrix = current_orientation.toRotationMatrix();

  // Combine the rotation matrices
  Eigen::Matrix3d new_rotation_matrix = rotation_matrix * current_rotation_matrix;

  // Extract the new orientation as a quaternion
  Eigen::Quaterniond new_orientation(new_rotation_matrix);

  // Create the new pose with the rotated orientation
  geometry_msgs::msg::Pose new_pose;
  new_pose.position.x = pose.position.x;
  new_pose.position.y = pose.position.y;
  new_pose.position.z = pose.position.z;
  new_pose.orientation.x = new_orientation.x();
  new_pose.orientation.y = new_orientation.y();
  new_pose.orientation.z = new_orientation.z();
  new_pose.orientation.w = new_orientation.w();

  return new_pose;
}


  void planPickAndPlace(const MoverServiceRequest::SharedPtr req, MoverServiceResponse::SharedPtr res)
  {
    try
    {
      auto pre_grasp_pose = planTrajectory(req->pick_pose, req->robot_arm_rotation);
      if (pre_grasp_pose.trajectory_.joint_trajectory.points.empty())
        return;

      // sendPlanToUnity(pre_grasp_pose.trajectory_);
      auto previous_ending_joint_angles = pre_grasp_pose.trajectory_.joint_trajectory.points.back().positions;

      auto pick_pose = req->pick_pose;
      pick_pose.position.z -= 0.05; // Adjust node_ value as necessary

      auto grasp_pose = planTrajectory(pick_pose, req->robot_arm_rotation);
      if (grasp_pose.trajectory_.joint_trajectory.points.empty())
        return;

      // sendPlanToUnity(grasp_pose.trajectory_);
      previous_ending_joint_angles = grasp_pose.trajectory_.joint_trajectory.points.back().positions;

      auto pick_up_pose = planTrajectory(req->pick_pose, req->robot_arm_rotation);
      if (pick_up_pose.trajectory_.joint_trajectory.points.empty())
        return;

      // sendPlanToUnity(pick_up_pose.trajectory_);
      previous_ending_joint_angles = pick_up_pose.trajectory_.joint_trajectory.points.back().positions;

      auto place_pose = planTrajectory(req->place_pose, req->robot_arm_rotation);
      if (place_pose.trajectory_.joint_trajectory.points.empty())
        return;

      sendPlanToUnity(place_pose.trajectory_);
      res->trajectories.push_back(pre_grasp_pose.trajectory_);
      res->trajectories.push_back(grasp_pose.trajectory_);
      res->trajectories.push_back(pick_up_pose.trajectory_);
      res->trajectories.push_back(place_pose.trajectory_);

      move_group_->clearPoseTargets();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(LOGGER, "Exception in planning: %s", ex.what());
    }
  }

  void sendPlanToUnity(const moveit_msgs::msg::RobotTrajectory &trajectory)
  {
    Xarm7ROSJointsToUnity msg;
    msg.trajectory = trajectory;
    pub_->publish(msg);
    RCLCPP_INFO(LOGGER, "Trajectory sent to Unity");
    is_trajectory_ = false;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Xarm7ROSJointsToUnity>::SharedPtr pub_;
  rclcpp::Subscription<Xarm7UnityTargetPoseToROS>::SharedPtr sub_;
  rclcpp::Service<MoverService>::SharedPtr srv_;
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
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mover_node", node_options);
  auto mover_node = std::make_shared<MoverNode>(node);
  RCLCPP_INFO(node->get_logger(), "mover_node start");
  signal(SIGINT, exit_sig_handler);

  rclcpp::spin(node);
  rclcpp::shutdown();

  RCLCPP_INFO(node->get_logger(), "mover_node over");
  return 0;
}
