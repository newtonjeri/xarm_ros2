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
#include <niryo_moveit_msgs/srv/mover_service.hpp>
#include <niryo_moveit_msgs/msg/niryo_unity_target_pose_to_ros.hpp>
#include <niryo_moveit_msgs/msg/niryo_ros_joints_to_unity.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using MoverService = niryo_moveit_msgs::srv::MoverService;
using MoverServiceRequest = niryo_moveit_msgs::srv::MoverService::Request;
using MoverServiceResponse = niryo_moveit_msgs::srv::MoverService::Response;
using NiryoUnityTargetPoseToROS = niryo_moveit_msgs::msg::NiryoUnityTargetPoseToROS;
using NiryoROSJointsToUnity = niryo_moveit_msgs::msg::NiryoROSJointsToUnity;

class MoverNode : public rclcpp::Node
{
public:
    MoverNode() : Node("mover_node")
    {
        this->declare_parameter<std::string>("group_name", "arm");

        std::string group_name;
        this->get_parameter("group_name", group_name);

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
        move_group_->setPoseReferenceFrame("base_link");

        pub_ = this->create_publisher<NiryoROSJointsToUnity>("niryo_ros_joints_to_unity", 10);
        sub_ = this->create_subscription<NiryoUnityTargetPoseToROS>("niryo_unity_target_pose_to_ros", 10, 
                std::bind(&MoverNode::niryoUnityCallback, this, std::placeholders::_1));
        srv_ = this->create_service<MoverService>("niryo_moveit", std::bind(&MoverNode::planPickAndPlace, this, 
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Ready to plan");
    }

private:
    void niryoUnityCallback(const NiryoUnityTargetPoseToROS::SharedPtr msg)
    {
        if (!planning_)
        {
            planning_ = true;
            try
            {
                motion_plan_ = planTrajectory(msg->target_pose, msg->joints_input.joints, msg->robot_arm_rotation);
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in planning: %s", ex.what());
                planning_ = false;
            }
        }
    }

    moveit::planning_interface::MoveGroupInterface::Plan planTrajectory(const geometry_msgs::msg::Pose &destination_pose, 
                                                                         const std::vector<double> &start_joint_angles, 
                                                                         const geometry_msgs::msg::Vector3 &robot_arm_rotation)
    {
        moveit::core::RobotState start_state(*move_group_->getCurrentState());
        start_state.setJointGroupPositions(joint_names_, start_joint_angles);
        move_group_->setStartState(start_state);

        geometry_msgs::msg::Pose rotated_pose = rotatePose(destination_pose, robot_arm_rotation.x, robot_arm_rotation.y, robot_arm_rotation.z);
        move_group_->setPoseTarget(rotated_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            throw std::runtime_error("Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
        }

        return plan;
    }

    geometry_msgs::msg::Pose rotatePose(const geometry_msgs::msg::Pose &pose, double roll, double pitch, double yaw)
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Isometry3d current_pose = tf2::transformToEigen(tf2::toMsg(pose));
        Eigen::Isometry3d rotation = Eigen::Isometry3d(q);

        Eigen::Isometry3d new_pose = current_pose * rotation;

        geometry_msgs::msg::Pose new_pose_msg = tf2::toMsg(new_pose);

        return new_pose_msg;
    }

    void planPickAndPlace(const MoverServiceRequest::SharedPtr req, MoverServiceResponse::SharedPtr res)
    {
        try
        {
            auto pre_grasp_pose = planTrajectory(req->pick_pose, req->joints_input.joints, req->robot_arm_rotation);
            if (pre_grasp_pose.trajectory_.joint_trajectory.points.empty())
                return;

            auto previous_ending_joint_angles = pre_grasp_pose.trajectory_.joint_trajectory.points.back().positions;

            auto pick_pose = req->pick_pose;
            pick_pose.position.z -= 0.05; // Adjust this value as necessary

            auto grasp_pose = planTrajectory(pick_pose, previous_ending_joint_angles, req->robot_arm_rotation);
            if (grasp_pose.trajectory_.joint_trajectory.points.empty())
                return;

            previous_ending_joint_angles = grasp_pose.trajectory_.joint_trajectory.points.back().positions;

            auto pick_up_pose = planTrajectory(req->pick_pose, previous_ending_joint_angles, req->robot_arm_rotation);
            if (pick_up_pose.trajectory_.joint_trajectory.points.empty())
                return;

            previous_ending_joint_angles = pick_up_pose.trajectory_.joint_trajectory.points.back().positions;

            auto place_pose = planTrajectory(req->place_pose, previous_ending_joint_angles, req->robot_arm_rotation);
            if (place_pose.trajectory_.joint_trajectory.points.empty())
                return;

            res->trajectories.push_back(pre_grasp_pose.trajectory_);
            res->trajectories.push_back(grasp_pose.trajectory_);
            res->trajectories.push_back(pick_up_pose.trajectory_);
            res->trajectories.push_back(place_pose.trajectory_);

            move_group_->clearPoseTargets();
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in planning: %s", ex.what());
        }
    }

    rclcpp::Publisher<NiryoROSJointsToUnity>::SharedPtr pub_;
    rclcpp::Subscription<NiryoUnityTargetPoseToROS>::SharedPtr sub_;
    rclcpp::Service<MoverService>::SharedPtr srv_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<std::string> joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    bool planning_ = false;
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
