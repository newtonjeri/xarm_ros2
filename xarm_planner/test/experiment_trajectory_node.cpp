/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[experiment_trajectory_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
    return res->success;
}

int main(int argc, char** argv)
{	
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);    
    node = rclcpp::Node::make_shared("experiment_trajectory_node", node_options);
    RCLCPP_INFO(node->get_logger(), "experiment_trajectory_node start");
    signal(SIGINT, exit_sig_handler);

    int dof;
    float z_value = 0.293454;
    node->get_parameter_or("dof", dof, 7);
    RCLCPP_INFO(node->get_logger(), "namespace: %s, dof: %d", node->get_namespace(), dof);

    rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_ = node->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

    std::shared_ptr<xarm_msgs::srv::PlanPose::Request> pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();;
    std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;

    exec_plan_req->wait = true;

    geometry_msgs::msg::Pose target_posehome;
    target_posehome.position.x = 0.399;
	target_posehome.position.y = 0.0;
	target_posehome.position.z = z_value;
	target_posehome.orientation.x = 0.7071;
	target_posehome.orientation.y = 0;
	target_posehome.orientation.z = 0.7071;
	target_posehome.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.4;
	target_pose1.position.y = 0.0;
	target_pose1.position.z = z_value;
	target_pose1.orientation.x = 0.7071;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0.7071;
	target_pose1.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.2586;
	target_pose2.position.y = 0.1;
	target_pose2.position.z = z_value;
	target_pose2.orientation.x = 0.7071;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0.7071;
	target_pose2.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = 0.2;
	target_pose3.position.y = 0;
	target_pose3.position.z = z_value;
	target_pose3.orientation.x = 0.7071;
	target_pose3.orientation.y = 0;
	target_pose3.orientation.z = 0.7071;
	target_pose3.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose4;
    target_pose4.position.x = 0.2586;
	target_pose4.position.y = -0.1;
	target_pose4.position.z = z_value;
	target_pose4.orientation.x = 0.7071;
	target_pose4.orientation.y = 0;
	target_pose4.orientation.z = 0.7071;
	target_pose4.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose5;
    target_pose5.position.x = 0.5414;
	target_pose5.position.y = -0.1;
	target_pose5.position.z = z_value;
	target_pose5.orientation.x = 0.7071;
	target_pose5.orientation.y = 0;
	target_pose5.orientation.z = 0.7071;
	target_pose5.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose6;
    target_pose6.position.x = 0.6;
	target_pose6.position.y = 0;
	target_pose6.position.z = z_value;
	target_pose6.orientation.x = 0.7071;
	target_pose6.orientation.y = 0;
	target_pose6.orientation.z = 0.7071;
	target_pose6.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose7;
    target_pose7.position.x = 0.5414;
	target_pose7.position.y = -0.1;
	target_pose7.position.z = z_value;
	target_pose7.orientation.x = 0.7071;
	target_pose7.orientation.y = 0;
	target_pose7.orientation.z = 0.7071;
	target_pose7.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose8;
    target_pose8.position.x = 0.3;
	target_pose8.position.y = 0.1414;
	target_pose8.position.z = z_value;
	target_pose8.orientation.x = 0.7071;
	target_pose8.orientation.y = 0;
	target_pose8.orientation.z = 0.7071;
	target_pose8.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose9;
    target_pose9.position.x = 0.4;
	target_pose9.position.y = 0.2;
	target_pose9.position.z = z_value;
	target_pose9.orientation.x = 1;
	target_pose9.orientation.y = 0;
	target_pose9.orientation.z = 0;
	target_pose9.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose10;
    target_pose10.position.x = 0.5;
	target_pose10.position.y = 0.1414;
	target_pose10.position.z = z_value;
	target_pose10.orientation.x = 1;
	target_pose10.orientation.y = 0;
	target_pose10.orientation.z = 0;
	target_pose10.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose11;
    target_pose11.position.x = 0.3;
	target_pose11.position.y = -0.1414;
	target_pose11.position.z = z_value;
	target_pose11.orientation.x = 1;
	target_pose11.orientation.y = 0;
	target_pose11.orientation.z = 0;
	target_pose11.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose12;
    target_pose12.position.x = 0.4;
	target_pose12.position.y = -0.2;
	target_pose12.position.z = z_value;
	target_pose12.orientation.x = 1;
	target_pose12.orientation.y = 0;
	target_pose12.orientation.z = 0;
	target_pose12.orientation.w = 0;
    
    geometry_msgs::msg::Pose target_pose13;
    target_pose13.position.x = 0.5;
	target_pose13.position.y = -0.1414;
	target_pose13.position.z = z_value;
	target_pose13.orientation.x = 1;
	target_pose13.orientation.y = 0;
	target_pose13.orientation.z = 0;
	target_pose13.orientation.w = 0;
    // while (rclcpp::ok())
    // {
        pose_plan_req->target = target_pose1;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose2;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose3;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose4;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
                
        pose_plan_req->target = target_pose1;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose7;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose6;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose5;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose1;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
                
        pose_plan_req->target = target_pose8;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_pose9;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);

        // pose_plan_req->target = target_pose10;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);

        // pose_plan_req->target = target_pose1;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);

        // pose_plan_req->target = target_pose11;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);
                
        // pose_plan_req->target = target_pose12;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);
                
        // pose_plan_req->target = target_pose13;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);
                
        // pose_plan_req->target = target_pose13;
        // call_request(pose_plan_client_, pose_plan_req);
        // call_request(exec_plan_client_, exec_plan_req);

        pose_plan_req->target = target_posehome;
        call_request(pose_plan_client_, pose_plan_req);
        call_request(exec_plan_client_, exec_plan_req);


    // }

    RCLCPP_INFO(node->get_logger(), "experiment_trajectory_node over");
    return 0;
}