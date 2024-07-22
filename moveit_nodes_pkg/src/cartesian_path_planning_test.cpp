#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

geometry_msgs::msg::Pose current_pose;

void plan_and_execute_cartesian_path(
    moveit::planning_interface::MoveGroupInterface &move_group,
    geometry_msgs::msg::Pose &start_pose,
    const int iterations,
    const std::vector<double> direction)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(start_pose);
    RCLCPP_INFO(LOGGER, "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    // waypoints.push_back(start_pose);
    for (int i = 0; i < iterations; i++)
    {
        start_pose.position.x += (direction[0] / iterations);
        start_pose.position.y += (direction[1] / iterations);
        start_pose.position.z += (direction[2] / iterations);
        waypoints.push_back(start_pose);
    }
    RCLCPP_INFO(rclcpp::get_logger("move_group"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    // Define trajectory for the approach
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    // Compute the Cartesian path for the approach
    double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory_approach);
    if (fraction < 0.90)
    {
        RCLCPP_WARN(rclcpp::get_logger("move_group"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
        current_pose = *waypoints.begin();
    }
    else
    {
        // Execute the approach trajectory
        move_group.execute(trajectory_approach);
        current_pose = waypoints.back();
    }
}

void plan_and_execute_to_joint_space_goal(
    moveit::planning_interface::MoveGroupInterface &move_group,
    std::vector<double> &joint_values)
{
    bool success = move_group.setJointValueTarget(joint_values);

    if (!success)
    {
        RCLCPP_WARN(LOGGER, "Set JointValueTarget Out of Bounds");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        throw std::runtime_error(
            "Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
    }
    else
    {
        move_group.execute(plan);
    }
}

void plan_and_execute_to_pose_goal(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target_pose)
{

    bool success = move_group.setPoseTarget(target_pose);

    if (!success)
    {
        RCLCPP_WARN(LOGGER, "Set Target Pose is not reachable!");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        throw std::runtime_error(
            "Trajectory could not be planned. Please ensure target and destination are reachable by the robot");
    }
    else
    {
        move_group.execute(plan);
        current_pose = target_pose;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("cartesian_path_node", options);

    const std::string GROUP_NAME = "xarm7";
    auto arm_group = moveit::planning_interface::MoveGroupInterface(node, GROUP_NAME);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", arm_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", arm_group.getEndEffectorLink().c_str());

    // std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};
    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // std::vector<double> wg_home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796};

    auto start_pose = []
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.399;
        pose.position.y = 0.0;
        pose.position.z = 0.2935;
        pose.orientation.x = 0.707107;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.707107;
        pose.orientation.w = 0.0;
        return pose;
    }();

    auto pregrasp_pose = []
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.310;
        pose.position.y = -0.3;
        pose.position.z = 0.2000;
        // pose.orientation.x = 1.0;
        // pose.orientation.y = 0.0193;
        // pose.orientation.z = -0.0045;
        // pose.orientation.w = -0.0100;

        pose.orientation.x = 0.707107;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.707107;
        pose.orientation.w = 0.0;
        return pose;
    }();

    auto target_pose = []
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.50;
        pose.position.y = 0.35;
        pose.position.z = 0.12000;
        pose.orientation.x = 0.98423;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.17689;
        pose.orientation.w = 0.0;
        return pose;
    }();

    plan_and_execute_to_pose_goal(arm_group, target_pose);

    const std::vector<double> approach = {0.0, 0.0, -0.1};

    const auto retreat = {0.0, 0.0, 0.1};

    for (int i = 0; i < 5; i++){
        plan_and_execute_cartesian_path(arm_group, current_pose, 10, approach);
        plan_and_execute_cartesian_path(arm_group, current_pose, 10, retreat);
    }
        

    plan_and_execute_to_joint_space_goal(arm_group, home_joint_values);
}
