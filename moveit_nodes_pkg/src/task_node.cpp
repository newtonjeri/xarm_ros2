#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <geometry_msgs/msg/pose.hpp>

#include <chrono>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick And Place");

const double max_velocity_scaling_factor = 0.1;     // [move_group_interface] default is 0.1
const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

class PickAndPlaceTask
{
public:
    PickAndPlaceTask(rclcpp::Node::SharedPtr &node, std::string &group_name) : node_(node)
    {
        init(group_name);
    }

    void init(const std::string &group_name)
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
        // move_group->setEndEffectorLink("link_eef");
        RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
        move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    }

    // Method to plan to a pose target
    void plan_and_execute_to_pose_goal(geometry_msgs::msg::Pose &target_pose);

    // Method to generate and execute to a joint space target
    void plan_and_execute_to_joint_space_goal(std::vector<double> &joint_values);

    // Method to generate and execute a cartesian path
    void plan_and_execute_cartesian_path(
        const geometry_msgs::msg::Pose &start_pose,
        std::vector<geometry_msgs::msg::Pose> &waypoints,
        const double eef_step,
        const double jump_threshold,
        const int iterations,
        const std::vector<double> step);

    // Method to plan and execute a trajectory with equality constrains
    void PickAndPlaceTask::plan_and_execute_with_equality_constraint(
        const geometry_msgs::msg::Pose &start_pose,
        const std::vector<double> plane_dimension,
        const geometry_msgs::msg::Pose plane_pose,
        const geometry_msgs::msg::Pose target_pose);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};

void PickAndPlaceTask::plan_and_execute_to_pose_goal(geometry_msgs::msg::Pose &target_pose)
{
    bool success = move_group->setPoseTarget(target_pose);

    if (!success)
    {
        RCLCPP_WARN(node_->get_logger(), "Set Target Pose is not reachable!");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        throw std::runtime_error(
            "Trajectory could not be planned. Please ensure target and destination are reachable by the robot");
    }
    else
    {
        move_group->execute(plan);
    }
}

void PickAndPlaceTask::plan_and_execute_to_joint_space_goal(std::vector<double> &joint_values)
{
    bool success = move_group->setJointValueTarget(joint_values);

    if (!success)
    {
        RCLCPP_WARN(node_->get_logger(), "Set JointValueTarget Out of Bounds");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        throw std::runtime_error(
            "Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
    }
    else
    {
        move_group->execute(plan);
    }
}

void PickAndPlaceTask::plan_and_execute_cartesian_path(
    const geometry_msgs::msg::Pose &start_pose,
    std::vector<geometry_msgs::msg::Pose> &waypoints,
    const double eef_step,
    const double jump_threshold,
    const int iterations,
    const std::vector<double> step)
{
    waypoints.push_back(start_pose);
    geometry_msgs::msg::Pose pose2 = start_pose;

    for (int i = 0; i < iterations; i++)
    {
        pose2.position.x += step[0];
        pose2.position.y += step[1];
        pose2.position.z += step[2];
        waypoints.push_back(pose2);
    }

    // Define trajectory for the approach
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    // Compute the Cartesian path for the approach
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_approach);
    if (fraction < 0.90)
    {
        RCLCPP_WARN(rclcpp::get_logger("move_group"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
    }

    // Execute the approach trajectory
    move_group->execute(trajectory_approach);
}

/* Continue From Here */
void PickAndPlaceTask::plan_and_execute_with_equality_constraint(
    const geometry_msgs::msg::Pose &start_pose,
    const std::vector<double> line_dimensions,
    const geometry_msgs::msg::Pose line_pose,
    const geometry_msgs::msg::Pose target_pose)
{

    moveit_msgs::msg::PositionConstraint line_constraint;
    line_constraint.header.frame_id = move_group->getPoseReferenceFrame();
    line_constraint.link_name = move_group->getEndEffectorLink();
    shape_msgs::msg::SolidPrimitive line;
    line.type = shape_msgs::msg::SolidPrimitive::BOX;
    line.dimensions = line_dimensions;
    line_constraint.constraint_region.primitives.emplace_back(line);

    ////////////////////////////////////////////////////////////////////
    moveit_msgs::msg::Constraints line_constraints;
    line_constraints.position_constraints.emplace_back(line_constraint);
    line_constraints.name = "use_equality_constraints";
    move_group->setPathConstraints(line_constraints);



    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(10.0);
    move_group->plan(plan);
}

void exit_sig_handler([[maybe_unused]] int signum)
{
    fprintf(stderr, "[pick_and_place_node] Ctrl+C cought, exit process...\n");
    exit(-1);
}

// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pick_and_place_node", node_options);

    /* Variables and constants for cartesian planning */
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;
    int iterations = 1;

    // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    std::string GRIPPER_GROUP = "xarm_gripper";

    // Arm group object
    auto arm_node = std::make_shared<PickAndPlaceTask>(node, ARM_GROUP);
    // gripper group object
    auto gripper_node = std::make_shared<PickAndPlaceTask>(node, GRIPPER_GROUP);

    /* Joint Space targets */
    // std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};
    // std::vector<double> joint_values_1 = {1.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0};

    // std::vector<double> pregrasp_joint_values = {0.0, -0.174533, -0.830777, 0.589921, -0.232129, 0.710349, -0.614357};
    // std::vector<double> preplace_joint_values = {0.169297, 0.143117, 0.647517, 0.757473, -0.073304, 0.612611, 0.801106};
    std::vector<double> open_gripper = {0.2};
    std::vector<double> close_gripper = {849.914};

    /* POSE Target */

    auto home_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.4;
        msg.position.y = 0.0;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto start_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.45;
        msg.position.y = 0.0;
        msg.position.z = 0.293486;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto pregrasp_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3699;
        msg.position.y = -0.2698;
        msg.position.z = 0.1381;
        msg.orientation.x = 0.7106;
        msg.orientation.y = -0.7031;
        msg.orientation.z = 0.0256;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto preplace_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.3095;
        msg.position.y = 0.3372;
        msg.position.z = 0.1374;
        msg.orientation.x = 1.0;
        msg.orientation.y = -0.0291;
        msg.orientation.z = 0.0142;
        msg.orientation.w = -0.021;
        return msg;
    }();

    /* Waypoints */
    std::vector<geometry_msgs::msg::Pose>
        waypoints;

    arm_node->plan_and_execute_to_pose_goal(pregrasp_pose);
    arm_node->plan_and_execute_to_pose_goal(preplace_pose);
    arm_node->plan_and_execute_to_pose_goal(home_pose);
    /* Cartesian path planning */
    std::vector<double> steps = {0.05, 0.0, 0.0};
    iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(start_pose, waypoints, eef_step, jump_threshold, iterations, steps);

    // arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    // // Move to pregrasp position
    // arm_node->plan_and_execute_to_joint_space_goal(pregrasp_joint_values);
    // // Open gripper
    // gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    // // rclcpp::sleep_for(2s);
    // // Close gripper
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);
    // // Move to the preplace position
    // arm_node->plan_and_execute_to_joint_space_goal(preplace_joint_values);
    // // Open gripper
    // gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    // // Close gripper
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);
    // // Move to home position
    // arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);

    // arm_node->plan_and_execute_to_pose_goal(start_pose);
    // gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}