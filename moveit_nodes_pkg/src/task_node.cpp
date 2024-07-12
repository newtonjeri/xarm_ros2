#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <boost/filesystem.hpp>


namespace fs = boost::filesystem;

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick And Place");

const double max_velocity_scaling_factor = 0.07;     // [move_group_interface] default is 0.1
const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

const int number_of_times = 2;

geometry_msgs::msg::Pose current_pose;

class PickAndPlaceTask
{
public:
    PickAndPlaceTask(
        rclcpp::Node::SharedPtr &node,
        std::string &group_name) : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
    {
        init(group_name);
        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());
    }

    void reset_visualisation()
    {
        // Resets the demo by cleaning up any constraints and markers
        move_group->clearPathConstraints();
        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->trigger();
    }

    geometry_msgs::msg::Pose getFramePose(
        const std::string &target_frame,
        const std::string &base_frame,
        const rclcpp::Time &time);

    // Method to initialise and setup the move group interfaces
    void init(const std::string &group_name);

    // Method to plan to a pose target
    void plan_and_execute_to_pose_goal(
        const geometry_msgs::msg::Pose &target_pose,
        bool is_constrained_planning);

    // Method to generate and execute to a joint space target
    void plan_and_execute_to_joint_space_goal(std::vector<double> &joint_values);

    // Method to generate and execute a cartesian path
    void plan_and_execute_cartesian_path(
        geometry_msgs::msg::Pose &start_pose,
        const int iterations,
        const std::vector<double> step);

    // Method to plan and execute a trajectory with equality line constraint
    void plan_and_execute_with_line_equality_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> line_dimensions);

    // Method to plan and execute a trajectory with equality plane constraint
    void plan_and_execute_with_plane_equality_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> plane_dimensions,
        const Eigen::Vector3d &normal,
        double distance);

    // Method to plan and execute a trajectory with box constraints
    void plan_and_execute_with_box_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        const geometry_msgs::msg::Pose box_pose,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> box_dimensions);

    // Method to plan and execute an orientation constrained trajectory

    void plan_and_execute_with_orientation_constraint(
        geometry_msgs::msg::Pose &start_pose,
        const geometry_msgs::msg::Pose &orientation_pose,
        const std::vector<double> relative_position);

    // Method to calculate an offset pose from the current pose
    geometry_msgs::msg::Pose get_relative_pose(
        geometry_msgs::msg::Pose pose,
        const std::vector<double> relative_pos);

    // Function to visualise a plane in rviz
    void visualize_plane(
        const Eigen::Vector3d &normal,
        double distance);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

void PickAndPlaceTask::init(const std::string &group_name)
{
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
    move_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

void PickAndPlaceTask::plan_and_execute_to_pose_goal(
    const geometry_msgs::msg::Pose &target_pose,
    bool is_constrained_planning)
{

    bool success = move_group->setPoseTarget(target_pose);

    if (is_constrained_planning)
    {
        move_group->setPlanningTime(10.0);
    }
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
        current_pose = target_pose;
    }
}

void PickAndPlaceTask::plan_and_execute_to_joint_space_goal(std::vector<double> &joint_values)
{
    reset_visualisation();
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
    geometry_msgs::msg::Pose &start_pose,
    const int iterations,
    const std::vector<double> step)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    RCLCPP_INFO(rclcpp::get_logger("move_group"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    // waypoints.push_back(start_pose);
    for (int i = 0; i < iterations; i++)
    {
        start_pose.position.x += step[0];
        start_pose.position.y += step[1];
        start_pose.position.z += step[2];
        waypoints.push_back(start_pose);
    }
    RCLCPP_INFO(rclcpp::get_logger("move_group"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    // Define trajectory for the approach
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    // Compute the Cartesian path for the approach
    double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory_approach);
    if (fraction < 0.90)
    {
        RCLCPP_WARN(rclcpp::get_logger("move_group"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
    }   

    // Execute the approach trajectory
    move_group->execute(trajectory_approach);
    current_pose = start_pose;
   }

void PickAndPlaceTask::plan_and_execute_with_line_equality_constraint(
    const geometry_msgs::msg::Pose start_pose,
    const geometry_msgs::msg::Pose target_pose,
    rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> line_dimensions)
{
    // Clear path constraints after planning and execution
    reset_visualisation();
    moveit_msgs::msg::PositionConstraint line_constraint;
    line_constraint.header.frame_id = move_group->getPoseReferenceFrame();
    // line_constraint.header.frame_id = move_group->getEndEffectorLink();
    line_constraint.link_name = move_group->getEndEffectorLink();

    shape_msgs::msg::SolidPrimitive line;
    line.type = shape_msgs::msg::SolidPrimitive::BOX;
    line.dimensions = line_dimensions;
    line_constraint.constraint_region.primitives.emplace_back(line);

    // line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
    line_constraint.constraint_region.primitive_poses.emplace_back(start_pose);
    line_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints line_constraints;
    line_constraints.position_constraints.emplace_back(line_constraint);
    line_constraints.name = "use_equality_constraints";
    move_group->setPathConstraints(line_constraints);

    // Define the target pose with an offset from the start pose
    auto target = get_relative_pose(
        start_pose,
        {target_pose.position.x - start_pose.position.x,
         target_pose.position.y - start_pose.position.y,
         target_pose.position.z - start_pose.position.z});

    // Visualise line
    moveit_visual_tools_->publishLine(start_pose.position, target.position, rviz_visual_tools::TRANSLUCENT_DARK);
    moveit_visual_tools_->trigger();

    // move_group->setPoseReferenceFrame(move_group->getEndEffectorLink());
    // Set the pose target and planning parameters
    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(10.0);
    // plan_and_execute_to_pose_goal(target_pose, true);
    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan with line constraint %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
        move_group->execute(plan);
        current_pose = target_pose;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed, execution skipped.");
    }

}

void PickAndPlaceTask::plan_and_execute_with_plane_equality_constraint(
    const geometry_msgs::msg::Pose start_pose,
    const geometry_msgs::msg::Pose target_pose,
    rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> plane_dimensions,
    const Eigen::Vector3d &normal,
    double distance)

{
    // Clear path constraints after planning and execution
    reset_visualisation();
    // Define the position constraint
    moveit_msgs::msg::PositionConstraint plane_constraint;
    plane_constraint.header.frame_id = move_group->getPoseReferenceFrame();
    plane_constraint.link_name = move_group->getEndEffectorLink();

    // Define the plane as a very thin box
    shape_msgs::msg::SolidPrimitive plane;
    plane.type = shape_msgs::msg::SolidPrimitive::BOX;
    plane.dimensions = plane_dimensions;
    plane_constraint.constraint_region.primitives.emplace_back(plane);

    // Add the pose for the plane constraint
    plane_constraint.constraint_region.primitive_poses.emplace_back(target_pose);
    plane_constraint.weight = 1.0;

    // Set the path constraints
    moveit_msgs::msg::Constraints plane_constraints;
    plane_constraints.position_constraints.emplace_back(plane_constraint);
    plane_constraints.name = "use_equality_constraints";
    move_group->setPathConstraints(plane_constraints);

    // Define the target pose with a relative offset
    auto target = get_relative_pose(
        start_pose,
        {target_pose.position.x - start_pose.position.x,
         target_pose.position.y - start_pose.position.y,
         target_pose.position.z - start_pose.position.z});

    // Visualize the plane
    moveit_visual_tools_->publishNormalAndDistancePlane(normal, distance, rviz_visual_tools::TRANSLUCENT_DARK);
    moveit_visual_tools_->trigger();
    // Set the pose target and planning parameters
    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(10.0);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan with plane constraint %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
        move_group->execute(plan);
        current_pose = target;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed, execution skipped.");
    }

}

void PickAndPlaceTask::plan_and_execute_with_box_constraint(
    const geometry_msgs::msg::Pose start_pose,
    const geometry_msgs::msg::Pose target_pose,
    const geometry_msgs::msg::Pose box_pose_,
    rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> box_dimensions)
{
    // Clear path constraints after planning and execution
    reset_visualisation();

    // Determine the box constraints to be used by checking th target pos
    if(target_pose.position.z - start_pose.position.z > 0.05 || target_pose.position.z - start_pose.position.z < -0.05){
        box_dimensions[0] = 0.5;
        // box_dimensions[1] = 0.5;
        box_dimensions[2] = 0.5;
    }
    geometry_msgs::msg::Pose box_pose = getFramePose("link3", "world", rclcpp::Time(0));

    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = move_group->getPoseReferenceFrame();
    // box_constraint.link_name = move_group->getEndEffectorLink();
    box_constraint.link_name = "link3";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = box_dimensions;
    box_constraint.constraint_region.primitives.emplace_back(box);

    box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    box_constraint.weight = 1.0;

    // Visualize the box constraint
    Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                                box_pose.position.z - box.dimensions[2] / 2);
    Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                                box_pose.position.z + box.dimensions[2] / 2);
    moveit_visual_tools_->publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
    moveit_visual_tools_->trigger();

    moveit_msgs::msg::Constraints box_constraints;
    box_constraints.position_constraints.emplace_back(box_constraint);
    move_group->setPathConstraints(box_constraints);

    // Define the target pose with a relative offset
     auto target = get_relative_pose(
        start_pose,
        {target_pose.position.x - start_pose.position.x,
         target_pose.position.y - start_pose.position.y,
         target_pose.position.z - start_pose.position.z});

    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(10.0);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan with plane constraint %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
        move_group->execute(plan);
        current_pose = target_pose;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed, execution skipped.");
    }

}

    void PickAndPlaceTask::plan_and_execute_with_orientation_constraint(
        geometry_msgs::msg::Pose & start_pose,
        const geometry_msgs::msg::Pose &orientation_pose,
        const std::vector<double> relative_position)
    {
        // Clear path constraints after the move
        reset_visualisation();

        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group->getEndEffectorLink();
        orientation_constraint.orientation = orientation_pose.orientation;
        orientation_constraint.absolute_x_axis_tolerance = 0.4;
        orientation_constraint.absolute_y_axis_tolerance = 0.4;
        orientation_constraint.absolute_z_axis_tolerance = 0.4;
        orientation_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints orientation_constraints;
        orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

        auto target = get_relative_pose(start_pose, relative_position);
        move_group->setPathConstraints(orientation_constraints);
        move_group->setPoseTarget(target);
        move_group->setPlanningTime(10.0);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Planning with constraints %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->move();
            current_pose = target;
        }
}

// Function to set a relative position to be used for orientation constrained planning
geometry_msgs::msg::Pose PickAndPlaceTask::get_relative_pose(
    geometry_msgs::msg::Pose pose,
    const std::vector<double> relative_pos)
{
    auto local_current_pose = pose;
    // Creates a pose at a given positional offset from the current pose
    auto get_relative_pose = [local_current_pose, this](double x, double y, double z)
    {
        auto target_pose = local_current_pose;
        target_pose.position.x += x;
        target_pose.position.y += y;
        target_pose.position.z += z;
        this->moveit_visual_tools_->publishSphere(local_current_pose, rviz_visual_tools::RED, 0.05);
        this->moveit_visual_tools_->publishSphere(target_pose, rviz_visual_tools::GREEN, 0.05);
        this->moveit_visual_tools_->trigger();
        return target_pose;
    };

    auto relative_pose = get_relative_pose(relative_pos[0], relative_pos[1], relative_pos[2]);
    return relative_pose;
}

void PickAndPlaceTask::visualize_plane(const Eigen::Vector3d &normal, double distance)
{
    moveit_visual_tools_->publishNormalAndDistancePlane(normal, distance, rviz_visual_tools::TRANSLUCENT_DARK);
    moveit_visual_tools_->trigger();
}

geometry_msgs::msg::Pose PickAndPlaceTask::getFramePose(
    const std::string& target_frame,
    const std::string& base_frame,
    const rclcpp::Time& time = rclcpp::Time(0)
){
    geometry_msgs::msg::Pose pose;
    try{
        // Lookup the transform from target_frame to base_frame
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                                                                                base_frame, 
                                                                                target_frame, 
                                                                                time, 
                                                                                tf2::durationFromSec(1.0));

        // Extraact the translation and rotation
        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;
        pose.orientation = transform_stamped.transform.rotation;
    }
    catch(tf2::TransformException &ex)
    {

        RCLCPP_WARN(LOGGER, "Could not transform %s to %s: %s", target_frame.c_str(), base_frame.c_str(), ex.what());
    }
    return pose;
}

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%m/%d/%Y %H:%M:%S");
    ss << '.' << std::setw(3) << std::setfill('0') << milliseconds.count();
    return ss.str();
}

std::string getFilename() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << "data/log_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << milliseconds.count() << ".csv";
    return ss.str();
}

void saveToCSV(std::string filename, std::string cycle) {
    // Ensure the data directory exists
    fs::create_directory("data");
    std::ofstream csvfile;

    // Check if file exists and open in append mode if it does
    if (fs::exists(filename)) {
        csvfile.open(filename, std::ios::app);
    } else {
        csvfile.open(filename);
        csvfile << "Cycle start Time" << std::endl; // Write the header if the file is new
    }

    if (!csvfile.is_open()) {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
        return;
    }

    std::string timestamp = getCurrentTimestamp();
    csvfile << cycle << ". "<<timestamp << std::endl;

    csvfile.close();
    std::cout << "Data saved to " << filename << std::endl;
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
    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> wg_home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796};
    // std::vector<double> joint_values_1 = {1.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0};

    std::vector<double> pregrasp_joint_values = {0.0, -0.174533, -0.830777, 0.589921, -0.232129, 0.710349, -0.614357};
    std::vector<double> preplace_joint_values = {0.169297, 0.143117, 0.647517, 0.757473, -0.073304, 0.612611, 0.801106};
    std::vector<double> open_gripper = {0.2};
    std::vector<double> close_gripper = {0.84};

    auto link3_home_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;
        msg.position.y = 0.0;
        msg.position.z = 0.56;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();

    auto pregrasp_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.310;
        msg.position.y = -0.2260;
        msg.position.z = 0.1000;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0193;
        msg.orientation.z = -0.0045;
        msg.orientation.w = -0.0100;
        return msg;
    }();

    auto preplace_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.310;
        msg.position.y = 0.2660;
        msg.position.z = 0.1000;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0193;
        msg.orientation.z = -0.0045;
        msg.orientation.w = -0.0100;
        // msg.orientation.x = 0.707107;
        // msg.orientation.y = 0.0;
        // msg.orientation.z = 0.707107;
        // msg.orientation.w = 0.0;

        return msg;
    }();

        auto target_poseC = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;
        msg.position.y = 0.5;
        msg.position.z = 0.1;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0193;
        msg.orientation.z = -0.0045;
        msg.orientation.w = -0.0100;
        return msg;
    }();

    auto box_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.45;
        msg.position.y = 0.0;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();

    /* Perpendicular to Z axis */
    auto plane_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.399;
        msg.position.y = 0.0;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();

    auto line_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.227;
        msg.position.y = 0.0;
        msg.position.z = 0.294;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        return msg;
    }();

    auto wg_home_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.227;
        msg.position.y = 0.0;
        msg.position.z = 0.294;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();


    // /////////////////////////////////////////////////////////////////////////////////////

    auto home_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.399;
        msg.position.y = 0.0;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto move_x_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.659;
        msg.position.y = 0.0;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto move_y_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.399;
        msg.position.y = 0.3;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto move_z_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.399;
        msg.position.y = 0.0;
        msg.position.z = 0.45;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto move_xy_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.399;
        msg.position.y = -0.3;
        msg.position.z = 0.2935;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

    auto move_xy_home = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.499;
        msg.position.y = 0.0;
        msg.position.z = 0.5;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();

        auto move_xyz_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.2;
        msg.position.y = 0.2;
        msg.position.z = 0.2;
        msg.orientation.x = 0.707107;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.707107;
        msg.orientation.w = 0.0;
        return msg;
    }();
    /* Waypoints */
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // /* Cartesian path planning */
    // std::vector<double> steps = {0.05, 0.0, 0.0};
    // iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(home_pose, waypoints, eef_step, jump_threshold, iterations, steps);

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

    // arm_node->plan_and_execute_to_pose_goal(home_pose, false);
    // gripper_node->plan_and_execute_to_joint_space_goal(open_gripper); 
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    /* Line Constraint test */
    // arm_node->plan_and_execute_with_line_equality_constraint(home_pose, move_x_pose, {0.0005, 0.0005, 1.0});
    // arm_node->plan_and_execute_with_line_equality_constraint(home_pose, move_y_pose, {0.0005, 1.0, 0.0005});
    // arm_node->plan_and_execute_with_line_equality_constraint(home_pose, move_z_pose, {1.0, 0.0005, 0.0005});

    /* Plane Constraint test */
    // arm_node->plan_and_execute_to_pose_goal(move_xy_home, false);
    // arm_node->plan_and_execute_with_plane_equality_constraint(move_xy_home, move_xy_pose, {0.0005, 1.0, 1.0}, {0, 0, 1}, move_xy_home.position.z);


    /* Box constraint test */
    // arm_node->plan_and_execute_to_pose_goal(home_pose, false);
    // arm_node->plan_and_execute_with_box_constraint(home_pose, move_xy_home, {0.6, 0.4, 0.4});

    /* Orientation Constraint test */
    // arm_node->plan_and_execute_with_line_equality_constraint(home_pose,start_pose, {0.0005, 0.0005, 1.0});
    // arm_node->plan_and_execute_with_line_equality_constraint(home_pose,move_y_pose, {0.0005, 1.0, 0.0005});
    // std::string filename_ = getFilename();
    // for (int i = 0; i < number_of_times; i++)
    // {
    //     /* Line Constraint test */
    //     RCLCPP_INFO(LOGGER, "PLANNING AND EXECUTING TO A TARGET ATTEMPT - %d", i+1);
    //     // saveToCSV(filename_, std::to_string(i+1));
    //     // arm_node->plan_and_execute_with_line_equality_constraint(home_pose, move_x_pose, {0.0005, 0.0005, 1.0});
    //     // arm_node->plan_and_execute_with_plane_equality_constraint(home_pose, move_x_pose, {0.0005, 1.0, 1.0}, {0, 0, 1}, move_x_pose.position.z);
    //     arm_node->plan_and_execute_with_box_constraint(home_pose, move_xyz_pose, link3_home_pose, {0.3, 0.3, 0.3});
    //     arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    // }

    /* TASK */
    // Make sure the robot is at home pose
    arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    RCLCPP_INFO(LOGGER, "Robot At Home POSE - 1");
    // Move to top of picking position
    arm_node->plan_and_execute_with_box_constraint(home_pose, pregrasp_pose, link3_home_pose, {0.2, 0.2, 0.2});
    RCLCPP_INFO(LOGGER, "Robot At top of Picking POSE - 2");
    // Move to picking position via cartesian path planning
    /* Cartesian path planning */
    // std::vector<double> steps = {0.0, 0.0, -0.05};
    // iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(pregrasp_pose, iterations, steps);
    // // Move retreat from picking position using cartesian path planning
    // steps = {0.0, 0.0, 0.0};
    // arm_node->plan_and_execute_cartesian_path(current_pose, iterations, steps);

    // Move to top of placing position with constrained orientation
    arm_node->plan_and_execute_with_box_constraint(current_pose, preplace_pose, link3_home_pose, {0.2, 0.2, 0.2});
    RCLCPP_INFO(LOGGER, "Robot At top of Placing POSE - 3");

    // Move back to top of picking position
    arm_node->plan_and_execute_with_box_constraint(current_pose, pregrasp_pose, link3_home_pose, {0.2, 0.2, 0.2});
    RCLCPP_INFO(LOGGER, "Robot At top of Picking POSE - 4");

    // Move to top of placing position with constrained orientation
    arm_node->plan_and_execute_with_box_constraint(current_pose, target_poseC, link3_home_pose, {0.2, 0.2, 0.2});
    RCLCPP_INFO(LOGGER, "Robot At top POSE A - 5");

    // Move back to top of picking position
    arm_node->plan_and_execute_with_box_constraint(current_pose, pregrasp_pose, link3_home_pose, {0.2, 0.2, 0.2});
    RCLCPP_INFO(LOGGER, "Robot At top of Picking POSE - 6");
    // // Move to placing position with cartesian path
    // steps = {0.02, 0.0, 0.0};
    // iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(preplace_pose, iterations, steps);
    // // Move retreat from picking position using cartesian path planning
    // steps = {-0.02, 0.0, 0.05};
    // arm_node->plan_and_execute_cartesian_path(current_pose, iterations, steps);
    // Move to home position
    arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}