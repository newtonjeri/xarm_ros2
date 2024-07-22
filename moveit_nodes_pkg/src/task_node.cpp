#include "moveit_nodes_pkg/task_node.h"

namespace fs = boost::filesystem;
using namespace std::chrono_literals;

namespace task_node
{
    // static const rclcpp::Logger LOGGER_ = rclcpp::get_logger("Pick And Place");

    const double max_velocity_scaling_factor = 0.07;    // [move_group_interface] default is 0.1
    const double max_acceleration_scaling_factor = 0.1; // [move_group_interface] default is 0.1

    geometry_msgs::msg::Pose current_pose;

    PickAndPlaceTask::PickAndPlaceTask(
        rclcpp::Node::SharedPtr &node,
        std::string &group_name) : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
    {
        init(group_name);
        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());
    }

    void PickAndPlaceTask::reset_visualisation()
    {
        // Resets the demo by cleaning up any constraints and markers
        move_group->clearPathConstraints();
        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->trigger();
    }

    void PickAndPlaceTask::init(const std::string &group_name)
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "End effector link: %s", move_group->getEndEffectorLink().c_str());
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
            RCLCPP_WARN(rclcpp::get_logger("pick_and_place_node"), "Set Target Pose is not reachable!");
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
            RCLCPP_WARN(rclcpp::get_logger("pick_and_place_node"), "Set JointValueTarget Out of Bounds");
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
        const geometry_msgs::msg::Pose &start_pose_,
        const int iterations,
        const std::vector<double> direction)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose start_pose = start_pose_;
        waypoints.push_back(start_pose);
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
        // waypoints.push_back(start_pose);
        for (int i = 0; i < iterations; i++)
        {
            start_pose.position.x += (direction[0] / iterations);
            start_pose.position.y += (direction[1] / iterations);
            start_pose.position.z += (direction[2] / iterations);
            waypoints.push_back(start_pose);
        }
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "x = %f, y = %f, z = %f, ox = %f, oy = %f, oz = %f, ow = %f",
                    start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
        // Define trajectory for the approach
        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        // Compute the Cartesian path for the approach
        double fraction = move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory_approach);
        if (fraction < 0.90)
        {
            RCLCPP_WARN(rclcpp::get_logger("pick_and_place_node"), "Cartesian path for approach not fully planned, fraction: %f", fraction);
            current_pose = *waypoints.begin();
        }
        else
        {
            // Execute the approach trajectory
            move_group->execute(trajectory_approach);
            current_pose = waypoints.back();
        }
    }

    void PickAndPlaceTask::plan_and_execute_with_line_equality_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        const std::string target_link,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> line_dimensions)
    {
        // Clear path constraints after planning and execution
        reset_visualisation();

        geometry_msgs::msg::Pose line_pose = getFramePose(target_link, "world", rclcpp::Time(0));

        auto line_pose_end = line_pose;
        line_pose_end.position.x = line_dimensions[2];
        moveit_msgs::msg::PositionConstraint line_constraint;
        line_constraint.header.frame_id = move_group->getPoseReferenceFrame();
        // line_constraint.header.frame_id = move_group->getEndEffectorLink();
        line_constraint.link_name = target_link;

        shape_msgs::msg::SolidPrimitive line;
        line.type = shape_msgs::msg::SolidPrimitive::BOX;
        line.dimensions = line_dimensions;
        line_constraint.constraint_region.primitives.emplace_back(line);

        line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
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
        moveit_visual_tools_->publishLine(line_pose.position, line_pose_end.position, rviz_visual_tools::TRANSLUCENT_DARK);
        moveit_visual_tools_->trigger();

        // move_group->setPoseReferenceFrame(move_group->getEndEffectorLink());
        // Set the pose target and planning parameters
        move_group->setPoseTarget(target_pose);
        move_group->setPlanningTime(10.0);
        // plan_and_execute_to_pose_goal(target_pose, true);
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Plan with line constraint %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->execute(plan);
            current_pose = target_pose;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pick_and_place_node"), "Planning failed, execution skipped.");
        }
    }

    void PickAndPlaceTask::plan_and_execute_with_plane_equality_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        const std::string target_link,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> plane_dimensions,
        const Eigen::Vector3d &normal,
        double distance)

    {
        // Clear path constraints after planning and execution
        reset_visualisation();

        const std::string reference_link = move_group->getPoseReferenceFrame();

        // Get POSE of the target link
        geometry_msgs::msg::Pose plane_pose = getFramePose(target_link, reference_link, rclcpp::Time(0));

        // Define the position constraint
        moveit_msgs::msg::PositionConstraint plane_constraint;
        plane_constraint.header.frame_id = reference_link;
        plane_constraint.link_name = target_link;

        // Define the plane as a very thin box
        shape_msgs::msg::SolidPrimitive plane;
        plane.type = shape_msgs::msg::SolidPrimitive::BOX;
        plane.dimensions = plane_dimensions;
        plane_constraint.constraint_region.primitives.emplace_back(plane);

        // Add the pose for the plane constraint
        plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
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

        distance = plane_pose.position.z;
        // Visualize the plane
        moveit_visual_tools_->publishNormalAndDistancePlane(normal, distance, rviz_visual_tools::TRANSLUCENT_DARK);
        moveit_visual_tools_->trigger();
        // Set the pose target and planning parameters
        move_group->setPoseTarget(target_pose);
        move_group->setPlanningTime(10.0);

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Plan with plane constraint %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->execute(plan);
            current_pose = target;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pick_and_place_node"), "Planning failed, execution skipped.");
        }
    }

    void PickAndPlaceTask::plan_and_execute_with_box_constraint(
        const geometry_msgs::msg::Pose start_pose,
        const geometry_msgs::msg::Pose target_pose,
        const std::string link_name,
        rosidl_runtime_cpp::BoundedVector<double, 3UL, std::allocator<double>> box_dimensions)
    {
        // Clear path constraints after planning and execution
        reset_visualisation();

        // Determine the box constraints to be used by checking th target pos
        if (target_pose.position.z - start_pose.position.z > 0.05 || target_pose.position.z - start_pose.position.z < -0.05)
        {
            box_dimensions[0] = 0.5;
            box_dimensions[2] = 0.5;
        }
        geometry_msgs::msg::Pose box_pose = getFramePose(link_name, "world", rclcpp::Time(0));

        moveit_msgs::msg::PositionConstraint box_constraint;
        box_constraint.header.frame_id = move_group->getPoseReferenceFrame();
        // box_constraint.link_name = move_group->getEndEffectorLink();
        box_constraint.link_name = link_name;

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
        // move_group->setPlanningTime(10.0);

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Plan with plane constraint %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->execute(plan);
            current_pose = target_pose;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pick_and_place_node"), "Planning failed, execution skipped.");
        }
    }

    void PickAndPlaceTask::plan_and_execute_with_orientation_constraint(
        geometry_msgs::msg::Pose &start_pose,
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
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Planning with constraints %s", success ? "SUCCEEDED" : "FAILED");

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
        const std::string &target_frame,
        const std::string &base_frame,
        const rclcpp::Time &time = rclcpp::Time(0))
    {
        geometry_msgs::msg::Pose pose;
        try
        {
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
        catch (tf2::TransformException &ex)
        {

            RCLCPP_WARN(rclcpp::get_logger("pick_and_place_node"), "Could not transform %s to %s: %s", target_frame.c_str(), base_frame.c_str(), ex.what());
        }
        return pose;
    }
}

std::string getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%m/%d/%Y %H:%M:%S");
    ss << '.' << std::setw(3) << std::setfill('0') << milliseconds.count();
    return ss.str();
}

std::string getFilename()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << "data/log_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << milliseconds.count() << ".csv";
    return ss.str();
}

void saveToCSV(std::string filename, std::string cycle)
{
    // Ensure the data directory exists
    fs::create_directory("data");
    std::ofstream csvfile;

    // Check if file exists and open in append mode if it does
    if (fs::exists(filename))
    {
        csvfile.open(filename, std::ios::app);
    }
    else
    {
        csvfile.open(filename);
        csvfile << "Cycle start Time" << std::endl; // Write the header if the file is new
    }

    if (!csvfile.is_open())
    {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
        return;
    }

    std::string timestamp = getCurrentTimestamp();
    csvfile << cycle << ". " << timestamp << std::endl;

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
    auto arm_node = std::make_shared<task_node::PickAndPlaceTask>(node, ARM_GROUP);
    // gripper group object
    auto gripper_node = std::make_shared<task_node::PickAndPlaceTask>(node, GRIPPER_GROUP);

    /* Joint Space targets */
    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};
    // std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> wg_home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796};
    // std::vector<double> joint_values_1 = {1.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0};

    std::vector<double> pregrasp_joint_values = {0.0, -0.174533, -0.830777, 0.589921, -0.232129, 0.710349, -0.614357};
    std::vector<double> preplace_joint_values = {0.169297, 0.143117, 0.647517, 0.757473, -0.073304, 0.612611, 0.801106};
    std::vector<double> open_gripper(6, 0.0);
    std::vector<double> close_gripper(6, 0.45);

    const auto home_pose = []
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

    const auto pregrasp_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.310;
        msg.position.y = -0.2260;
        msg.position.z = 0.2000;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    const auto preplace_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.310;
        msg.position.y = 0.2660;
        msg.position.z = 0.2000;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    const auto target_poseC = []
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;
        msg.position.y = 0.5;
        msg.position.z = 0.2;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        return msg;
    }();

    /* Waypoints */
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // /* Cartesian path planning */
    // std::vector<double> direction = {0.05, 0.0, 0.0};
    // iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(home_pose, waypoints, eef_step, jump_threshold, iterations, direction);

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
    // arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At Home POSE - 1");
    // arm_node->plan_and_execute_with_plane_equality_constraint(home_pose, pregrasp_pose, "link3", {1.0, 0.0005, 1.0}, {0, 0, 1}, pregrasp_pose.position.z);
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
    //     RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "PLANNING AND EXECUTING TO A TARGET ATTEMPT - %d", i+1);
    //     arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    //     RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At Home POSE - 1");
    //     // arm_node->plan_and_execute_with_plane_equality_constraint(home_pose, move_xy_pose, "link3", {1.0, 0.0005, 0.5}, {0, 0, 1}, pregrasp_pose.position.z);
    //     // arm_node->plan_and_execute_with_line_equality_constraint(home_pose, move_x_pose, "link4", {0.0005, 1.0, 0.0005});

    //     arm_node->plan_and_execute_with_box_constraint(home_pose, pregrasp_pose, "link3", {0.2, 0.2, 0.2});
    //     arm_node->plan_and_execute_with_box_constraint(current_pose, target_poseC, "link4", {0.2, 0.2, 0.2});
    // }

    /* TASK */
    // Make sure the robot is at home pose
    arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At Home POSE - 1");
    // Move to top of picking position
    arm_node->plan_and_execute_with_box_constraint(home_pose, pregrasp_pose, "link3", {0.2, 0.2, 0.2});
    RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top of Picking POSE - 2");
    // Move to picking position via cartesian path planning
    /* Cartesian path planning */
    std::vector<double> direction = {0.0, 0.0, -0.1};
    iterations = 1;
    // Approach the picking position using cartesian path planning
    arm_node->plan_and_execute_cartesian_path(pregrasp_pose, iterations, direction);

    gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    rclcpp::sleep_for(2s);
    gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    // Retreat from picking position using cartesian path planning
    direction = {0.0, 0.0, 0.1};
    arm_node->plan_and_execute_cartesian_path(task_node::current_pose, iterations, direction);

    // Move to top of placing position with constrained orientation
    arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, preplace_pose, "link3", {0.2, 0.2, 0.2});
    RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top of Placing POSE - 3");
    /* Cartesian path planning */
    direction = {0.0, 0.0, -0.1};
    iterations = 1;
    // Approach the placing position using cartesian path planning
    arm_node->plan_and_execute_cartesian_path(preplace_pose, iterations, direction);

    gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    rclcpp::sleep_for(2s);
    gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    // Retreat from placing position using cartesian path planning
    direction = {0.0, 0.0, 0.1};
    arm_node->plan_and_execute_cartesian_path(task_node::current_pose, iterations, direction);
    // Move back to top of picking position
    arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, pregrasp_pose, "link3", {0.2, 0.2, 0.2});
    RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top of Picking POSE - 4");

    // // Move back to top of picking position
    // arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, pregrasp_pose, "link4", {0.2, 0.2, 0.2});
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top of Picking POSE - 5");

    // // Move to intermediate pose with constrained orientation
    // arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, intermediate_pose, "link4", {0.2, 0.2, 0.2});
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top POSE A - 6");

    // // Move to top of placing position with constrained orientation
    // arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, target_poseC, "link4", {0.2, 0.2, 0.2});
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top POSE A - 7");

    // // Move to intermediate pose with constrained orientation
    // arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, intermediate_pose, "link4", {0.2, 0.2, 0.2});
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top POSE A - 8");

    // // Move back to top of picking position
    // arm_node->plan_and_execute_with_box_constraint(task_node::current_pose, pregrasp_pose, "link4", {0.2, 0.2, 0.2});
    // RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Robot At top of Picking POSE - 9");

    // Move to placing position with cartesian path
    // direction = {0.02, 0.0, 0.0};
    // iterations = 1;
    // arm_node->plan_and_execute_cartesian_path(preplace_pose, iterations, direction);
    // // Move retreat from picking position using cartesian path planning
    // direction = {-0.02, 0.0, 0.05};
    // arm_node->plan_and_execute_cartesian_path(task_node::current_pose, iterations, direction);
    // Move to home position
    arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}