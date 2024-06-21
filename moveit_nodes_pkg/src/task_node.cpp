#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <chrono>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick And Place");


class PickAndPlaceTask
{
    public:
        PickAndPlaceTask(rclcpp::Node::SharedPtr &node, std::string &group_name) : node_(node){
            init(group_name);
        }

        void init(const std::string &group_name){
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
            RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
            RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
        }

        // Method to plan to a pose target
        void plan_and_execute_to_pose_goal();
        // Method to generate and execute to a joint space target
        void plan_and_execute_to_joint_space_goal(std::vector<double>& joint_values);
        // Mothod to genearte and execute a cartesian path
        void plan_and_execute_cartesian_path();


    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};


void PickAndPlaceTask::plan_and_execute_to_pose_goal(){

}

void PickAndPlaceTask::plan_and_execute_to_joint_space_goal(std::vector<double>& joint_values){
    bool success = move_group->setJointValueTarget(joint_values);

    if(!success){
        RCLCPP_WARN(node_->get_logger(), "Set JointValueTarget Out of Bounds");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(!success){
        throw std::runtime_error(
            "Trajectory could not be planned. Please ensure target and destination are reachable by the robot.");
    }else{
        move_group->execute(plan);
    }
}

void exit_sig_handler([[maybe_unused]] int signum){
    fprintf(stderr, "[pick_and_place_node] Ctrl+C cought, exit process...\n");
    exit(-1);
}

// Main Function
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true); 

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pick_and_place_node", node_options);

    // arm planning group name
    std::string ARM_GROUP = "xarm7";
    // gripper planning group name
    std::string GRIPPER_GROUP = "xarm_gripper";

    // Arm group object
    // auto arm_node = std::make_shared<PickAndPlaceTask>(node, ARM_GROUP);
    // gripper group object
    auto gripper_node = std::make_shared<PickAndPlaceTask>(node, GRIPPER_GROUP);

    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, -1.570796, 0.0};
    std::vector<double> joint_values_1 = {1.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0};

    std::vector<double> pregrasp_joint_values = {0.0, -0.174533, -0.830777, 0.589921, -0.232129, 0.710349, -0.614357};
    std::vector<double> preplace_joint_values = {0.169297, 0.143117, 0.647517, 0.757473, -0.073304, 0.612611, 0.801106};
    std::vector<double> open_gripper = {0.2};
    std::vector<double> close_gripper = {0.849914};

    // arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);

    // Move to pregrasp position
    // arm_node->plan_and_execute_to_joint_space_goal(pregrasp_joint_values);
    // Open gripper
    gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    // rclcpp::sleep_for(2s);
    // Close gripper
    gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);
    // MMove to the preplace position
    // arm_node->plan_and_execute_to_joint_space_goal(preplace_joint_values);
    // Open gripper
    // gripper_node->plan_and_execute_to_joint_space_goal(open_gripper);
    // Close gripper 
    // gripper_node->plan_and_execute_to_joint_space_goal(close_gripper);
    // Move to home position
    // arm_node->plan_and_execute_to_joint_space_goal(home_joint_values);

    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}