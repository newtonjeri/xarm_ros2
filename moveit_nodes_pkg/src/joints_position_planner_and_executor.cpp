#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.h>
#include <xarm_msgs/srv/gripper_pose.hpp>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Home pose
double x = 0.28;
double y = -0.2;
double z = 0.5;
double w = 1.0;

class JointsPositionPlannerAndExecutor : public rclcpp::Node
{
public:
  JointsPositionPlannerAndExecutor() : Node(
      "goal_pose_planner_and_executor", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // A service interface to get the gripper pose and return the result of planning and execution
    gripper_pose_service_ = create_service<xarm_msgs::srv::GripperPose>(
        "gripper_pose", std::bind(&JointsPositionPlannerAndExecutor::gripperPoseCallback, this, std::placeholders::_1,
                                  std::placeholders::_2));
  }

  void gripperPoseCallback(const std::shared_ptr<xarm_msgs::srv::GripperPose::Request> request,
                           std::shared_ptr<xarm_msgs::srv::GripperPose::Response> response)
  {
    // Get the request
    x = request->x;
    y = request->y;
    z = request->z;
    w = request->w;

    response->success = true;
  }

private:
  rclcpp::Service<xarm_msgs::srv::GripperPose>::SharedPtr gripper_pose_service_;
};


int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  auto const node = std::make_shared<JointsPositionPlannerAndExecutor>();
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("joints_position_planner_and_executor");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm7");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "xarm7")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = w;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}