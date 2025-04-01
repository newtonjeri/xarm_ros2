#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& psi) {
    // Create a collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "box";
    collision_object.header.frame_id = "world";  // Adjust to your robot's reference frame

    // Define the shape of the object (box in this case)
    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.5;  // Length
    box_primitive.dimensions[1] = 0.5;  // Width
    box_primitive.dimensions[2] = 0.5;  // Height

    // Define the pose of the box
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;  // Half the height above the ground
    box_pose.orientation.w = 1.0;

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(box_primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add the object to the planning scene
    psi.applyCollisionObject(collision_object);
    RCLCPP_INFO(rclcpp::get_logger("addCollisionObject"), "Added collision object: box");
}

void updateCollisionObjectPose(moveit::planning_interface::PlanningSceneInterface& psi) {
    // Retrieve the current objects in the planning scene
    auto objects = psi.getObjects();

    // Check if the object exists
    if (objects.find("box") == objects.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("updateCollisionObjectPose"), "Object 'box' not found in planning scene.");
        return;
    }

    // Retrieve and update the object's pose
    moveit_msgs::msg::CollisionObject collision_object = objects["box"];
    collision_object.primitive_poses[0].position.x = 2.0;  // New X position
    collision_object.primitive_poses[0].position.y = 0.0;  // New Y position
    collision_object.primitive_poses[0].position.z = 0.5;  // New Z position

    // Ensure the object operation is set to MOVE
    collision_object.operation = moveit_msgs::msg::CollisionObject::MOVE;

    // Reapply the updated object to the planning scene
    psi.applyCollisionObject(collision_object);
    RCLCPP_INFO(rclcpp::get_logger("updateCollisionObjectPose"), "Updated pose of collision object: box");
}

int main(int argc, char** argv) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("planning_scene_example");

    // Create PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface psi;

    // Add a collision object
    addCollisionObject(psi);

    // Wait a moment for the object to be added
    rclcpp::sleep_for(std::chrono::seconds(5));

    // Update the position of the object
    // updateCollisionObjectPose(psi);
    psi.removeCollisionObjects({"base_plate"});

    rclcpp::shutdown();
    return 0;
}
