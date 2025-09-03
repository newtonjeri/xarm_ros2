/* Copyright 2024 Virtual Reality Labs DeKUT All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Newton Kariuki <newtonkaris45@gmail.com>
 ============================================================================*/


#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


void scaleMesh(shapes::Mesh* mesh, double scale_factor_x, double scale_factor_y, double scale_factor_z) {
    for (size_t i = 0; i < mesh->vertex_count; ++i) {
        mesh->vertices[3 * i] *= scale_factor_x;     // Scale X coordinate
        mesh->vertices[3 * i + 1] *= scale_factor_y; // Scale Y coordinate
        mesh->vertices[3 * i + 2] *= scale_factor_z; // Scale Z coordinate
    }
}

void addCollisionObjectsToScene(
    std::string file_path,
    std::string part_name,
    std::string header_frame_id,
    geometry_msgs::msg::Pose object_pose,
    std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
    std::vector<moveit_msgs::msg::ObjectColor>& object_colors,
    const std_msgs::msg::ColorRGBA& color,
    double scale_factor_x = 0.00075,
    double scale_factor_y = 0.00075,
    double scale_factor_z = 0.00075)
{
    // Load the collision object mesh file
    shapes::Mesh* collision_object_mesh = shapes::createMeshFromResource(file_path);
    
    // Check if mesh is loaded successfully
    if (!collision_object_mesh) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load mesh from %s", file_path.c_str());
        return;
    }

    // Scale the mesh
    scaleMesh(collision_object_mesh, scale_factor_x, scale_factor_y, scale_factor_z);
    shapes::ShapeMsg collision_object_msg;
    shapes::constructMsgFromShape(collision_object_mesh, collision_object_msg);
    shape_msgs::msg::Mesh collision_object_mesh_msg = boost::get<shape_msgs::msg::Mesh>(collision_object_msg);

    // Create a collision object
    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.id = part_name;
    collision_object.header.frame_id = header_frame_id;

    collision_object.meshes[0] = collision_object_mesh_msg;
    collision_object.mesh_poses[0] = object_pose;

    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    // Add color for this object
    moveit_msgs::msg::ObjectColor object_color;
    object_color.id = part_name;
    object_color.color = color;
    object_colors.push_back(object_color);

    // Clear the collision object mesh from memory
    delete collision_object_mesh;
}

void exit_sig_handler([[maybe_unused]] int signum)
{
    fprintf(stderr, "[pick_and_place_node] Ctrl+C cought, exit process...\n");
    exit(-1);
}

int main(int argc, char ** argv){
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions mode_options;
    mode_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("collision_object_adder_node", mode_options);
    RCLCPP_INFO(node->get_logger(), "Collision Object Adder Node Started");

    // Create a planning scene interface
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::vector<moveit_msgs::msg::ObjectColor> object_colors;

    double scale_factor = 0.00075;

    // Add the gears to the scene

    // Define the file path and frame ID of the collision object mesh
    std::string base_plate = "package://moveit_nodes_pkg/gear_box/BASE_PLATE.stl";
    std::string cover_plate = "package://moveit_nodes_pkg/gear_box/COVER_PLATE.stl";
    std::string spindle_2 = "package://moveit_nodes_pkg/gear_box/SPINDLE_1.stl";
    std::string idler_gear = "package://moveit_nodes_pkg/gear_box/IDLER_GEAR.stl";
    std::string pinion_gear = "package://moveit_nodes_pkg/gear_box/PINION.stl";
    std::string header_frame_id = "world";

    // Define colors
    std_msgs::msg::ColorRGBA dark_gray;
    dark_gray.r = 0.1;  // Red channel
    dark_gray.g = 0.1;  // Green channel
    dark_gray.b = 0.1;  // Blue channel
    dark_gray.a = 1.0;  // Alpha (opacity)

    std_msgs::msg::ColorRGBA gray;
    gray.r = 0.5;
    gray.g = 0.5;
    gray.b = 0.5;
    gray.a = 1.0;

    // Define the pose of the collision objects
    geometry_msgs::msg::Pose base_plate_pose;
    base_plate_pose.position.x = 0.3665;
    base_plate_pose.position.y = -0.3788;
    base_plate_pose.position.z = 0.005;
    base_plate_pose.orientation.x = 0.0;
    base_plate_pose.orientation.y = 0.0;
    base_plate_pose.orientation.z = 0.0;
    base_plate_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose cover_plate_pose;
    cover_plate_pose.position.x = 0.5052;
    cover_plate_pose.position.y = -0.08395;
    cover_plate_pose.position.z = 0.005;
    cover_plate_pose.orientation.x = 0.0;
    cover_plate_pose.orientation.y = 0.0;
    cover_plate_pose.orientation.z = 0.0;
    cover_plate_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose spindle_2_pose;
    spindle_2_pose.position.x = 0.3285;
    spindle_2_pose.position.y = -0.08823;
    spindle_2_pose.position.z = 0.005;
    spindle_2_pose.orientation.x = 0.0;
    spindle_2_pose.orientation.y = 0.0;
    spindle_2_pose.orientation.z = 0.0;
    spindle_2_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose idler_gear_pose;
    idler_gear_pose.position.x = 0.3357;
    idler_gear_pose.position.y = 0.0295;
    idler_gear_pose.position.z = 0.005;
    idler_gear_pose.orientation.x = 0.0;
    idler_gear_pose.orientation.y = 0.0;
    idler_gear_pose.orientation.z = 0.0;
    idler_gear_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose pinion_gear_pose;
    pinion_gear_pose.position.x = 0.4944;
    pinion_gear_pose.position.y = 0.06812;
    pinion_gear_pose.position.z = 0.007;
    pinion_gear_pose.orientation.x = 0.0;
    pinion_gear_pose.orientation.y = 0.0;
    pinion_gear_pose.orientation.z = 0.0;
    pinion_gear_pose.orientation.w = 1.0;

    // Add collision objects to the scene
    addCollisionObjectsToScene(base_plate, "base_plate", header_frame_id, base_plate_pose, collision_objects, object_colors, dark_gray);
    addCollisionObjectsToScene(cover_plate, "cover_plate", header_frame_id, cover_plate_pose, collision_objects, object_colors, dark_gray);
    addCollisionObjectsToScene(spindle_2, "spindle_2", header_frame_id, spindle_2_pose, collision_objects, object_colors, dark_gray);
    addCollisionObjectsToScene(idler_gear, "idler_gear", header_frame_id, idler_gear_pose, collision_objects, object_colors, gray);
    addCollisionObjectsToScene(pinion_gear, "pinion_gear", header_frame_id, pinion_gear_pose, collision_objects, object_colors, gray, scale_factor, scale_factor, scale_factor);

    // Add the collision objects and their colors to the planning scene separately
    psi.applyCollisionObjects(collision_objects, object_colors);

    rclcpp::sleep_for(std::chrono::seconds(2));
    signal(SIGINT, exit_sig_handler);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
