from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joint info node
    joints_info_node = Node(
        name='joints_info_node',
        package='xarm_custom_nodes',
        executable='joints_info_node',
    )
    
    # TCP pose publisher node
    tcp_pose_publisher_node = Node(
        name='tcp_pose_publisher_node',
        package='xarm_custom_nodes',
        executable='tcp_pose_publisher_node',
    )


    return LaunchDescription([
        joints_info_node,
        tcp_pose_publisher_node
    ])