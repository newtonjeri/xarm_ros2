from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joint states subscriber node
    joint_states_subscriber_node = Node(
        name='joint_states_subscriber_node',
        package='xarm_custome_nodes',
        executable='joint_states_subscriber_node',
    )

    # TCP pose publisher node
    tcp_pose_publisher_node = Node(
        name='tcp_pose_publisher_node',
        package='xarm_custome_nodes',
        executable='tcp_pose_publisher_node',
    )


    return LaunchDescription([
        joint_states_subscriber_node,
        tcp_pose_publisher_node,
    ])