from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Voice command client node
    voice_command_client_node = Node(
        name='voice_command_client_node',
        package='xarm7_voice_command_api',
        executable='voice_command_client_node',
    )

    # Voice command server node
    voice_command_server_node = Node(
        name='voice_command_server_node',
        package='moveit_nodes_pkg',
        executable='voice_command_server_node',
    )

    return LaunchDescription([
        voice_command_client_node,
        voice_command_server_node,
    ])