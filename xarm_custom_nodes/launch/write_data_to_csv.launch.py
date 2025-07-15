from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joints info saver node
    joints_info_saver_node = Node(
        name='joints_info_saver_node',
        package='xarm_custom_nodes',
        executable='joints_info_saver_node'
    )

    # Data from unity node 
    from_unity_node = Node(
        name = "time_stamp_subscriber_node",
        package = "xarm_custom_nodes",
        executable = "time_stamp_subscriber_node"
    )


    return LaunchDescription([
        joints_info_saver_node,
        from_unity_node
    ])