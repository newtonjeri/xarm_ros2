from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # Start a container that can hold composable nodes
    container = ComposableNodeContainer(
        name='exp2_container',  # This name should match `target_container`
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[],
        output='screen',
    )

    return LaunchDescription([container])
