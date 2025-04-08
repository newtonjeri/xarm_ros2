from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes
)

def generate_launch_description():
    # Create a multi-threaded container
    container = ComposableNodeContainer(
        name='state_machine_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Note the '_mt' suffix for multi-threaded
        output='screen',
    )

    # Load your node into the container
    load_nodes = LoadComposableNodes(
        target_container='state_machine_container',
        composable_node_descriptions=[
            ComposableNode(
                package='xarm7_hybrid_state_machine',
                plugin='simple_state_machine::PickAndPlaceStateMachine',
                name='pick_and_place_sm_node',
            )
        ],
    )

    return LaunchDescription([
        container,
        load_nodes,
    ])