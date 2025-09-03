from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        LoadComposableNodes(
            target_container='exp2_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='moveit_nodes_pkg',
                    plugin='Experiment002Node',
                    name='experiment_002_node'
                )
            ],
        ),
    ])
