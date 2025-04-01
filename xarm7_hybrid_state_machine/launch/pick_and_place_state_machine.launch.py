import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        # Start a component container
        launch_ros.actions.Node(
            package="rclcpp_components",
            executable="component_container",
            name="component_container",
            output="screen"
        ),

        # Load the PickAndPlaceStateMachine component
        launch_ros.actions.Node(
            package="xarm7_hybrid_state_machine",
            executable="pick_and_place_sm_node",
            name="pick_and_place_sm",
            output="screen",
            parameters=[],
            emulate_tty=True
        )
    ])
