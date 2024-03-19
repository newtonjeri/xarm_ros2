""""
    A custom launch file to launch the controllers 
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joint_state_broadcaster = Node(
        package = 'controller_manager',
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster"
        ]
    )

    xarm7_traj_controller = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "xarm7_traj_controller",
            "--controller-manager",
            "controller_manager"
        ]
    )

    xarm_gripper_traj_controller = Node(
        package= "controller_manager",
        executable = "spawner",
        arguments = [
            "xarm_gripper_traj_controller"
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster,
        xarm7_traj_controller,
        xarm_gripper_traj_controller
    ])