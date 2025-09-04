#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the path to the main moveit nodes launch file
    moveit_nodes_launch = PathJoinSubstitution([
        FindPackageShare('moveit_nodes_pkg'),
        'launch',
        '_moveit_nodes.launch.py'
    ])

    # Include main launch file with xarm7_mover_node configuration
    xarm7_mover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_nodes_launch),
        launch_arguments={
            'node_executable': 'xarm7_mover_node',
            'node_name': 'xarm7_mover_node',
            'group_name': LaunchConfiguration('group_name', default='xarm7'),
            'dof': LaunchConfiguration('dof', default='7'),
            'robot_type': LaunchConfiguration('robot_type', default='xarm'),
        }.items()
    )

    return LaunchDescription([
        xarm7_mover_launch
    ])
