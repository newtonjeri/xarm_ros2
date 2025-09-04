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

    # Unity subscriber node
    unity_subscriber_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_nodes_launch),
        launch_arguments={
            'node_executable': 'unity_subscriber_cpp_node',
            'node_name': 'unitysubscribercppnode',
            'group_name': LaunchConfiguration('unity_group_name', default='xarm7'),
            'dof': LaunchConfiguration('dof', default='7'),
            'robot_type': LaunchConfiguration('robot_type', default='xarm'),
        }.items()
    )

    # xarm7_mover node  
    xarm7_mover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_nodes_launch),
        launch_arguments={
            'node_executable': 'xarm7_mover_node',
            'node_name': 'xarm7_mover_node',
            'group_name': LaunchConfiguration('mover_group_name', default='xarm7'),
            'dof': LaunchConfiguration('dof', default='7'),
            'robot_type': LaunchConfiguration('robot_type', default='xarm'),
        }.items()
    )

    # Gripper node
    gripper_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_nodes_launch),
        launch_arguments={
            'node_executable': 'xarm_gripper_node',
            'node_name': 'xarm_gripper_node',
            'group_name': LaunchConfiguration('gripper_group_name', default='xarm_gripper'),
            'dof': LaunchConfiguration('dof', default='7'),
            'robot_type': LaunchConfiguration('robot_type', default='xarm'),
            'add_gripper': LaunchConfiguration('add_gripper', default='true'),
        }.items()
    )

    return LaunchDescription([
        unity_subscriber_launch,
        xarm7_mover_launch,
        gripper_node_launch
    ])
