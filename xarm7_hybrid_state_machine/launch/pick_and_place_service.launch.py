#!/usr/bin/env python3
# Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: Newton Kariuki <newtonkaris45@gmail.com>

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Pick and Place Service Node
    pick_and_place_service_node = Node(
        package='xarm7_hybrid_state_machine',
        executable='pick_and_place_service_node',
        name='pick_and_place_service_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/xarm7_state_topic', '/xarm7_state_topic'),
            ('/xarm7_state_machine_state', '/xarm7_state_machine_state'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        pick_and_place_service_node,
    ])
