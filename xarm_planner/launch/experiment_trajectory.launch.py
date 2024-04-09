#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dof = LaunchConfiguration('dof')
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    xarm_planner_node_test = Node(
        name='experiment_trajectory_node',
        package='xarm_planner',
        executable='experiment_trajectory_node',
        output='screen',
        parameters=[
            {
                'robot_type': robot_type,
                'dof': dof
            },
        ],
    )
    return LaunchDescription([
        xarm_planner_node_test
    ])