#!/usr/bin/env python3
"""
Launch MoveIt Task Constructor test node for the xarm7 robotic arm.

This script creates a simple ROS 2 launch file that starts the MTC test node.
It assumes that the main MoveIt system is already running (e.g., via xarm7_moveit_realmove.launch.py).

:author: Newton Kariuki
:date: September 19, 2025
"""

import subprocess
import tempfile
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a launch description for MoveIt Task Constructor test node.

    Returns:
        LaunchDescription: A launch description for the MTC test node only
    """
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    def setup_mtc_node(context):
        """Setup MTC node by copying parameters from move_group"""
        
        # Get parameters from move_group node
        robot_description = ""
        robot_description_semantic = ""
        
        # Dictionary to store all robot_description_kinematics parameters
        kinematics_params = {}
        
        try:
            # Get robot_description from move_group
            result = subprocess.run(['ros2', 'param', 'get', '/move_group', 'robot_description'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if len(lines) > 1 and lines[0].startswith('String value is:'):
                    # Remove the "String value is: " prefix and join the rest
                    first_line = lines[0].replace('String value is: ', '')
                    robot_description = first_line + '\n'.join(lines[1:]) if len(lines) > 1 else first_line
        except Exception as e:
            print(f"Warning: Could not get robot_description: {e}")
        
        try:
            # Get robot_description_semantic from move_group
            result = subprocess.run(['ros2', 'param', 'get', '/move_group', 'robot_description_semantic'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if len(lines) > 0 and lines[0].startswith('String value is:'):
                    # Remove the "String value is: " prefix and join the rest
                    first_line = lines[0].replace('String value is: ', '')
                    robot_description_semantic = first_line + '\n'.join(lines[1:]) if len(lines) > 1 else first_line
        except Exception as e:
            print(f"Warning: Could not get robot_description_semantic: {e}")

        try:
            # Get essential kinematics parameters for xarm7 group only
            essential_params = [
                'robot_description_kinematics.xarm7.kinematics_solver',
                'robot_description_kinematics.xarm7.kinematics_solver_search_resolution',
                'robot_description_kinematics.xarm7.kinematics_solver_timeout',
                'robot_description_kinematics.xarm7.kinematics_solver_attempts'
            ]
            
            for param_name in essential_params:
                try:
                    param_result = subprocess.run(['ros2', 'param', 'get', '/move_group', param_name], 
                                                capture_output=True, text=True, timeout=5)
                    if param_result.returncode == 0:
                        param_output = param_result.stdout.strip()
                        if param_output.startswith('String value is: '):
                            value = param_output.replace('String value is: ', '')
                            if value.strip():  # Only add non-empty values
                                kinematics_params[param_name] = value
                                print(f"Debug: Added parameter {param_name}: {value}")
                        elif param_output.startswith('Integer value is: '):
                            value = int(param_output.replace('Integer value is: ', ''))
                            kinematics_params[param_name] = value
                            print(f"Debug: Added parameter {param_name}: {value}")
                        elif param_output.startswith('Double value is: '):
                            value = float(param_output.replace('Double value is: ', ''))
                            kinematics_params[param_name] = value
                            print(f"Debug: Added parameter {param_name}: {value}")
                except Exception as e:
                    print(f"Warning: Could not get parameter {param_name}: {e}")
        except Exception as e:
            print(f"Warning: Could not get kinematics parameters: {e}")

        print(f"Debug: robot_description length: {len(robot_description)}")
        print(f"Debug: robot_description_semantic length: {len(robot_description_semantic)}")
        print(f"Debug: Found {len(kinematics_params)} kinematics parameters")

        # Set up parameters for the node
        parameters = {
            'use_sim_time': use_sim_time.perform(context) == 'true',
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
        }
        
        # Add all kinematics parameters
        parameters.update(kinematics_params)
        
        print(f"Debug: Passing {len(parameters)} parameters to MTC node")

        # Create MTC test node with the copied parameters
        mtc_test_node = Node(
            package="xarm7_mtc_hybrid_state_machine",
            executable="mtc_test",
            output="screen",
            parameters=[parameters],
        )

        return [mtc_test_node]

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the MTC test node setup
    ld.add_action(OpaqueFunction(function=setup_mtc_node))

    return ld