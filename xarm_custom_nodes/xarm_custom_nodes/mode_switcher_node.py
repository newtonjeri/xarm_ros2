#! /usr/bin/env python3

#----------------------------------------------------------------#
# Copyright 2025 Virtual Reality Labs DKUT All Rights Reserved.
# Software License Agreement (BSD License)
#
# Author: Newton Kariuki <newtonkaris45@gmail.com>
#----------------------------------------------------------------#

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16  # Import the service message
from xarm_msgs.msg import RobotMode
import subprocess
import shlex
import psutil

import time


class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        self.process_exist: bool = False

        # Create a subscriber 
        self.mode_subscriber = self.create_subscription(RobotMode, "/robot_mode", self.mode_callback, 10)
        
        # Initialize process variables 
        self.moveit_process = "ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py"
        # self.moveit_process = "ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py"
        self.driver_process = "ros2 launch xarm_api xarm7_driver.launch.py"
        self.current_mode = "MODE-MOVEIT"
        self.get_logger().info("Mode Switcher node started")
        self.switch_processes(self.current_mode)

    def mode_callback(self, msg):
        if msg.data == 0:
            self.current_mode = "MODE-MOVEIT"
        elif msg.data == 2:
            self.current_mode = "MODE-MANUAL"
        else:
            self.get_logger().error("Invalid mode: " + str(msg.data))
            return

        # Switch processes based on the mode
        self.switch_processes(self.current_mode)

    def switch_processes(self, mode):
        # Kill the current process based on the mode
        self.get_logger().info("Current mode: " + mode)
        if mode == "MODE-MANUAL":
            self.kill_processes_by_name(self.moveit_process)
            self.start_driver_process()
            if self.process_exist == False:
                self.run_in_current_terminal("ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById '{id: 8, data: 1}'")
                for i in range(2):
                    self.run_in_current_terminal("ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 '{data: 2}'")
                    self.run_in_current_terminal("ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 '{data: 0}'")
                
                self.get_logger().info("Mode changed to: " + (mode))
            else:
                self.get_logger().warn("A similar process is already running")
                
        elif mode == "MODE-MOVEIT":
            self.kill_processes_by_name(self.driver_process)
            self.start_moveit_process()
            if self.process_exist == False:
                self.get_logger().info("Mode changed to: " + (mode))
            else:
                self.get_logger().warn("A similar process is already running")
        else:
            self.get_logger().error("Invalid mode: " + (mode))

    def start_moveit_process(self):
        # Start the moveit_process
        self.get_logger().info("Starting moveit_process...")
        self.moveit_process_full = "cd /home/shared_folder/dev_ws; source install/setup.bash; ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py add_gripper:=true robot_ip:=172.16.40.20"
        # self.moveit_process_full = "cd /home/shared_folder/dev_ws; source install/setup.bash; ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py add_gripper:=true"
    
        self.run_in_new_tab(self.moveit_process_full)
        self.get_logger().info("Running moveit_process...")
    
    def start_driver_process(self):
        # Start the driver_process
        self.get_logger().info("Starting driver_process...")
        self.driver_process_full = "cd /home/shared_folder/dev_ws; source install/setup.bash; ros2 launch xarm_api xarm7_driver.launch.py report_type:=normal robot_ip:=172.16.40.20"
        self.run_in_new_tab(self.driver_process_full)
        self.get_logger().info("Running driver_process...")
        
    
    def run_in_new_tab(self, command):
        """
        Opens a new terminal tab and runs the specified command only if no similar process is running.
        """
        self.process_exist = self.is_process_running(command)
        # Check if a similar process is already running
        if not self.process_exist:
            # If no similar process is running, run the command in a new terminal tab
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", f"{command}; exec bash"])
        else:
            print(f"A similar process is already running for command: {command}")
            
    def is_process_running(self, command):
        """
        Checks if a process matching the specific package and launch file is already running.
        """
        # Extract the "ros2 launch" part of the command
        # Example: "ros2 launch xarm_api xarm7_driver.launch.py report_type:=normal robot_ip:=172.16.40.20"
        ros2_launch_command = None
        for part in shlex.split(command):
            if part == "ros2":
                ros2_launch_command = command[command.index("ros2"):]  # Extract from "ros2" to the end
                break

        if not ros2_launch_command:
            return False  # No "ros2 launch" command found
        
        self.get_logger().info("ros2 launch command: " + ros2_launch_command)

        # Extract the package name and launch file name from the "ros2 launch" command
        command_parts = shlex.split(ros2_launch_command)
        if len(command_parts) >= 4 and command_parts[1] == "launch":
            target_package = command_parts[2]  # Extract the package name (e.g., "xarm_api")
            target_launch_file = command_parts[3]  # Extract the launch file name (e.g., "xarm7_driver.launch.py")
            self.get_logger().info("target_package: " + target_package)
            self.get_logger().info("target_launch_file: " + target_launch_file)
        else:
            target_package = None
            target_launch_file = None

        # Iterate through all running processes
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['cmdline']:
                    # Join the command line arguments into a single string
                    cmdline_str = ' '.join(proc.info['cmdline'])
                    
                    # Check if both the package name and launch file name are in the command line
                    if (target_package and target_package in cmdline_str and
                        target_launch_file and target_launch_file in cmdline_str):
                        return True  # A matching process is running
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        return False  # No matching process found

    def run_in_current_terminal(self, command):
        """
        Runs the specified command in the current terminal.
        """
        try:
            # Run the command in the current terminal
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Command failed with error: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def kill_processes_by_name(self, process_name):
        """
        Kills processes by their full command name.
        """
        # Use 'ps aux' to find processes matching the full command name
        ps_command = f"ps aux | grep '{process_name}' | grep -v grep"
        try:
            # Run the command and capture the output
            output = subprocess.check_output(ps_command, shell=True, text=True)
            
            # Extract PIDs from the output
            pids = [line.split()[1] for line in output.splitlines()]
            
            if not pids:
                print(f"No processes found with the name: {process_name}")
            else:
                print(f"Killing processes with the name: {process_name}")
                for pid in pids:
                    print(f"Killing process ID {pid}")
                    subprocess.run(["kill", pid])
        except subprocess.CalledProcessError:
            print(f"No processes found with the name: {process_name}")

def main(args=None):
    rclpy.init(args=args)
    mode_switcher = ModeSwitcher()
    
    rclpy.spin(mode_switcher)
    mode_switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()