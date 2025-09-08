#! /usr/bin/env python3

#----------------------------------------------------------------#
# Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
# Software License Agreement (BSD License)
#
# Author: Newton Kariuki <newtonkaris45@gmail.com>
#----------------------------------------------------------------#

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16  # Import the service message
from xarm_msgs.msg import RobotMode
from std_msgs.msg import UInt8
import subprocess
import shlex
import psutil

workspace_folder = "/home/newtonjeri/dev_ws"
robot_ip = "172.16.40.20"

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')

        self.process_exist: bool = False
        self.state_machine_state = 0  # Track state machine state
        self.state_names = {
            0: "IDLE", 1: "MOVING", 2: "PICKING", 3: "PLACING", 
            4: "FINAL", 5: "ERROR"
        }

        # Create subscribers 
        self.mode_subscriber = self.create_subscription(RobotMode, "/robot_mode", self.mode_callback, 10)
        self.state_machine_subscriber = self.create_subscription(UInt8, "/xarm7_state_machine_state", self.state_machine_callback, 10)
        
        # Initialize process variables 
        self.moveit_process = "ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py"
        # self.moveit_process = "ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py"
        self.driver_process = "ros2 launch xarm_api xarm7_driver.launch.py"
        self.current_mode = "MODE-MOVEIT"
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Mode Switcher node started" % (
                              self.current_mode, self.state_names.get(self.state_machine_state, "UNKNOWN")))
        self.switch_processes(self.current_mode)

    def mode_callback(self, msg):
        requested_mode = None
        if msg.data == 0:
            requested_mode = "MODE-MOVEIT"
        elif msg.data == 2:
            requested_mode = "MODE-MANUAL"
        else:
            self.get_logger().error("MODE: %s -- STATE-MACHINE: %s -- Invalid mode: %s" % (
                                  self.current_mode, 
                                  self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                  str(msg.data)))
            return

        # Check if mode switch is safe based on current state
        if self.is_safe_to_switch_mode(requested_mode):
            self.current_mode = requested_mode
            # Switch processes based on the mode
            self.switch_processes(self.current_mode)
        else:
            self.get_logger().warn("MODE: %s -- STATE-MACHINE: %s -- Mode switch to %s denied - unsafe state" % (
                                 self.current_mode,
                                 self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                 requested_mode))

    def is_safe_to_switch_mode(self, requested_mode):
        """
        Check if it's safe to switch modes based on current state machine state
        """
        current_state_name = self.state_names.get(self.state_machine_state, "UNKNOWN")
        
        # Always allow switching to MANUAL mode from ERROR state (for recovery)
        if self.state_machine_state == 5 and requested_mode == "MODE-MANUAL":  # ERROR state
            return True
            
        # Don't allow switching during critical operations
        if self.state_machine_state in [2, 3]:  # PICKING or PLACING states
            self.get_logger().warn("MODE: %s -- STATE-MACHINE: %s -- Cannot switch mode during %s operation" % (
                                 self.current_mode,
                                 current_state_name,
                                 current_state_name))
            return False
            
        # Safe states for mode switching: IDLE, MOVING, FINAL
        if self.state_machine_state in [0, 1, 4]:  # IDLE, MOVING, FINAL
            return True
            
        # Default to safe
        return True

    def state_machine_callback(self, msg):
        """Callback to track state machine state for synchronization"""
        self.state_machine_state = msg.data
        self.get_logger().debug("MODE: %s -- STATE-MACHINE: %s -- State machine state updated" % (
                              self.current_mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN")))

    def switch_processes(self, mode):
        # Kill the current process based on the mode
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Switching to mode: %s" % (
                              mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN"),
                              mode))
                              
        if mode == "MODE-MANUAL":
            # Graceful transition: wait if robot is moving
            if self.state_machine_state == 1:  # MOVING state
                self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Waiting for movement to complete..." % (
                                     mode, 
                                     self.state_names.get(self.state_machine_state, "UNKNOWN")))
                # Could implement a retry mechanism here
                
            self.start_driver_process()
            if self.process_exist == False:
                self.run_in_current_terminal("ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById '{id: 8, data: 1}'")
                for i in range(2):
                    self.run_in_current_terminal("ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 '{data: 2}'")
                    self.run_in_current_terminal("ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 '{data: 0}'")
                
                self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Mode changed to: %s" % (
                                     mode, 
                                     self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                     mode))
            else:
                self.get_logger().warn("MODE: %s -- STATE-MACHINE: %s -- A similar process is already running" % (
                                     mode, 
                                     self.state_names.get(self.state_machine_state, "UNKNOWN")))
                
        elif mode == "MODE-MOVEIT":
            self.kill_processes_by_name(self.driver_process)
            self.kill_processes_by_name(self.moveit_process)
            self.start_moveit_process()
            if self.process_exist == False:
                self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Mode changed to: %s" % (
                                     mode, 
                                     self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                     mode))
            else:
                self.get_logger().warn("MODE: %s -- STATE-MACHINE: %s -- A similar process is already running" % (
                                     mode, 
                                     self.state_names.get(self.state_machine_state, "UNKNOWN")))
        else:
            self.get_logger().error("MODE: %s -- STATE-MACHINE: %s -- Invalid mode: %s" % (
                                  mode, 
                                  self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                  mode))

    def start_moveit_process(self):
        # Start the moveit_process
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Starting moveit_process..." % (
                              self.current_mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN")))
        self.moveit_process_full = f" cd {workspace_folder}; source install/setup.bash; ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py add_gripper:=true robot_ip:={robot_ip}"
        # self.moveit_process_full = f" cd {workspace_folder}; source install/setup.bash; ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py add_gripper:=true"
    
        self.run_in_new_tab(self.moveit_process_full)
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Running moveit_process..." % (
                              self.current_mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN")))
    
    def start_driver_process(self):
        # Start the driver_process
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Starting driver_process..." % (
                              self.current_mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN")))
        self.driver_process_full = f" cd {workspace_folder}; source install/setup.bash; ros2 launch xarm_api xarm7_driver.launch.py report_type:=normal robot_ip:={robot_ip}"
        self.run_in_new_tab(self.driver_process_full)
        self.get_logger().info("MODE: %s -- STATE-MACHINE: %s -- Running driver_process..." % (
                              self.current_mode, 
                              self.state_names.get(self.state_machine_state, "UNKNOWN")))
        
    
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
        # Example: "ros2 launch xarm_api xarm7_driver.launch.py report_type:=normal robot_ip:={robot_ip}"
        ros2_launch_command = None
        for part in shlex.split(command):
            if part == "ros2":
                ros2_launch_command = command[command.index("ros2"):]  # Extract from "ros2" to the end
                break

        if not ros2_launch_command:
            return False  # No "ros2 launch" command found
        
        self.get_logger().debug("MODE: %s -- STATE-MACHINE: %s -- ros2 launch command: %s" % (
                               self.current_mode, 
                               self.state_names.get(self.state_machine_state, "UNKNOWN"),
                               ros2_launch_command))

        # Extract the package name and launch file name from the "ros2 launch" command
        command_parts = shlex.split(ros2_launch_command)
        if len(command_parts) >= 4 and command_parts[1] == "launch":
            target_package = command_parts[2]  # Extract the package name (e.g., "xarm_api")
            target_launch_file = command_parts[3]  # Extract the launch file name (e.g., "xarm7_driver.launch.py")
            self.get_logger().debug("MODE: %s -- STATE-MACHINE: %s -- target_package: %s" % (
                                   self.current_mode, 
                                   self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                   target_package))
            self.get_logger().debug("MODE: %s -- STATE-MACHINE: %s -- target_launch_file: %s" % (
                                   self.current_mode, 
                                   self.state_names.get(self.state_machine_state, "UNKNOWN"),
                                   target_launch_file))
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