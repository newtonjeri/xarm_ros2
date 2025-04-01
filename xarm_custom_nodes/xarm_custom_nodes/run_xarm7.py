import subprocess
import re

def run_in_new_tab(command):
    """
    Opens a new terminal tab and runs the specified command.
    """
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", f"{command}; exec bash"])

def validate_ip(ip):
    """
    Validates an IP address format.
    """
    return re.match(r"^([0-9]{1,3}\.){3}[0-9]{1,3}$", ip) is not None

def main():
    # Prompt the user to choose between simulation or real robot
    choice = input("Do you want to run the simulation (sim) or the real robot (real)? ").strip().lower()
    if choice not in ["sim", "real"]:
        print("Invalid choice. Please enter 'sim' or 'real'.")
        return

    # Prompt the user to connect to Unity
    ip_choice = input("Do you want to connect to Unity? (y/n) ").strip().lower()
    ros_ip = "172.16.40.49"  # Default ROS_IP

    if ip_choice == "y":
        custom_ip = input(f"Please enter the ROS_IP (press Enter to use default IP {ros_ip}): ").strip()
        if custom_ip and validate_ip(custom_ip):
            ros_ip = custom_ip
        else:
            print(f"Invalid IP address format. Using the default ROS_IP: {ros_ip}")

    # Initialize an empty list for commands
    commands = []

    # Add the command to run the ros_tcp_endpoint node if Unity is connected
    if ip_choice == "y":
        commands.append(
            f"cd /home/shared_folder/xarm7_ws; source install/setup.bash; "
            f"ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:={ros_ip}"
        )

    # Add the appropriate launch command based on the user's choice
    if choice == "real":
        commands.append(
            "cd /home/shared_folder/xarm7_ws; source install/setup.bash; "
            "ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py add_gripper:=true robot_ip:=172.16.40.20"
        )
    else:
        commands.append(
            "cd /home/shared_folder/xarm7_ws; source /usr/share/gazebo/setup.bash; source install/setup.bash; "
            "ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py add_gripper:=true"
        )

    # Common commands for both simulation and real robot
    commands.extend([
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 launch xarm_custom_nodes custom_nodes.launch.py",
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 run moveit_nodes_pkg xarm7_mover_node",
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 run moveit_nodes_pkg xarm_gripper_node",
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 run moveit_nodes_pkg unity_subscriber_cpp_node",
        # "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 run moveit_nodes_pkg update_planning_scene_node",
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 launch moveit_nodes_pkg start_container.launch.py",
        "cd /home/shared_folder/xarm7_ws; source install/setup.bash; ros2 launch moveit_nodes_pkg experiment002.launch.py",
    ])

    # Run each command in a new tab
    for cmd in commands:
        run_in_new_tab(cmd)

if __name__ == "__main__":
    main()