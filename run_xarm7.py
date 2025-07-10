import subprocess
import re
import socket


workspace_folder = "/home/shared_folder/dev_ws"


def get_ip_address_simple():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Error getting IP address: {e}")
        return None


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
    ros_ip = str(get_ip_address_simple())  # Default ROS_IP

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
            f"cd {workspace_folder}; source install/setup.bash; "
            f"ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:={ros_ip}"
        )

    # Add the appropriate launch command based on the user's choice
    if choice == "real":
        commands.append(
            f"cd {workspace_folder}; source install/setup.bash; "
            "ros2 run xarm_custom_nodes mode_switcher_node"
        )
    else:
        commands.append(
            f"cd {workspace_folder}; source /usr/share/gazebo/setup.bash; source install/setup.bash; "
            "ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py add_gripper:=true"
        )

    # Common commands for both simulation and real robot
    commands.extend([
        f"cd {workspace_folder}; source install/setup.bash; ros2 launch xarm_custom_nodes custom_nodes.launch.py",
        f"cd {workspace_folder}; source install/setup.bash; ros2 run moveit_nodes_pkg xarm7_mover_node",
        f"cd {workspace_folder}; source install/setup.bash; ros2 run moveit_nodes_pkg xarm_gripper_node",
        f"cd {workspace_folder}; source install/setup.bash; ros2 run moveit_nodes_pkg unity_subscriber_cpp_node",
        # f"cd {workspace_folder}; source install/setup.bash; ros2 run moveit_nodes_pkg update_planning_scene_node",
        f"cd {workspace_folder}; source install/setup.bash; ros2 launch moveit_nodes_pkg start_container.launch.py",
        f"cd {workspace_folder}; source install/setup.bash; ros2 launch moveit_nodes_pkg experiment002.launch.py",
    ])

    # Run each command in a new tab
    for cmd in commands:
        run_in_new_tab(cmd)

if __name__ == "__main__":
    main()