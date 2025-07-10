import subprocess
import shlex

def kill_processes_by_name(process_name):
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

def main():
    # List of process names to kill
    process_names = [
        "ros2 run ros_tcp_endpoint default_server_endpoint",
        "ros2 run xarm_custom_nodes mode_switcher_node",
        "ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py",
        "ros2 launch xarm_moveit_config xarm7_moveit_gazebo.launch.py",
        "ros2 launch xarm_custom_nodes custom_nodes.launch.py",
        "ros2 run moveit_nodes_pkg unity_subscriber_cpp_node",
        "ros2 run moveit_nodes_pkg xarm7_mover_node",
        "ros2 run moveit_nodes_pkg update_planning_scene_node",
        "ros2 run moveit_nodes_pkg xarm_gripper_node",
        "ros2 launch moveit_nodes_pkg start_container.launch.py",
        "ros2 launch moveit_nodes_pkg experiment002.launch.py",
        "ros2 launch xarm_api xarm7_driver.launch.py",
    ]

    # Kill processes for each name in the list
    for process_name in process_names:
        kill_processes_by_name(process_name)

if __name__ == "__main__":
    main()