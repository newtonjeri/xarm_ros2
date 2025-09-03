import subprocess
import re
import socket
import threading
import time
import sys
import select
import termios
import tty
import os
import signal

workspace_folder = "/home/newtonjeri/dev_ws"

class KeyboardController:
    def __init__(self):
        self.recording_process = None
        self.recording_active = False
        self.old_settings = None
        
    def get_char(self):
        """Get a single character from stdin without pressing Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def start_recording(self):
        """Start the data recording process"""
        if self.recording_process is None or self.recording_process.poll() is not None:
            print("\n Starting data recording...")
            command = f"cd {workspace_folder} && source install/setup.bash && ros2 launch xarm_custom_nodes write_data_to_csv.launch.py"
            run_in_new_tab(command)
            self.recording_active = True
            print(" Data recording started! Press 's' to stop.")
        else:
            print("\n  Recording is already active!")
    
    def stop_recording(self):
        """Stop the data recording process"""

        kill_processes_by_name("ros2 launch xarm_custom_nodes write_data_to_csv.launch.py")
        if self.recording_process and self.recording_process.poll() is None:
            print("\n Stopping data recording...")
            try:
                kill_processes_by_name("ros2 launch xarm_custom_nodes write_data_to_csv.launch.py")
            except subprocess.TimeoutExpired:
                print(" Process didn't terminate gracefully, forcing shutdown...")
                os.killpg(os.getpgid(self.recording_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass  # Process already terminated
            
            self.recording_active = False
            self.recording_process = None
            print(" Data recording stopped!")
        else:
            print("\n No active recording to stop!")
    
    def keyboard_listener(self):
        """Listen for keyboard input in a separate thread"""
        print("\n" + "="*50)
        print("   KEYBOARD CONTROLS:")
        print("   Press 'w' to START data recording")
        print("   Press 's' to STOP data recording")
        print("   Press 'q' to QUIT the program")
        print("="*50)
        
        while True:
            try:
                char = self.get_char().lower()
                
                if char == 'w':
                    self.start_recording()
                elif char == 's':
                    self.stop_recording()
                elif char == 'q':
                    print("\n Exiting program...")
                    self.stop_recording()  # Stop recording if active
                    kill_all()
                    break
                elif char == '\x03':  # Ctrl+C
                    break
                    
            except KeyboardInterrupt:
                print("\n Exiting program...")
                self.stop_recording()
                break
            except Exception as e:
                print(f"\nError in keyboard listener: {e}")
                break

def get_ip_address_simple():
    """
    Lists all available network interfaces and their IP addresses,
    then allows the user to select which one to use.
    """
    import netifaces
    
    try:
        print("\n" + "="*50)
        print("🌐 AVAILABLE NETWORK INTERFACES:")
        print("="*50)
        
        interfaces = netifaces.interfaces()
        valid_interfaces = []
        
        for i, interface in enumerate(interfaces):
            try:
                # Get IPv4 addresses for this interface
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    ipv4_info = addrs[netifaces.AF_INET][0]
                    ip_addr = ipv4_info['addr']
                    netmask = ipv4_info.get('netmask', 'N/A')
                    
                    # Skip loopback addresses
                    if not ip_addr.startswith('127.'):
                        valid_interfaces.append((interface, ip_addr, netmask))
                        print(f"{len(valid_interfaces)}. {interface:12} - {ip_addr:15} (Netmask: {netmask})")
                        
            except (KeyError, IndexError):
                # Skip interfaces without IPv4 addresses
                continue
        
        if not valid_interfaces:
            print("No valid network interfaces found!")
            # Fallback to original method
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        
        print("="*50)
        
        # Let user choose interface
        while True:
            try:
                choice = input(f"Select interface (1-{len(valid_interfaces)}) or press Enter for auto-detect: ").strip()
                
                if choice == "":
                    # Auto-detect using original method
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ip = s.getsockname()[0]
                    s.close()
                    print(f"Auto-detected IP: {ip}")
                    return ip
                
                choice_num = int(choice)
                if 1 <= choice_num <= len(valid_interfaces):
                    selected_interface, selected_ip, selected_netmask = valid_interfaces[choice_num - 1]
                    print(f"Selected: {selected_interface} - {selected_ip}")
                    return selected_ip
                else:
                    print(f"Invalid choice. Please enter a number between 1 and {len(valid_interfaces)}")
                    
            except ValueError:
                print("Invalid input. Please enter a number or press Enter for auto-detect.")
            except KeyboardInterrupt:
                print("\nOperation cancelled by user")
                return None
                
    except ImportError:
        print("netifaces module not found. Using fallback method...")
        print("Install with: pip install netifaces")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as e:
            print(f"Error getting IP address: {e}")
            return None
    except Exception as e:
        print(f"Error listing network interfaces: {e}")
        # Fallback to original method
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception as fallback_e:
            print(f"Error getting IP address: {fallback_e}")
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

def kill_all():
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

def main():
    # Prompt the user to choose between simulation or real robot
    choice = input("Do you want to run the simulation (sim) or the real robot (real)? ").strip().lower()
    if choice not in ["sim", "real"]:
        print("Invalid choice. Please enter 'sim' or 'real'.")
        return

    # Prompt the user to connect to Unity
    ip_choice = input("Do you want to connect to Unity? (y/n) ").strip().lower()

    if ip_choice == "y":
        ros_ip = str(get_ip_address_simple())
        # custom_ip = input(f"Please enter the ROS_IP (press Enter to use default IP {ros_ip}): ").strip()
        # if custom_ip and validate_ip(custom_ip):
        #     ros_ip = custom_ip
        # else:
        #     print(f"Invalid IP address format. Using the default ROS_IP: {ros_ip}")

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
        f"cd {workspace_folder}; source install/setup.bash; ros2 run moveit_nodes_pkg update_planning_scene_node",
        f"cd {workspace_folder}; source install/setup.bash; ros2 launch moveit_nodes_pkg start_container.launch.py",
        f"cd {workspace_folder}; source install/setup.bash; ros2 launch moveit_nodes_pkg experiment002.launch.py",
    ])

    # Run each command in a new tab
    print("Starting all ROS2 nodes...")
    for cmd in commands:
        run_in_new_tab(cmd)

    # Give some time for the nodes to start
    print("Waiting for nodes to initialize...")
    time.sleep(3)

    # Start the keyboard controller
    controller = KeyboardController()
    
    try:
        # Start keyboard listener in the main thread
        controller.keyboard_listener()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        controller.stop_recording()
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        controller.stop_recording()

if __name__ == "__main__":
    main()