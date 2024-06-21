# HOW TO RUN THE ROBOT


## Overview

This file contains the procedure for running the real xarm7 robot arm


## Procedure to run the project
1. Open terminal and paste the below commands to navigate to the root folder of the workspace and run the ros2 tcp endpoint to connect to unity
```bash
# Navigate to the root of the workspace 
cd /home/xarm7_dev/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the ros2 to unity connection node (TCP endpoint)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.1.123
```

The IP address of the computer running ros2 was 192.168.1.123. Use this command to check whether it has changed
```bash 
hostname -I
```
Use the IP address output in the terminal after running this command

2. Open another terminal window and paste the following commands to launch the moveit config folder for the real arm
```bash
# Navigate to the root of the workspace 
cd /home/xarm7_dev/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the moveit configuration package for the real arm with the gripper attached
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py add_gripper:=true robot_ip:=192.168.1.225
```

3. Open another terminal and paste the below commands to run the nodes responsible for publishing data to unity and writing the data to files
```bash
# Navigate to the root of the workspace 
cd /home/xarm7_dev/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the custome nodes that publishes data to unity
ros2 launch xarm_custome_nodes custom_nodes.launch.py
```
Note that after finishing the motion (described below), this process should be killed to prevent writing of unnecessary data in the csv files

4. Open another terminal and paste the following commands to run the node that gets the joint angles from unity
```bash
# Navigate to the root of the workspace 
cd /home/xarm7_dev/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the node to subcribe the values of the joint angles published by unity
ros2 run moveit_nodes_pkg unity_subscriber_cpp_node
```

## Moving the robot using moveit rviz GUI
To move the robot one has to select a start state and a goal state, for this experiment the default start state (current)
is used while for the goal state the "target_pose" is selected, then the plan and execute button is pressed to move the robot.
Once the robot reaches the goal state the goal state is changed to "home" then the paln and execute button is pressed to move the robot to the new goal pose.


## Moving the robot using Ufactory Studio
- Open the terminal and navigate to the folder where the appimage of Ufactory Studio is
```bash
cd /home/xarm7_dev/Ufactory/
```
- Run the application using this command
```bash
./UfactoryStudio-client-linux-1.0.1.AppImage
```
- The GUI of the app will open and you can be able to move the robot using a recorded motion, changing joint values and also selecting different modes 