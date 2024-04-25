# HOW TO RUN THE ROBOT


## Overview

This file contains the procedure for running the real xarm7 robot arm


## Procedure to run the project
1. Open terminator and paste the below commands to navigate to the root folder of the workspace and run the ros2 tcp endpoint to connect to unity
```bash
# Navigate to the root of the workspace 
cd /home/robotics/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the ros2 to unity connection node (TCP endpoint)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.1.119
```

2. Open another terminal window and paste the following commands to launch the moveit config folder for the real arm
```bash
# Navigate to the root of the workspace 
cd /home/robotics/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the moveit configuration package for the real arm with the gripper attached
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py add_gripper:=true robot_ip:=192.168.1.225
```

3. Open another terminal and paste the below commands to run the nodes responsible for publishing data to unity and writing the data to files
```bash
# Navigate to the root of the workspace 
cd /home/robotics/xarm7_ws;

# Source the workspace
source install/setup.bash;

# Run the custome nodes that publishes data to unity
ros2 launch xarm_custome_nodes custom_nodes.launch.py
```
Note that after finishing the motion (described below), this process should be killed to prevent writing of unnecessary data in the csv files

## Moving the robot using moveit rviz GUI
To move the robot one has to select a start state and a goal state, for this experiment the defaul start state (<current>)
is used while for the goal state the "target_pose" is selected, then the plan and execute button is pressed to move the robot.
Once the robot reaches the goal state the goal state is changed to "home" then the paln and execute button is pressed to move the robot to the new goal pose.