import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# moveit
from moveit_msgs.action import MoveGroup
 
from moveit_msgs.msg import RobotState

# Custom interfaces
from xarm_msgs.srv import MoverService

import numpy as np
import transformations

# xarm7 joints
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]



"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles, robot_arm_rotation): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_reference_frame('base_link')  # Assuming 'base_link' is the robot's base frame

    # Rotate the destination pose by 180 degrees
    # rotated_destination_pose = rotate_pose_180_deg(destination_pose)
    rotated_destination_pose = rotate_pose(destination_pose, robot_arm_rotation.x, robot_arm_rotation.y, robot_arm_rotation.z)
    # move_group.set_pose_target(rotated_destination_pose)
    move_group.set_pose_target(rotated_destination_pose)
    # move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)



def rotate_pose(pose, roll, pitch, yaw):
    # Convert these angle to radians
    roll = np.deg2rad([roll])
    pitch = np.deg2rad([pitch])
    yaw = np.deg2rad([yaw])
    # Create a rotation matrix from roll, pitch, and yaw angles
    rotation_matrix = transformations.euler_matrix(roll, pitch, yaw)

    # Extract the current pose orientation as a quaternion
    current_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Create the current pose transformation matrix
    current_pose_matrix = transformations.quaternion_matrix(current_orientation)
    current_pose_matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

    # Apply the rotation to the current pose
    new_pose_matrix = transformations.concatenate_matrices(rotation_matrix, current_pose_matrix)

    # Extract the new position and orientation
    new_position = new_pose_matrix[:3, 3]
    new_orientation = transformations.quaternion_from_matrix(new_pose_matrix)

    # Create a new Pose object for the transformed pose
    new_pose = Pose()
    new_pose.position.x = new_position[0]
    new_pose.position.y = new_position[1]
    new_pose.position.z = new_position[2]
    new_pose.orientation.x = new_orientation[0]
    new_pose.orientation.y = new_orientation[1]
    new_pose.orientation.z = new_orientation[2]
    new_pose.orientation.w = new_orientation[3]

    return new_pose

# def rotate_pose_180_deg(pose):
#     # Create a 180 degree rotation matrix around the Z axis
#     rotation_matrix = transformations.rotation_matrix(math.pi, (0, 0, 1))

#     # Extract the current pose orientation as a quaternion
#     current_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

#     # Create the current pose transformation matrix
#     current_pose_matrix = transformations.quaternion_matrix(current_orientation)
#     current_pose_matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

#     # Apply the 180 degree rotation to the current pose
#     new_pose_matrix = transformations.concatenate_matrices(rotation_matrix, current_pose_matrix)

#     # Extract the new position and orientation
#     new_position = new_pose_matrix[:3, 3]
#     new_orientation = transformations.quaternion_from_matrix(new_pose_matrix)

#     # Create a new Pose object for the transformed pose
#     new_pose = Pose()
#     new_pose.position.x = new_position[0]
#     new_pose.position.y = new_position[1]
#     new_pose.position.z = new_position[2]
#     new_pose.orientation.x = new_orientation[0]
#     new_pose.orientation.y = new_orientation[1]
#     new_pose.orientation.z = new_orientation[2]
#     new_pose.orientation.w = new_orientation[3]

#     return new_pose

"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    response = MoverService.Response()

    group_name = "arm"
    move_group = moveit
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration, req.robotArmRotation)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles, req.robotArmRotation)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles, req.robotArmRotation)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles, req.robotArmRotation)

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response


def main(args=None):
    rclpy.init(args=args)

    mover_node = rclpy.Node("mover_node")

    rclpy.spin(mover_node)
    mover_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
