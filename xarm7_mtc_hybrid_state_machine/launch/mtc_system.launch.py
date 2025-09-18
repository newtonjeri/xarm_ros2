# Copyright 2025 Virtual Mechatronics Labs DeKUT All Rights Reserved.
#
# Software License Agreement (BSD Li    # MTC Service Node
# author: Newton Kariuki <newtonkaris45@gmail.com>

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Robot configuration parameters with defaults
    dof = LaunchConfiguration('dof', default='7')
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    # Build MoveIt configuration

    # Build MoveIt configuration
    moveit_config = (MoveItConfigsBuilder(
        context=context,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    )
    .trajectory_execution(file_path='config/xarm7/controllers.yaml')
    .joint_limits(file_path='config/xarm7/joint_limits.yaml')
    .robot_description_kinematics(file_path='config/xarm7/kinematics.yaml')
    .planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
        default_planning_pipeline="ompl"
    )
    .planning_scene_monitor(
        publish_robot_description=False,
        publish_robot_description_semantic=True,
        publish_planning_scene=True,
    )
    .pilz_cartesian_limits(file_path='moveit_configs/pilz_cartesian_limits.yaml')
    .to_moveit_configs())


    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # MTC Service Node - Main interface for task requests
    mtc_service_node = Node(
        package='xarm7_mtc_hybrid_state_machine',
        executable='mtc_service_standalone',
        name='mtc_service_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),  # Use the full MoveIt config for proper execution
            {
                'use_sim_time': use_sim_time,
                'group_name': 'xarm7',
                'robot_type': robot_type,
                'dof': dof,
                'prefix': prefix
            }
        ],
        remappings=[
            ('/mtc_task_service', '/xarm/mtc_task_service'),
        ]
    )
    
    return [
        # Core MTC system nodes
        mtc_service_node,
        # No separate execution server needed - MTC handles execution directly
        # when properly configured with MoveIt trajectory execution
    ]


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='7',
        description='DOF of the robot'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='xarm',
        description='Type of robot'
    )
    
    add_gripper_arg = DeclareLaunchArgument(
        'add_gripper',
        default_value='true',
        description='Add gripper to robot'
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        dof_arg,
        robot_type_arg,
        add_gripper_arg,
        
        OpaqueFunction(function=launch_setup)
    ])