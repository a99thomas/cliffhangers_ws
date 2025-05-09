#!/usr/bin/env python3
"""
Launch RViz visualization for the mycobot robot.

This launch file sets up the complete visualization environment for the mycobot robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: Addison Sears-Collins
:date: November 15, 2024
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare, FindPackage

import subprocess



def process_ros2_controllers_config(context):
    """Process the ROS 2 controller configuration yaml file before loading the URDF.

    This function reads a template configuration file, replaces placeholder values
    with actual configuration, and writes the processed file to both source and
    install directories.

    Args:
        context: Launch context containing configuration values

    Returns:
        list: Empty list as required by OpaqueFunction
    """

    # Get the configuration values
    prefix = LaunchConfiguration('prefix').perform(context)
    l3_yaw = LaunchConfiguration('l3_yaw').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    home = str(Path.home())

    # Define both source and install paths
    src_config_path = os.path.join(
        home,
        'workspaces/cliffhangers_ws/src/climbbot3_moveit/config',
        # robot_name
    )
    install_config_path = os.path.join(
        home,
        'workspaces/cliffhangers_ws/install/climbbot3_moveit/share/climbbot3_moveit/config',
        # robot_name
    )

    # Read from source template
    template_path = os.path.join(src_config_path, 'ros2_controllers.yaml')
    with open(template_path, 'r', encoding='utf-8') as file:
        template_content = file.read()

    # Create processed content (leaving template untouched)
    processed_content = template_content.replace('${prefix}', prefix)

    # Write processed content to both source and install directories
    for config_path in [src_config_path, install_config_path]:
        os.makedirs(config_path, exist_ok=True)
        output_path = os.path.join(config_path, 'ros2_controllers.yaml')
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(processed_content)

    return []


# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='',
                          description='Name of the robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('add_world', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to add world link'),
    DeclareLaunchArgument('base_link', default_value='assembly_7',
                          description='Name of the base link'),
    DeclareLaunchArgument('base_type', default_value='g_shape',
                          description='Type of the base'),
    DeclareLaunchArgument('l3_yaw', default_value='l3_yaw',
                          description='Name of the flange link'),
    DeclareLaunchArgument('gripper_type', default_value='adaptive_gripper',
                          description='Type of the gripper'),
    DeclareLaunchArgument('use_camera', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use the RGBD Gazebo plugin for point cloud'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('use_gripper', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to attach a gripper')
]


def generate_launch_description():
    """Generate the launch description for the mycobot robot visualization.

    This function sets up all necessary nodes and parameters for visualizing
    the mycobot robot in RViz, including:
    - Robot state publisher for broadcasting transforms
    - Joint state publisher for simulating joint movements
    - RViz for visualization

    Returns:
        LaunchDescription: Complete launch description for the visualization setup
    """
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('climbbot3_description'),
            'climbbot3',
            'assets',
            'intel_rgbd_cam_d435.urdf.xacro'
        ]), ' ',
        'use_camera:=true ', 
        'use_gazebo:=true '
    ])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])
    

    # # Publish the joint state values for the non-fixed joints in the URDF file.
    # start_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     condition=IfCondition(use_jsp))

    # # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # start_joint_state_publisher_gui_cmd = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     condition=IfCondition(jsp_gui))

    # Launch RViz
    # start_rviz_cmd = Node(
    #     condition=IfCondition(use_rviz),
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': use_sim_time}])


    
    
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(OpaqueFunction(function=print_urdf_path))
    # ld.add_action(OpaqueFunction(function=print_urdf_contents))

    # Process the controller configuration before starting nodes
    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))
    

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    # ld.add_action(start_joint_state_publisher_cmd)
    # ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
