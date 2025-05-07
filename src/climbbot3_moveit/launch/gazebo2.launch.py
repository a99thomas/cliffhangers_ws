# File: core_system.launch.py

from pathlib import Path
import os

# Import necessary modules from the launch package
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def create_moveit_config():
    """Create and return the MoveIt configuration for the UR5e robot."""
    return (
        MoveItConfigsBuilder(robot_name="climbbot3", package_name="climbbot3_moveit")
        .robot_description(file_path=Path("config") / "climbbot3.urdf.xacro")
        # .robot_description_semantic(file_path=Path("config") / "ur5e_robotiq.srdf")
        .trajectory_execution(file_path=Path("config") / "moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .joint_limits(file_path=Path("config") / "joint_limits.yaml")
        .robot_description_kinematics(file_path=Path("config") / "kinematics.yaml")
        .moveit_cpp(file_path=Path("config") / "moveit_cpp.yaml")
        .to_moveit_configs()
    )

def create_rviz_node(launch_rviz, rviz_config_file, moveit_config, use_sim_time):
    """Create and return the RViz node for visualization."""
    return Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

def create_move_group_node(moveit_config, use_sim_time):
    """Create and return the Move Group node for controlling the robot's movements."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

def create_ros_gz_bridge_node(use_sim_time):
    """Create and return the ROS-GZ bridge node with the required topics."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/ur5e_arm_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory",
            "/robotiq_gripper_controller/command@std_msgs/msg/Float64@gz.msgs.Double",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

def create_robot_state_publisher_node(use_sim_time, moveit_config, sim_ignition):
    """Create and return the Robot State Publisher node to publish the robot's state to ROS topics."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"sim_ignition": sim_ignition},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        remappings=[
            ("robot_description", "/robot_description"),
            ("robot_description_semantic", "/robot_description_semantic"),
        ],
    )

def create_controller_manager_node(ros2_controllers_file, use_sim_time):
    """Create and return the Controller Manager node to manage robot controllers."""
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_file,
            {"use_sim_time": use_sim_time},
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
    )

def create_joint_state_broadcaster_spawner():
    """Create and return the Joint State Broadcaster spawner node."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

def create_controller_spawner(controller_name, use_sim_time):
    """Create and return a Controller spawner node for a specific controller."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration(controller_name)],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

def create_delay_rviz_event_handler(joint_state_broadcaster_spawner, rviz_node, move_group_node, launch_rviz):
    """Create and return an event handler to delay the launch of RViz until the joint state broadcaster is active."""
    return RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node, move_group_node],
        ),
        condition=IfCondition(launch_rviz),
    )

def create_ignition_gazebo_node(world_file):
    """Create and return the Ignition Gazebo node."""
    return ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen',
    )

def create_ignition_spawn_entity_node(robot_description_content):
    """Create and return the Ignition entity spawner node."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "climbbot3",  # Robot name
            "-allow_renaming", "true"
        ]
    )

def launch_setup(context, *args, **kwargs):
    """Setup and return the nodes to be launched based on the context and launch arguments."""
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ros2_controllers_file = LaunchConfiguration("ros2_controllers_file")
    sim_ignition = LaunchConfiguration("sim_ignition")
    world_file = LaunchConfiguration("world")

    moveit_config = create_moveit_config()

    # Generate robot description from XACRO
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("climbbot3_moveit"), "config", "climbbot3.urdf.xacro"]),
            " ",  # Pass any additional xacro arguments here
        ]
    )

    ros_gz_bridge_node = create_ros_gz_bridge_node(use_sim_time)
    rviz_node = create_rviz_node(launch_rviz, rviz_config_file, moveit_config, use_sim_time)
    move_group_node = create_move_group_node(moveit_config, use_sim_time)
    robot_state_publisher_node = create_robot_state_publisher_node(use_sim_time, moveit_config, sim_ignition)
    controller_manager_node = create_controller_manager_node(ros2_controllers_file, use_sim_time)
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    manipulator_controller_spawner = create_controller_spawner("ros2_controllers", use_sim_time)
    # gripper_controller_spawner = create_controller_spawner("robotiq_gripper_controller", use_sim_time)
    ignition_spawn_entity_node = create_ignition_spawn_entity_node(robot_description_content)
    delay_rviz_event_handler = create_delay_rviz_event_handler(
        joint_state_broadcaster_spawner, rviz_node, move_group_node, launch_rviz
    )
    ignition_gazebo_node = create_ignition_gazebo_node(world_file)

    nodes_to_start = [
        ignition_gazebo_node,
        TimerAction(
            period=10.0,
            actions=[
                ignition_spawn_entity_node,
                robot_state_publisher_node,
                controller_manager_node,
                joint_state_broadcaster_spawner,
                manipulator_controller_spawner,
                # gripper_controller_spawner,
                delay_rviz_event_handler,
                ros_gz_bridge_node,
            ]
        )
    ]

    return nodes_to_start

def generate_launch_description():
    """Generate and return the launch description, including declared arguments and setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            "manipulator_joint_controller",
            default_value="ur5e_arm_controller",
            description="Name of the robot manipulator controller to start.",
        ),
        DeclareLaunchArgument(
            "robotiq_gripper_controller",
            default_value="robotiq_gripper_controller",
            description="Name of the robot gripper controller to start.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("climbbot3_moveit"), "config", "moveit.rviz"]
            ),
            description="Absolute path to the RViz config file used when launching RViz.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use simulation time.",
        ),
        DeclareLaunchArgument(
            "ros2_controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("climbbot3_moveit"), "config", "ros2_controllers.yaml"]
            ),
            description="Path to the ROS 2 controllers configuration file.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(get_package_share_directory("climbbot3_moveit"), "worlds", "empty.sdf"),
            description="Path to the world file to load in Ignition Gazebo."
        ),
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Whether to use Ignition Gazebo simulation."
        ),
        DeclareLaunchArgument(
            "ros_gz_bridge_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("climbbot3_moveit"), "config", "ros_gz_bridge.yaml"]
            ),
            description="Path to the ROS-Ignition bridge configuration file.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
