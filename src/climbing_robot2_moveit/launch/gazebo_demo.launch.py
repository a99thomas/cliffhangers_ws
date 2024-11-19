from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to your robot's URDF file
    urdf_path = os.path.join(
        get_package_share_directory('robot_1'),  # replace 'robot_1' with your actual package name
        'urdf',
        'robot.urdf'  # replace 'your_robot.urdf' with the actual URDF file name
    )

    return LaunchDescription([
        # Start gz_sim with an empty world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '-r', '4', 'empty.sdf'],
            output='screen'
        ),

        # Bridge the joint states topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_states_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointStategz.msgs.Model'],
            output='screen'
        ),

        # Bridge the robot description topic
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='robot_description_bridge',
        #     arguments=['/robot_description@std_msgs/msg/String@gz.msgs.StringMsg'],
        #     output='screen',
        #     parameters=[{'qos_overrides./robot_description.publisher.durability': 'volatile'}]
        # ),
        # Robot state publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': urdf_path}]
        # ),

        # Spawn the robot model in gz_sim
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'onshape',
                '-file', urdf_path,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5'
            ],
            output='screen',
        )


        # ExecuteProcess(
        #     cmd=['gz', 'sim', 'create', '-v', '4', '-m', 'onshape', '-f', urdf_path],
        #     output='screen'
        # )
    ])