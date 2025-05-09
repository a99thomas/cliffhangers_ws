#!/bin/bash
# Single script to launch the myCobot with Gazebo and ROS 2 Controllers

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
ros2 launch climbbot3_gz climbbot3.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_camera:=false \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.29 \
    y:=0.253 \
    z:=0.04 \
    roll:=3.14 \
    pitch:=-1.3\
    yaw:=1.57


# ros2 launch climbbot3_moveit move_group.launch.py
#ros2 param set /move_group use_sim_time True



    # x:=0.29 \
    # y:=-0.12 \
    # z:=0.03 \
    # roll:=3.14 \
    # pitch:=0\
    # yaw:=1.57
    # y:=0.255 \
    # z:=0.04 \
    # roll:=3.14 \
    # pitch:=-1.3\
    # yaw:=1.57
