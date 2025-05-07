#!/usr/bin/env python3
import math
import time
import sys
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State, GripperInterface
from threading import Thread, Event

from pymoveit2.robots import climbing_robot

def define_joint_goals():
    """
    Define and return joint goals for the UR5e robot.
    These joint goals represent an initial configuration for the robot.
    """
    joint_goal_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return joint_goal_1

def define_pose_goal():
    """
    Define and return the pose goal for the UR5e robot.
    The pose goal includes the position (in meters) and the orientation as a quaternion.
    """
    pose_goal_position = [0.7, 0.3, 0.1]
    pose_goal_orientation = [1.0, 0.0, 0.0, 0.0]
    return pose_goal_position, pose_goal_orientation

class ClimbingMotionNode(Node):
    def __init__(self):
        super().__init__("climbing_joint_and_pose_goal_repeater")
        
        # Initialize MoveIt2 for both left and right arms
        self.left_moveit2 = MoveIt2(
            node=self,
            joint_names=climbing_robot.left_arm_joint_names(),
            base_link_name="world",
            end_effector_name=climbing_robot.left_arm_tip_link_name(),
            group_name=climbing_robot.MOVE_GROUP_LEFT_ARM,
        )
        self.right_moveit2 = MoveIt2(
            node=self,
            joint_names=climbing_robot.right_arm_joint_names(),
            base_link_name="world",
            end_effector_name=climbing_robot.right_arm_tip_link_name(),
            group_name=climbing_robot.MOVE_GROUP_RIGHT_ARM,
        )

        # Set motion limits for smoother movement
        self.left_moveit2.max_velocity = 1.0  # Example value
        self.left_moveit2.max_acceleration = 1.0
        self.right_moveit2.max_velocity = 1.0

        # Define the goals
        self.joint_goal = define_joint_goals()
        self.pose_goal_position, self.pose_goal_orientation = define_pose_goal()
        
        # Create an event to control shutdown of the motion thread
        self.shutdown_event = Event()

        # Start the repeating motion in a background thread
        self.motion_thread = Thread(target=self.motion_loop, daemon=True)
        self.motion_thread.start()
        self.get_logger().info("climbing motion node started and motion thread launched.")

    def motion_loop(self):
        """
        This loop repeats the sequence:
        1. Move to the initial joint configuration.
        2. Log the current end-effector position (via forward kinematics).
        3. Move to the desired pose goal.
        4. Log the outcome.
        Then the loop repeats until the node is shut down.
        """
        while not self.shutdown_event.is_set():
            try:
                # Move to the initial joint configuration
                self.get_logger().info("Moving to joint goal...")
                self.left_moveit2.move_to_configuration(self.joint_goal)
                # wait for execution to complete
                if not self.left_moveit2.wait_until_executed():
                    self.get_logger().warn("Joint configuration execution did not complete successfully.")
                time.sleep(0.2)  # short pause

                # Compute and log the forward kinematics of the current joint configuration
                joints = self.left_moveit2.joint_state.position
                # Using only the first 5 joints (modify as needed)
                pose_stamped = self.left_moveit2.compute_fk(joints[0:5])
                position = pose_stamped.pose.position
                self.get_logger().info(
                    f"Post joint-goal FK: End Effector Position: x = {position.x}, y = {position.y}, z = {position.z}"
                )

                # Move to the pose goal
                self.get_logger().info(
                    f"Moving to pose goal at position {self.pose_goal_position} with orientation {self.pose_goal_orientation}."
                )
                self.left_moveit2.move_to_pose(
                    position=self.pose_goal_position,
                    quat_xyzw=self.pose_goal_orientation,
                    cartesian=False,
                )
                left_success = self.left_moveit2.wait_until_executed()
                self.get_logger().info(f"Pose goal trajectory success: {left_success}")
                time.sleep(0.25)

                # Log new end-effector position after pose move
                joints = self.left_moveit2.joint_state.position
                pose_stamped = self.left_moveit2.compute_fk(joints[0:5])
                position = pose_stamped.pose.position
                self.get_logger().info(
                    f"Post pose-goal FK: End Effector Position: x = {position.x}, y = {position.y}, z = {position.z}"
                )

                # Wait a bit before starting the next iteration
                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Error in motion loop: {e}")
                # Optionally, break out of loop or continue
                time.sleep(1.0)

    def destroy_node(self):
        # Signal the motion thread to stop
        self.shutdown_event.set()
        self.motion_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClimbingMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()
