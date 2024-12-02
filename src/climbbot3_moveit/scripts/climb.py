#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import climbing_robot
from visualization_msgs.msg import MarkerArray
import numpy as np
import time

class ClimbingRobotController(Node):
    def __init__(self):
        super().__init__("climbing_robot_marker_controller")
        
        # Parameters
        self.declare_parameter("cartesian", False)
        self.declare_parameter("synchronous", True)
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize marker subscriber
        self.marker_subscription = self.create_subscription(
            MarkerArray,
            'blue_object_centroids',
            self.marker_callback,
            10
        )
        
        # Initialize MoveIt2 interfaces for both arms
        self.left_moveit2 = MoveIt2(
            node=self,
            joint_names=climbing_robot.left_arm_joint_names(),
            base_link_name=climbing_robot.base_link_name(),
            end_effector_name=climbing_robot.left_arm_tip_link_name(),
            group_name=climbing_robot.MOVE_GROUP_LEFT_ARM,
            callback_group=self.callback_group,
        )
        
        self.right_moveit2 = MoveIt2(
            node=self,
            joint_names=climbing_robot.right_arm_joint_names(),
            base_link_name=climbing_robot.base_link_name(),
            end_effector_name=climbing_robot.right_arm_tip_link_name(),
            group_name=climbing_robot.MOVE_GROUP_RIGHT_ARM,
            callback_group=self.callback_group,
        )
        
        # Set velocity scaling
        self.left_moveit2.max_velocity = 0.5
        self.right_moveit2.max_velocity = 0.5
        
        # Initialize marker positions
        self.marker_positions = []
        self.is_executing = False

    def marker_callback(self, msg):
        if self.is_executing:
            return
            
        # Extract marker positions
        positions = []
        for marker in msg.markers:
            pos = marker.pose.position
            positions.append((pos.x, pos.y, pos.z))
            
        if not positions:
            self.get_logger().warn("No markers detected!")
            return
            
        # Find two nearest markers to robot base
        # Note: You might want to adjust this based on your robot's base position
        base_position = np.array([0.0, 0.0, 0.0])
        distances = [np.linalg.norm(np.array(pos) - base_position) for pos in positions]
        sorted_indices = np.argsort(distances)
        
        if len(positions) >= 2:
            self.move_arms_to_markers(
                positions[sorted_indices[0]],  # Nearest point for left arm
                positions[sorted_indices[1]]  # Second nearest point for right arm


            )

            self.move_arms_to_markers(
                positions[sorted_indices[2]],  # Nearest point for left arm
                positions[sorted_indices[3]] 

            )
        else:
            self.get_logger().warn("Need at least 2 markers for both arms!")

    def move_arms_to_markers(self, left_target, right_target):
        self.is_executing = True
        self.get_logger().info(f"Moving left arm to: {left_target}")
        
        
        # Clear existing constraints
        self.left_moveit2.clear_goal_constraints()
        self.right_moveit2.clear_goal_constraints()
        
        # Set position goals for left and right arms
        self.left_moveit2.set_position_goal(
            position=left_target,
            tolerance=0.01,
            weight=1.0
        )
        
        self.right_moveit2.set_position_goal(
            position=right_target,
            tolerance=0.01,
            weight=1.0
        )
        
        # Plan and execute movements for the left arm
        left_trajectory = self.left_moveit2.plan()
        if left_trajectory is not None:
            self.left_moveit2.execute(left_trajectory)
            self.left_moveit2.wait_until_executed()  # Wait until left arm completes movement
        else:
            self.get_logger().warn("Left arm planning failed!")

        self.move_arm_in_x_direction(self.left_moveit2, left_target, -0.2, "left")

        self.get_logger().info(f"Moving right arm to: {right_target}")
        
        # Plan and execute movements for the right arm
        right_trajectory = self.right_moveit2.plan()
        if right_trajectory is not None:
            self.right_moveit2.execute(right_trajectory)
            self.right_moveit2.wait_until_executed()  # Wait until right arm completes movement
        else:
            self.get_logger().warn("Right arm planning failed!")

        # Now move both arms by -0.3 in the x direction
        
        self.move_arm_in_x_direction(self.right_moveit2, right_target, -0.2, "right")
        
        self.is_executing = False

    def move_arm_in_x_direction(self, arm_moveit2, target_position, delta_x, arm_name):
        # Get current joint states
        current_state = arm_moveit2.joint_state
        if current_state is None:
            self.get_logger().warn("No joint state available!")
            return
            
        # Create list of all joint positions, setting only our target joint to 0.01
        target_joint_name = "l3_1" if arm_name == "left" else "r3_1"
        self.get_logger().info(f"Moving {arm_name} arm's {target_joint_name} joint to 0.01m")
        
        joint_positions = []
        joint_names = []
        
        # Go through current joint states and create our target state
        for i, name in enumerate(current_state.name):
            if name in arm_moveit2.joint_names:  # Only include joints for this arm
                joint_names.append(name)
                if name == target_joint_name:
                    joint_positions.append(0.01)  # Set our target joint to 0.01
                else:
                    joint_positions.append(current_state.position[i])  # Keep current position for other joints
        
        # Move to the target configuration
        arm_moveit2.move_to_configuration(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance=0.001,
            weight=1.0
        )
        
        # Wait until the motion is complete
        arm_moveit2.wait_until_executed()

def main():
    rclpy.init()
    
    # Create and spin the node
    node = ClimbingRobotController()
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    
    # Spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Wait for services to become available
    node.get_logger().info("Waiting for MoveIt 2 services...")
    time.sleep(5.0)
    
    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()
