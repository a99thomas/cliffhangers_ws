#!/usr/bin/env python3
from threading import Thread
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import climbing_robot
from visualization_msgs.msg import MarkerArray
import numpy as np
import time
from geometry_msgs.msg import Point, PoseStamped

class ClimbingRobotController(Node):
    def __init__(self):
        super().__init__("climbing_robot_marker_controller")
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Other initializations...

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
        self.left_moveit2.max_velocity = 1
        self.right_moveit2.max_velocity = 1
        
        # Initialize marker positions
        self.marker_positions = []
        self.is_executing = False
            
    def marker_callback(self, msg):
        self.latest_marker_msg = msg

    def climb(self):

        

        r_offset1 = [float(0.0), float(-0.0), float(-0.0)]
        r_offset2 = [float(0.0), float(-0.0), float(0.0)]
        l_offset1 = [float(0.0), float(0.0), float(-0.0)]
        l_offset2 = [float(0.0), float(0.0), float(0.0)]

        r_offset3 = [0.0, -0.0, -0.0]
        r_offset4 = [0.0, -0.0, 0.0]

        l_offset3 = [0.0, 0.0, -0.0]
        l_offset4 = [0.0, 0.0, 0.0]


        # offsets from monday
        # r_offset1 = [0.02, -0.04, -0.02]
        # r_offset2 = [0.02, -0.04, 0.01]
        # l_offset1 = [0.03, 0.09, -0.05]
        # l_offset2 = [0.03, 0.09, 0.03]
        
        
        pos_1 = self.get_closest_valid_position(self.get_transformed_marker_positions(self.latest_marker_msg), "left")
        pos_1 = tuple(float(x) for x in pos_1)  # Ensure float conversion
        self.move_left_arm(pos_1, l_offset1)
        self.move_left_arm(pos_1, l_offset2) 
        self.move_arm_in_x_direction(self.left_moveit2, pos_1, -0.2, "left") #the inputs in this function are obsolete

        pos_2 = self.get_closest_valid_position(self.get_transformed_marker_positions(self.latest_marker_msg), "right")

        self.move_right_arm(pos_2, r_offset1)
        self.move_right_arm(pos_2, r_offset2)
        
        self.move_arm_in_x_direction(self.right_moveit2, pos_2, -0.2, "right")


    def get_transformed_marker_positions(self, msg):
        """
        Get transformed marker positions from ArUco marker messages.
        
        Args:
            msg: The ArUco marker message containing detected markers
            
        Returns:
            list: List of tuples containing transformed (x, y, z) positions.
                Returns empty list if no markers detected or transformation fails.
        """
        transformed_positions = []
        
        for marker in msg.markers:
            try:
                # Create a PoseStamped for the marker in the camera_optical_frame
                marker_pose = PoseStamped()
                marker_pose.header = marker.header
                marker_pose.pose = marker.pose
                
                # Transform the pose to the assembly_frame
                transformed_pose = self.tf_buffer.transform(
                    marker_pose, 
                    "assembly_7", 
                    timeout=rclpy.time.Duration(seconds=1.0)
                )
                
                # Extract the transformed position
                pos = transformed_pose.pose.position
                transformed_positions.append((pos.x, pos.y, pos.z))
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF2 error: {e}")
                
        if not transformed_positions:
            self.get_logger().warn("No markers detected or transformation failed!")
            
        return transformed_positions

    def get_closest_valid_position(self, positions, side):
        """
        Get the closest position to the goal that is not behind the goal (x-axis).
        
        Args:
            positions: List of (x, y, z) position tuples
            side: "left" or "right" to specify which goal to use
            
        Returns:
            tuple: Closest valid (x, y, z) position, or None if no valid positions found
        """
        if not positions:
            return None
            
        # Define goals for each side
        r_goal = np.array([0.7, 0.35, 0])
        l_goal = np.array([0.7, -0.05, 0])
        goal = r_goal if side == "right" else l_goal
        
        # Filter positions to only include those not behind the goal
        valid_positions = [(i, pos) for i, pos in enumerate(positions) if pos[0] <= goal[0]]
        
        if not valid_positions:
            return None
            
        # Calculate distances to the goal for valid positions
        distances = [(i, np.linalg.norm(np.array(pos) - goal)) for i, pos in valid_positions]
        
        # Get the index of the position with minimum distance
        closest_idx = min(distances, key=lambda x: x[1])[0]
        
        return positions[closest_idx]

    
    def move_left_arm(self, left_target, offset):
        """
        Move the left arm to a target position with a specified offset.
        """
        if left_target is None:
            self.get_logger().error("Invalid target position - target is None")
            return False
            
        try:
            self.is_executing = True
            self.left_moveit2.clear_goal_constraints()
            
            # Create a Point message with explicit float conversions
            point = Point()
            point.x = float(left_target[0] + offset[0])
            point.y = float(left_target[1] + offset[1])
            point.z = float(left_target[2] + offset[2])
            
            self.get_logger().info(f"Moving left arm to position: [{point.x}, {point.y}, {point.z}]")
            
            # Set position goal using Point message
            self.left_moveit2.set_position_goal(
                position=point,
                tolerance=float(0.01),
                weight=float(1.0)
            )
            
            left_trajectory = self.left_moveit2.plan()
            if left_trajectory is None:
                self.get_logger().error("Left arm planning failed!")
                return False
                
            success = self.left_moveit2.execute(left_trajectory)
            if not success:
                self.get_logger().error("Execution failed!")
                return False
                
            success = self.left_moveit2.wait_until_executed()
            if not success:
                self.get_logger().error("Wait until executed failed!")
                return False
            
            time.sleep(1)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error in move_left_arm: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
            
        finally:
            self.is_executing = False
    def move_right_arm(self, right_target, offset):
        self.is_executing = True
        # self.get_logger().info(f"Moving left arm to: {left_target}")        
        first_offset = offset

        self.right_moveit2.clear_goal_constraints()
        print(right_target)
        
        updated_rt = np.add(list(right_target), first_offset)
        self.right_moveit2.set_position_goal(
            position=updated_rt,
            tolerance=0.01,
            weight=1.0
        )

        self.get_logger().info(f"Moving right arm to: {updated_rt}")
        
        # Plan and execute movements for the right arm
        right_trajectory = self.right_moveit2.plan()
        if right_trajectory is not None:
            self.right_moveit2.execute(right_trajectory)
            self.right_moveit2.wait_until_executed()  # Wait until right arm completes movement
        else:
            self.get_logger().warn("Right arm planning failed!")

        # Now move both arms by -0.3 in the x direction
        time.sleep(1)
        
        
        self.is_executing = False
    

    def move_arms_to_markers(self, left_target, right_target, offset):
        self.is_executing = True
        # self.get_logger().info(f"Moving left arm to: {left_target}")        
        first_offset = offset
        second_offset = [0.0, 0.1, 0.05]

        # Clear existing constraints
        self.left_moveit2.clear_goal_constraints()
        self.right_moveit2.clear_goal_constraints()
        
        # Set position goals for left and right arms
        # self.left_moveit2.set_position_goal(
        #     position=left_target,
        #     tolerance=0.01,
        #     weight=1.0
        # )
        updated_rt = np.add(list(right_target), first_offset)
        self.right_moveit2.set_position_goal(
            position=updated_rt,
            tolerance=0.01,
            weight=1.0
        )
        
        # Plan and execute movements for the left arm
        # left_trajectory = self.left_moveit2.plan()
        # if left_trajectory is not None:
        #     self.left_moveit2.execute(left_trajectory)
        #     self.left_moveit2.wait_until_executed()  # Wait until left arm completes movement
        # else:
        #     self.get_logger().warn("Left arm planning failed!")

        # self.move_arm_in_x_direction(self.left_moveit2, left_target, -0.2, "left")

        self.get_logger().info(f"Moving right arm to: {updated_rt}")
        
        # Plan and execute movements for the right arm
        right_trajectory = self.right_moveit2.plan()
        if right_trajectory is not None:
            self.right_moveit2.execute(right_trajectory)
            self.right_moveit2.wait_until_executed()  # Wait until right arm completes movement
        else:
            self.get_logger().warn("Right arm planning failed!")

        # Now move both arms by -0.3 in the x direction
        time.sleep(2)
        
        
        self.is_executing = False

    def move_arm_in_x_direction(self, arm_moveit2, target_position, delta_x, arm_name):
        """
        Move arm in x direction after converting position to float
        """
        try:
            # Get current joint states
            current_state = arm_moveit2.joint_state
            if current_state is None:
                self.get_logger().warn("No joint state available!")
                return False
                
            # Create list of all joint positions, setting only our target joint to 0.01
            target_joint_name = "l3_1" if arm_name == "left" else "r3_1"
            self.get_logger().info(f"Moving {arm_name} arm's {target_joint_name} joint to 0.01m")
            
            joint_positions = []
            joint_names = []
            
            # Go through current joint states and create our target state with explicit float conversion
            for i, name in enumerate(current_state.name):
                if name in arm_moveit2.joint_names:  # Only include joints for this arm
                    joint_names.append(name)
                    if name == target_joint_name:
                        joint_positions.append(float(0.01))  # Explicit float conversion
                    else:
                        joint_positions.append(float(current_state.position[i]))  # Explicit float conversion
            
            # Move to the target configuration
            return arm_moveit2.move_to_configuration(
                joint_positions=joint_positions,
                joint_names=joint_names,
                tolerance=float(0.001),
                weight=float(1.0)
            )
                
        except Exception as e:
            self.get_logger().error(f"Error moving arm in x direction: {str(e)}")
            return False
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
        node.climb()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()