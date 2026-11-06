#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_msgs.msg import String
import time
import math
import threading
from collections import deque
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from typing import Optional


class MoveGroupActionClient(Node):
    """
    ROS 2 node for controlling a robotic arm and gripper using MoveIt.
    - Subscribes to object detection labels.
    - Waits for user-specified label to be detected 10 times in a row.
    - Prompts user for 'pick' or 'place' command upon successful detection.
    """

    def __init__(self):
        super().__init__('move_group_action_client')

        # Initialize MoveGroup action client
        self._client = ActionClient(self, MoveGroup, 'move_action')

        # Arm joint configuration
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.joint_goal = [0.0, 0.436, 0.506, -1.239]  # Default target pose for pick/place

        # Home position to return after action
        self.home_joint_goal = [
            math.radians(-9),
            math.radians(-60),
            math.radians(62),
            math.radians(0)
        ]

        # Gripper configuration
        self.gripper_joint_name = 'gripper_left_joint'
        self.gripper_target = 0.019  # Set later based on mode (pick/place)

        # Runtime state tracking
        self.mode = None
        self.returning_home = False
        self.target_label = None               # User-specified object label to detect
        self.match_queue = deque(maxlen=10)    # Last 10 detections for stability check
        self.confirmed = False                 # True if label detected 10 times in a row

        # Subscribe to 3D object detection topic
        self.subscription_3d = self.create_subscription(
            String,
            '/detected_object_3d',
            self.detection_3d_callback,
            qos_profile_sensor_data
        )

        self.target_position = None  # Dynamic pose from detection
        self.subscription = self.create_subscription(
            String,
            '/detected_object_3d',
            self.detection_3d_callback,
            qos_profile_sensor_data
        )

        # Prompt user for label input in background
        threading.Thread(target=self.ask_target_label, daemon=True).start()

    def ask_target_label(self):
        """
        Prompt the user for the object label they want to detect.
        Starts detection monitoring once label is set.
        """
        while not self.target_label:
            label = input("Enter target label (e.g., 'red cylinder horizontal'): ").strip()
            if label:
                self.target_label = label
                self.get_logger().info(f"Waiting to detect: [{self.target_label}]...")
            else:
                print("Please enter a valid label.")

    def compute_ik(self, pose: Pose):
        """
        Call the IK service to compute joint values for a given end-effector pose.
        Returns JointState if successful, otherwise None.
        """
        from moveit_msgs.msg import RobotState
        client = self.create_client(GetPositionIK, '/compute_ik')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout.sec = 2

        # Set initial joint state (from RViz or default)
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = [
            0.0,
            math.radians(-59),
            math.radians(62),
            0.0
        ]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and result.error_code.val == 1:
            return result.solution.joint_state
        else:
            self.get_logger().error('IK computation failed.')
            return None

    def detection_callback(self, msg):
        """
        Callback for object detection topic. Parses detected label and coordinates,
        then prompts user for action selection.
        """
        if self.confirmed:
            return
        try:
            label, xyz = msg.data.split(":")
            if label.strip() != self.target_label:
                return

            self.confirmed = True
            self.get_logger().info(f'✅ Detected [{label}] once.')

            coords = xyz.strip().split(",")
            x = float(coords[0].split("=")[1])
            y = float(coords[1].split("=")[1])
            z = float(coords[2].split("=")[1])
            self.current_target_pose = (x, y, z)

            self.ask_mode_loop()

        except Exception as e:
            self.get_logger().error(f"Failed to parse /detected_object_3d: {e}")

    def detection_3d_callback(self, msg):
        """
        Callback for 3D object detection topic. Parses the detected label and coordinates,
        builds position and orientation constraints, and sends motion planning request.
        """
        if not self.confirmed or not self.mode or self.returning_home:
            return

        try:
            # Parse target label and coordinates
            label, xyz = msg.data.split(":")
            if label.strip() != self.target_label:
                return

            coords = xyz.strip().split(",")
            x = float(coords[0].split("=")[1])
            y = float(coords[1].split("=")[1])
            z = float(coords[2].split("=")[1])

            # -----------------------------
            # Build position constraint (PositionConstraint)
            # -----------------------------
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'base_link'
            position_constraint.link_name = 'gripper'  # OpenManipulator X end effector link
            position_constraint.target_point_offset.z = 0.0
            position_constraint.constraint_region.primitives = [
                SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.02, 0.02])
            ]
            position_constraint.constraint_region.primitive_poses = [
                Pose(position=Point(x=x, y=y, z=z + 0.02))  # Raise 2cm to avoid collision
            ]
            position_constraint.weight = 1.0

            # -----------------------------
            # Build orientation constraint (OrientationConstraint)
            # -----------------------------
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'base_link'
            orientation_constraint.link_name = 'gripper'  # OpenManipulator X end effector link
            orientation_constraint.orientation.y = 1.0
            orientation_constraint.orientation.w = 0.0
            orientation_constraint.absolute_x_axis_tolerance = 0.1
            orientation_constraint.absolute_y_axis_tolerance = 0.1
            orientation_constraint.absolute_z_axis_tolerance = 3.14
            orientation_constraint.weight = 1.0

            # -----------------------------
            # Combine constraints and send planning request
            # -----------------------------
            constraints = Constraints()
            constraints.position_constraints = [position_constraint]
            constraints.orientation_constraints = [orientation_constraint]

            request = MotionPlanRequest()
            request.group_name = 'arm'
            request.goal_constraints = [constraints]

            goal_msg = MoveGroup.Goal()
            goal_msg.request = request
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.look_around = False
            goal_msg.planning_options.replan = True

            self.get_logger().info(f'🎯 Planning to dynamic pose for [{label.strip()}]')
            self._client.wait_for_server()
            self._send_goal_future = self._client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

            # Prevent re-execution
            self.confirmed = False

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse 3D pose from message: {e}")

    def ask_mode_loop(self):
        """
        Prompt the user to choose between pick, place, or exit.
        Sets gripper goal and initiates motion based on selection.
        """
        while rclpy.ok():
            mode = input("Please enter the action (pick/place/exit): ").strip().lower()
            if mode == 'exit':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                return
            elif mode in ['pick', 'place']:
                self.mode = mode
                self.gripper_target = 0.019 if mode == 'pick' else -0.010
                self.returning_home = False
                self.get_logger().info(f'Executing [{mode.upper()}] action...')
                self.send_goal(self.joint_goal)
                return
            else:
                print("Invalid input. Please type 'pick', 'place', or 'exit'.")

    def send_goal(self, joint_goal):
        """
        Send a joint-space arm goal to the MoveIt action server.
        """
        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveGroup action server...')

        # Create joint constraints
        constraints = Constraints()
        for name, pos in zip(self.joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints = [jc]

        # Build the MoveGroup goal request
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints = [constraints]

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        label = 'RETURN-TO-HOME' if self.returning_home else (self.mode.upper() if self.mode else 'UNKNOWN')
        self.get_logger().info(f'Sending arm joint goal: [{label}]')
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback when arm motion goal is accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal was rejected.')
            return

        self.get_logger().info('Arm goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle result of arm motion.
        On success, continue with gripper action or return home.
        """
        result = future.result().result
        if result.error_code.val == 1:
            if not self.returning_home:
                self.get_logger().info('✅ Arm moved to goal. Pausing before gripper...')
                time.sleep(2)
                self.send_gripper_goal()
            else:
                self.get_logger().info('🏠 Returned to home position.')
                threading.Thread(target=self.ask_mode_loop, daemon=True).start()
        else:
            self.get_logger().error(f'❌ Arm movement failed. Error code: {result.error_code.val}')
            rclpy.shutdown()

    def send_gripper_goal(self):
        """
        Send a gripper command based on the mode (pick or place).
        """
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.gripper_joint_name
        jc.position = self.gripper_target
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        constraints.joint_constraints = [jc]

        request = MotionPlanRequest()
        request.group_name = 'gripper'
        request.goal_constraints = [constraints]

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.get_logger().info(f'Sending gripper goal ({self.gripper_target:.3f} m)...')
        self._gripper_goal_future = self._client.send_goal_async(goal_msg)
        self._gripper_goal_future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        """
        Callback when gripper goal is accepted.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal was rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Gripper goal accepted, waiting for result...')
        self._gripper_result_future = goal_handle.get_result_async()
        self._gripper_result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        """
        Handle result of gripper motion.
        On success, command return to home pose.
        """
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info(f'🦾 Gripper [{self.mode}] action complete.')
            time.sleep(1)
            self.returning_home = True
            self.send_goal(self.home_joint_goal)
        else:
            self.get_logger().error(f'❌ Gripper failed. Error code: {result.error_code.val}')
            rclpy.shutdown()


def main(args=None):
    """
    Main entry point for the ROS node.
    Initializes and spins the action client node.
    """
    rclpy.init(args=args)
    node = MoveGroupActionClient()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()

    
