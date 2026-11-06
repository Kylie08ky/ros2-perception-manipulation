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


class MoveGroupActionClient(Node):
    """
    ROS 2 node for controlling a robotic arm and gripper using MoveIt.
    - Subscribes to object detection labels.
    - Waits for user-specified label to be detected 10 times in a row.
    - Prompts user for 'pick' or 'place' command upon successful detection.
    """

    def __init__(self):
        """
        Initialize the MoveGroupActionClient node, set up action client, state variables,
        and start the detection label subscription and user input thread.
        """
        super().__init__('move_group_action_client')

        # Initialize MoveGroup action client for arm control
        self._client = ActionClient(self, MoveGroup, 'move_action')

        # Arm joint configuration (example: 4-DOF arm)
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

        # Subscribe to object detection label topic
        self.subscription = self.create_subscription(
            String,
            '/detected_object_class',
            self.detection_callback,
            qos_profile_sensor_data
        )

        # Prompt user for label input in background thread
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

    def detection_callback(self, msg):
        """
        Callback triggered when a detection label is received.
        If 10 consecutive detections match the target label, proceed to pick/place interaction.
        """
        if self.confirmed:
            return  # Detection already confirmed

        detected_label = msg.data.strip()
        self.match_queue.append(detected_label)

        # Check if last 10 messages match the target
        if len(self.match_queue) == 10 and all(label == self.target_label for label in self.match_queue):
            self.get_logger().info(f"✅ Detected [{self.target_label}] 10 times consecutively.")
            self.confirmed = True
            # Start user interaction for pick/place in a new thread
            threading.Thread(target=self.ask_mode_loop, daemon=True).start()

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

        # Create joint constraints for each joint
        constraints = Constraints()
        for name, pos in zip(self.joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        # Build the MoveGroup goal request
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        label = 'RETURN-TO-HOME' if self.returning_home else self.mode.upper()
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
                # Restart user interaction for next operation
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
        constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = 'gripper'
        request.goal_constraints.append(constraints)

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
