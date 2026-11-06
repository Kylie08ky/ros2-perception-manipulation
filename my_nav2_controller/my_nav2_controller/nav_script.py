#!/usr/bin/env python3

# =========================================
# File: nav_script.py
# Description: Simple navigation script based on nav2 in ROS2. Supports sending goals via terminal input, allowing the robot to navigate between predefined points and pass through an intermediate point when switching goals.
# Author: <Group 5>
# Date: <24.07.2025>
# =========================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import threading

# =============================
# Predefined goal points (in map frame)
# =============================
GOALS = {
    "goal_1": {
        "position": (0.4219396412372589, 0.20774532854557037, 0.006378173828125),  # Position (x, y, z)
        "orientation": (0.0, 0.0, 0.0, 1.0)  # Quaternion (x, y, z, w)
    },
    "goal_2": {
        "position": (-0.6399364471435547, 0.7504213452339172, 0.002471923828125),
        "orientation": (0.0, 0.0, 0.0, 1.0)
    }
}

# Intermediate waypoint (used when switching between goals)
INTERMEDIATE_POINT = {
    "position": (0.3388090133666992, 1.5037068128585815, 0.002471923828125),
    "orientation": (0.0, 0.0, 0.0, 1.0)
}


class SimpleNav(Node):
    """
    Simple navigation node. Supports sending goals via terminal input and navigating between predefined points.
    When switching between goals, the robot will pass through an intermediate point.
    """
    def __init__(self):
        super().__init__('simple_nav')
        self.pose_received = False  # Whether initial pose has been received
        self.current_goal_name = None  # Name of the current goal
        self.pending_goal_name = None  # Name of the pending final goal (for intermediate point transition)
        self.last_goal_name = None  # Name of the last reached goal
        # Subscribe to AMCL pose
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        # Create navigation action client
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def amcl_pose_callback(self, msg):
        """
        AMCL pose callback. Allows input of goals after receiving the initial pose.
        """
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info('Initial pose detected. You can now input a goal.')
            # Start input thread to avoid blocking the main thread
            threading.Thread(target=self.ask_and_send_goal, daemon=True).start()

    def ask_and_send_goal(self):
        """
        Terminal interaction: input goal name and send navigation command.
        """
        while True:
            goal_name = input('Enter the goal name (goal_1 / goal_2 / exit): ').strip()
            if goal_name == 'exit':
                self.get_logger().info('Exiting navigation input loop.')
                return
            if goal_name not in GOALS:
                print("Invalid goal. Please try again.")
                continue

            self.get_logger().info(f"Preparing to go to {goal_name} ...")

            # If switching between goal_1 and goal_2, go through the intermediate point first
            if self.last_goal_name in ['goal_1', 'goal_2'] and goal_name in ['goal_1', 'goal_2'] and goal_name != self.last_goal_name:
                self.get_logger().info("Switching goals, navigating to intermediate point first...")
                self.pending_goal_name = goal_name
                self.send_custom_goal(INTERMEDIATE_POINT)
            else:
                self.pending_goal_name = None
                self.send_goal(goal_name)
            break

    def send_goal(self, goal_name):
        """
        Send navigation goal by goal name.
        :param goal_name: Name of the goal (e.g., 'goal_1')
        """
        goal_data = GOALS[goal_name]
        self.current_goal_name = goal_name
        self.send_custom_goal(goal_data)

    def send_custom_goal(self, goal_data):
        """
        Send a custom navigation goal.
        :param goal_data: Dictionary containing position and orientation
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        pos = goal_data["position"]
        ori = goal_data["orientation"]
        goal.pose.pose.position.x = pos[0]
        goal.pose.pose.position.y = pos[1]
        goal.pose.pose.position.z = pos[2]
        goal.pose.pose.orientation.x = ori[0]
        goal.pose.pose.orientation.y = ori[1]
        goal.pose.pose.orientation.z = ori[2]
        goal.pose.pose.orientation.w = ori[3]

        self.action_client.wait_for_server()
        self.get_logger().info(f"Sending navigation goal...")
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Goal response callback. Checks if the goal was accepted.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was not accepted')
            return
        self.get_logger().info('Goal accepted, navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Navigation result callback. Handles logic after reaching the goal.
        """
        self.get_logger().info("✅ Arrived at destination.")

        # If just arrived at the intermediate point, continue to the final goal
        if self.pending_goal_name:
            self.get_logger().info(f"Proceeding to final goal: {self.pending_goal_name}")
            next_goal = self.pending_goal_name
            self.pending_goal_name = None
            self.send_goal(next_goal)
            return

        # Record the current goal and wait for the next input
        self.last_goal_name = self.current_goal_name
        threading.Thread(target=self.ask_and_send_goal, daemon=True).start()


def main(args=None):
    """
    Main function. Initializes the ROS2 node and enters the spin loop.
    """
    rclpy.init(args=args)
    node = SimpleNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
