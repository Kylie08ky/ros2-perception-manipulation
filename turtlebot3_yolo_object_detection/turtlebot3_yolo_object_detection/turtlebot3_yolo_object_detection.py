#!/usr/bin/env python3

"""
Object detection node for TurtleBot3 using YOLO and ROS2.
Subscribes to camera and lidar topics, performs object detection,
publishes detection results and fuses 2D/3D information.
"""

import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import qos_profile_sensor_data
import tf_transformations
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

class ObjectDetectionNode(Node):
    """
    ROS2 Node for object detection and sensor fusion on TurtleBot3.
    Subscribes to camera and lidar, runs YOLO detection, fuses with lidar,
    and publishes 2D/3D detection results.
    """

    def __init__(self):
        super().__init__('turtlebot3_object_detection_node')

        # Declare and get YOLO model path parameter
        self.declare_parameter('model_path', '~/Downloads/best.pt')
        model_path = os.path.expanduser(
            self.get_parameter('model_path').get_parameter_value().string_value)

        # Load YOLO model
        self.model = YOLO(model_path)

        # Image parameters
        self.image_width = 640  # Camera image width in pixels
        self.fov_deg = 60.0     # Estimated horizontal field of view in degrees

        # CV bridge for ROS <-> OpenCV image conversion
        self.bridge = CvBridge()

        # Subscribe to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # Publisher for fused 3D detection results
        self.fused_pub = self.create_publisher(String, '/detected_object_3d', 10)
        # Store latest lidar scan
        self.latest_scan = None

        # Subscribe to raw camera image
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        # Publisher for annotated/compressed detection image
        self.image_pub = self.create_publisher(
            CompressedImage, '/camera/detections/compressed', 10)
        # Publisher for detected object pixel location
        self.detection_pub = self.create_publisher(PointStamped, '/detected_object_pixel', 10)
        # Publisher for detected object class name
        self.class_pub = self.create_publisher(String, '/detected_object_class', 10)
        # Publisher for detected object info string
        self.object_info_pub = self.create_publisher(String, '/detected_object_info', 10)

        # TF2 Buffer and Listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def image_callback(self, msg):
        """
        Callback for camera image topic. Runs YOLO detection, fuses with lidar,
        publishes detection results and annotated image.
        """
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            # Run YOLO detection
            results = self.model(frame)

            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    # Get bounding box and class
                    xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                    x_center = (xyxy[0] + xyxy[2]) / 2
                    y_center = (xyxy[1] + xyxy[3]) / 2
                    class_id = int(box.cls[0].item())
                    class_name = self.model.names[class_id]

                    # Publish detected pixel location
                    point_msg = PointStamped()
                    point_msg.header = msg.header
                    point_msg.point.x = x_center
                    point_msg.point.y = y_center
                    point_msg.point.z = 0.0
                    self.detection_pub.publish(point_msg)

                    # Publish detected class name
                    class_msg = String()
                    class_msg.data = class_name
                    self.class_pub.publish(class_msg)

                    # Publish info string
                    info_str = f"{class_name} at (u={x_center:.1f}, v={y_center:.1f})"
                    info_msg = String()
                    info_msg.data = info_str
                    self.object_info_pub.publish(info_msg)

                    self.get_logger().info(info_str)

                    # If lidar scan is available, fuse for 3D position
                    if self.latest_scan is not None:
                        angle_per_pixel = self.fov_deg / self.image_width
                        relative_angle_deg = (x_center - self.image_width / 2) * angle_per_pixel
                        theta_rad = math.radians(relative_angle_deg)

                        scan = self.latest_scan
                        index_center = int((theta_rad - scan.angle_min) / scan.angle_increment)
                        best_distance = float('inf')
                        best_index = -1

                        # Camera to lidar offset (meters)
                        camera_to_lidar_offset_x = 0.03
                        camera_to_lidar_offset_y = 0.00

                        # Search for best lidar range near center index
                        min_distance = float('inf')
                        best_index = -1
                        for offset in range(-3, 4):  # Search ±3 indices
                            i = index_center + offset
                            if 0 <= i < len(scan.ranges):
                                d = scan.ranges[i]
                                if math.isfinite(d) and 0.05 < d < min_distance:
                                    min_distance = d
                                    best_index = i

                        if best_index >= 0 and min_distance < 1.0:
                            theta_used = scan.angle_min + best_index * scan.angle_increment
                            # Calculate camera frame coordinates
                            x_cam = min_distance * math.cos(theta_used) - camera_to_lidar_offset_x
                            y_cam = min_distance * math.sin(theta_used) - camera_to_lidar_offset_y
                            z_cam = 0.05  # Assume fixed height

                            try:
                                # Transform from camera_link to base_link
                                t: TransformStamped = self.tf_buffer.lookup_transform(
                                    "base_link",
                                    "camera_link",
                                    Time())

                                # Apply transform
                                x = t.transform.translation.x + x_cam
                                y = t.transform.translation.y + y_cam
                                z = t.transform.translation.z + z_cam
                            except Exception as e:
                                self.get_logger().warn(f"TF transform failed: {e}")
                                continue

                            # Filter out objects too far in x direction
                            if x > 1.0:
                                self.get_logger().warn(f"Skipping [{class_name}] because x={{x:.2f}} > 1.0")
                            else:
                                result_str = f"{class_name}: x={x:.2f}, y={y:.2f}, z={z:.2f}"
                                fused_msg = String()
                                fused_msg.data = result_str
                                self.fused_pub.publish(fused_msg)
                                self.get_logger().info(f"3D Detection: {result_str} (θ={{math.degrees(theta_used):.1f}}°, r={{min_distance:.2f}})")
                        else:
                            # Warn if no valid lidar range
                            self.get_logger().warn(f"Skipping [{class_name}] due to invalid or distant range (r={min_distance:.2f})")
                        # Fix: Remove invalid and duplicate code fragments
                        self.get_logger().warn(f"Skipping [{class_name}] due to invalid or distant range (r={min_distance:.2f})")

            # Publish annotated image with detections
            annotated_frame = np.array(results[0].plot())
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            success, encoded_image = cv2.imencode('.jpg', annotated_frame)
            if success:
                compressed_image = CompressedImage()
                compressed_image.header = msg.header
                compressed_image.format = 'jpeg'
                compressed_image.data = encoded_image.tobytes()
                self.image_pub.publish(compressed_image)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """
        Callback for lidar scan topic. Stores the latest scan for fusion.
        """
        self.latest_scan = msg

def main(args=None):
    """
    Main entry point for the node.
    """
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
