from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='~/Downloads/best.pt',
        description='Path to the YOLO model file'
    )
    model_path = LaunchConfiguration('model_path')

    object_detection_node = Node(
        package='turtlebot3_yolo_object_detection',
        executable='turtlebot3_yolo_object_detection',
        name='turtlebot3_object_detection_node',
        output='screen',
        parameters=[{'model_path': model_path}],
        remappings=[
            ('/camera/image_raw', '/pi_camera/image_raw')  # ⭐ Remap topic
        ]
    )

    return LaunchDescription([
        model_path_arg,
        object_detection_node,
    ])