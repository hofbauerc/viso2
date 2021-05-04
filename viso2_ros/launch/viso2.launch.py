import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="viso2_ros",
            executable="stereo_odometer",
            parameters=[os.path.join(
                get_package_share_directory("viso2_ros"),
                "cfg",
                "example.yaml"
            )],
            remappings=[
                ("left/image_rect", "/camera/left/image_rect"),
                ("right/image_rect", "/camera/right/image_rect"),
                ("left/image_rect/camera_info", "/camera/left/camera_info"),
                ("right/image_rect/camera_info", "/camera/right/camera_info")
            ]
        )
    ])