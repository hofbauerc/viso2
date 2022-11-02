import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

config = os.path.join(get_package_share_directory('viso2_ros'),'cfg','params.yaml')

def generate_launch_description():
    return LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '../kitti/kitti_2011_09_26_drive_0002_synced', '-r 0.1', '--clock'],
        #    cmd=['ros2', 'bag', 'play', '../st_valentin/rosbag2_2022_06_28-12_16_15', '-r 0.5', '--clock'],
        #    cmd=['ros2', 'bag', 'play', '../simulation/offroad/restamped', '-r 0.1', '--clock'],
        #    #cmd=['ros2', 'bag', 'play', '../simulation/plane_and_wall', '-r 1.3'],
        #    output='screen'
        ),


        Node(
            package="viso2_ros",
            executable="stereo_odometer",
            parameters=[config],
            remappings=[
                ("left/image_rect", "/kitti/camera_gray_left/image_raw"),
                ("right/image_rect", "/kitti/camera_gray_right/image_raw"),
                ("left/image_rect/camera_info", "/kitti/camera_gray_left/camera_info"),
                ("right/image_rect/camera_info", "/kitti/camera_gray_right/camera_info")
            ]
        ),
        Node(
            package="viso2_ros",
            executable="transform_odom_node",
            parameters=[config]
        )
    ])