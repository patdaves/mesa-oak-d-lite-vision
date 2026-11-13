import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("mesa_vision"), "config", "vision.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("blob_path", description="Path to the MobileNet-SSD .blob for RVC2"),
        Node(package="mesa_vision", executable="oak_camera_node", name="oak_camera_node",
             parameters=[cfg, {"blob_path": LaunchConfiguration("blob_path")}], output="screen"),
        Node(package="mesa_vision", executable="person_tracker_node", name="person_tracker_node",
             parameters=[cfg], output="screen"),
        # Camera mount: base_link -> camera optical frame. Replace the numbers after measuring the robot
        # (x forward, y left, z up in metres; yaw, pitch, roll in radians; optical frame has z forward).
        Node(package="tf2_ros", executable="static_transform_publisher", name="oak_mount_tf",
             arguments=["0.10", "0.0", "1.00", "-1.5708", "0.0", "-1.5708", "base_link",
                        "oak_rgb_camera_optical_frame"]),
        Node(package="mesa_vision", executable="obstacle_confirm_node", name="obstacle_confirm_node",
             parameters=[cfg], output="screen"),
    ])
