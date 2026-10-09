from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mask_topic", default_value="/gsam2/mask_target"),
        DeclareLaunchArgument("depth_topic", default_value="/topcam/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/topcam/depth/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="topcam_depth_optical_frame"),
        DeclareLaunchArgument(
            "depth_scale_to_meters", default_value="0.001",
            description="0.001 for 16UC1 (millimeters), 1.0 if depth is already in meters (32FC1)",
        ),
        DeclareLaunchArgument(
            "z_mode", default_value="top_surface",
            description="'top_surface' (closest point to camera - for placing ON TOP of the target) or 'centroid' (mean depth)",
        ),

        Node(
            package="target_pose_ros",
            executable="target_pose_node",
            output="screen",
            parameters=[{
                "mask_topic": LaunchConfiguration("mask_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "depth_scale_to_meters": LaunchConfiguration("depth_scale_to_meters"),
                "z_mode": LaunchConfiguration("z_mode"),
            }],
        ),
    ])
