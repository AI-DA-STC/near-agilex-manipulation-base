from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/topcam/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/topcam/depth/camera_info"),
        DeclareLaunchArgument("mask_topic", default_value="/gsam2/mask_source"),
        DeclareLaunchArgument("camera_frame", default_value="topcam_depth_optical_frame"),
        DeclareLaunchArgument("conda_env_name", default_value="contact_graspnet"),
        DeclareLaunchArgument(
            "inference_script",
            default_value="~/contact_graspnet_ros_scripts/contact_graspnet_inference.py",
        ),
        DeclareLaunchArgument(
            "ckpt_dir",
            default_value="~/contact_graspnet_pytorch/checkpoints/contact_graspnet",
        ),
        DeclareLaunchArgument(
            "depth_scale_to_meters", default_value="0.001",
            description="0.001 if depth topic is in millimeters (16UC1), 1.0 if already in meters (32FC1)",
        ),

        Node(
            package="contact_graspnet_ros",
            executable="contact_graspnet_node",
            output="screen",
            parameters=[{
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "mask_topic": LaunchConfiguration("mask_topic"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "conda_env_name": LaunchConfiguration("conda_env_name"),
                "inference_script": LaunchConfiguration("inference_script"),
                "ckpt_dir": LaunchConfiguration("ckpt_dir"),
                "depth_scale_to_meters": LaunchConfiguration("depth_scale_to_meters"),
            }],
        ),
    ])
