from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mask_topic", default_value="/gsam2/mask_target"),
        DeclareLaunchArgument("depth_topic", default_value="/topcam/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/topcam/depth/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="", description="empty = use the frame_id of the depth camera_info (recommended); or force a frame name"),
        DeclareLaunchArgument(
            "depth_scale_to_meters", default_value="0.001",
            description="0.001 for 16UC1 (millimeters), 1.0 if depth is already in meters (32FC1)",
        ),
        DeclareLaunchArgument(
            "z_mode", default_value="floor",
            description="'floor' (median depth near the centre = the plate's inner surface; default), 'top_surface' (single closest pixel: the rim or noise) or 'centroid' (mean depth)",
        ),
        DeclareLaunchArgument(
            "center_mode", default_value="hull",
            description="how the plate centre is found: 'hull' (default), 'mean' (the original average of all mask pixels), 'circle' or 'bbox'",
        ),
        DeclareLaunchArgument("center_fraction", default_value="0.5",
                              description="'floor' height uses the pixels within this fraction of the plate radius around the centre"),

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
                "center_mode": LaunchConfiguration("center_mode"),
                "center_fraction": ParameterValue(LaunchConfiguration("center_fraction"), value_type=float),
            }],
        ),
    ])
