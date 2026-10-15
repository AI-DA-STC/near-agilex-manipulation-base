from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/topcam/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/topcam/depth/camera_info"),
        DeclareLaunchArgument("mask_topic", default_value="/gsam2/mask_source"),
        DeclareLaunchArgument("camera_frame", default_value="", description="empty = use the frame_id of the depth camera_info (recommended); or force a frame name"),
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
        DeclareLaunchArgument(
            "use_worker", default_value="true",
            description="true: keep the model loaded in a persistent worker (fast); false: old one-process-per-call",
        ),
        DeclareLaunchArgument(
            "forward_passes", default_value="1",
            description="Contact-GraspNet forward passes (fixed when the worker starts); more passes = more candidate grasps, slower",
        ),
        DeclareLaunchArgument(
            "recentre_on_object", default_value="false",
            description="true: move every grasp's closing centre onto the middle of the object's top face (keeps orientation); can also be set live",
        ),
        DeclareLaunchArgument(
            "recentre_depth_below_top_m", default_value="0.02",
            description="how far below the object's top surface the closing centre is put when recentre_on_object is on (metres)",
        ),
        DeclareLaunchArgument(
            "retries_if_empty", default_value="2",
            description="worker only: predict again up to this many times when a prediction returns 0 grasps",
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
                "use_worker": LaunchConfiguration("use_worker"),
                "forward_passes": LaunchConfiguration("forward_passes"),
                "retries_if_empty": LaunchConfiguration("retries_if_empty"),
                "recentre_on_object": LaunchConfiguration("recentre_on_object"),
                "recentre_depth_below_top_m": LaunchConfiguration("recentre_depth_below_top_m"),
            }],
        ),
    ])
