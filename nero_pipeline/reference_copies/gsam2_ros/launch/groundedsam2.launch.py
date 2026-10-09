from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic", default_value="/topcam/color/image_raw",
            description="RGB image topic to segment (top-down camera per architecture)",
        ),
        DeclareLaunchArgument(
            "source_prompt", default_value="carrot.",
            description='Grounding DINO prompt for the object to pick up, '
                        'must end with a period, e.g. "carrot."',
        ),
        DeclareLaunchArgument(
            "target_prompt", default_value="plate.",
            description='Grounding DINO prompt for the placement target, '
                        'must end with a period, e.g. "plate."',
        ),
        DeclareLaunchArgument(
            "conda_env_name", default_value="grounded_sam2",
        ),
        DeclareLaunchArgument(
            "gsam2_repo_dir",
            default_value="~/near-agilex-manipulation-base/external/Grounded-SAM-2",
        ),
        DeclareLaunchArgument(
            "inference_script",
            default_value="~/gsam2_ros_scripts/grounded_sam2_inference.py",
        ),
        DeclareLaunchArgument("box_threshold", default_value="0.25"),
        DeclareLaunchArgument("text_threshold", default_value="0.3"),

        Node(
            package="gsam2_ros",
            executable="gsam2_segmentation_node",
            output="screen",
            parameters=[{
                "camera_topic": LaunchConfiguration("camera_topic"),
                "source_prompt": LaunchConfiguration("source_prompt"),
                "target_prompt": LaunchConfiguration("target_prompt"),
                "conda_env_name": LaunchConfiguration("conda_env_name"),
                "gsam2_repo_dir": LaunchConfiguration("gsam2_repo_dir"),
                "inference_script": LaunchConfiguration("inference_script"),
                "box_threshold": LaunchConfiguration("box_threshold"),
                "text_threshold": LaunchConfiguration("text_threshold"),
            }],
        ),
    ])
