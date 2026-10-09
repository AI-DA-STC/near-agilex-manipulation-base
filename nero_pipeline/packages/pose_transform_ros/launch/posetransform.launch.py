from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("grasps_topic", default_value="/contact_graspnet/grasps"),
        DeclareLaunchArgument("target_pose_topic", default_value="/target_pose/pose"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="2.0"),

        Node(
            package="pose_transform_ros",
            executable="pose_transform_node",
            output="screen",
            parameters=[{
                "grasps_topic": LaunchConfiguration("grasps_topic"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "tf_timeout_sec": LaunchConfiguration("tf_timeout_sec"),
            }],
        ),
    ])
