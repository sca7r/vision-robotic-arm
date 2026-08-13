from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("vision_arm_perception"), "config", "cube_detector.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="vision_arm_perception",
            executable="cube_detector.py",
            name="cube_detector",
            output="screen",
            parameters=[params,
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
