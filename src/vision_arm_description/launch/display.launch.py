from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vision_arm_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "vision_arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "vision_arm.rviz"])

    # camera:=true draws the D405 solid and enables the simulated sensor. Off by
    # default (see vision_arm.urdf.xacro) - this is the only way to set it from the
    # command line, so without this plumbing the arg was unreachable.
    camera_arg = DeclareLaunchArgument("camera", default_value="false")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path, " camera:=", LaunchConfiguration("camera")]),
            value_type=str,
        )
    }

    return LaunchDescription([
        camera_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
