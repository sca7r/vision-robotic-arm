"""
The MoveIt move_group node alone, no GUI.

Pass the same camera:= value used for spawn.launch.py - see demo.launch.py.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

URDF_XACRO = (
    Path(get_package_share_directory("vision_arm_description")) / "urdf" / "vision_arm.urdf.xacro"
)


def _nodes(context, *args, **kwargs):
    # See demo.launch.py: MoveItConfigsBuilder needs a resolved string, not a
    # LaunchConfiguration, so the build is deferred into an OpaqueFunction.
    camera = LaunchConfiguration("camera").perform(context)

    moveit_config = (
        MoveItConfigsBuilder("vision_arm", package_name="vision_arm_moveit_config")
        .robot_description(file_path=URDF_XACRO, mappings={"camera": camera})
        .robot_description_semantic(file_path="config/vision_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="false"),
        OpaqueFunction(function=_nodes),
    ])
