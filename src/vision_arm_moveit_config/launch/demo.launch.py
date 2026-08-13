"""
RViz MotionPlanning UI wired to a robot already spawned in Gazebo.

`vision_arm_gazebo spawn.launch.py` owns Gazebo, robot_state_publisher,
and the ros2_control controllers; this just adds MoveIt + RViz on top.

Pass the same camera:= value you gave spawn.launch.py. MoveIt builds its own copy
of robot_description, so if the two disagree the planning scene will not match the
robot that is actually in the world.
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
RVIZ_CONFIG = (
    Path(get_package_share_directory("vision_arm_moveit_config")) / "config" / "moveit.rviz"
)


def _nodes(context, *args, **kwargs):
    # MoveItConfigsBuilder expands the xacro eagerly, in plain Python, so it needs a
    # concrete string - it cannot take a LaunchConfiguration. OpaqueFunction defers
    # this until launch arguments have actually been resolved.
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(RVIZ_CONFIG)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="false"),
        OpaqueFunction(function=_nodes),
    ])
