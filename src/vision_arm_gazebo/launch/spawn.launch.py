import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_arm_description = FindPackageShare("vision_arm_description")
    vision_arm_gazebo = FindPackageShare("vision_arm_gazebo")
    xacro_path = PathJoinSubstitution([vision_arm_description, "urdf", "vision_arm.urdf.xacro"])
    world_path = PathJoinSubstitution([vision_arm_gazebo, "worlds", "vision_arm_world.sdf"])
    bridge_config = PathJoinSubstitution([vision_arm_gazebo, "config", "bridge.yaml"])

    # URDF mesh URIs are "package://vision_arm_description/meshes/...", which sdformat
    # rewrites to "model://vision_arm_description/meshes/...". Gazebo only resolves
    # that against GZ_SIM_RESOURCE_PATH, so point it at the share dir that
    # contains the "vision_arm_description" folder (i.e. share's parent-of-package).
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[PathJoinSubstitution([vision_arm_description, os.pardir])],
    )

    # This box is a VM with no GPU passthrough: ogre2's default EGL probing
    # fails ("VMware: No 3D enabled" / dri2 errors) and stalls every physics
    # step, tanking RTF to ~0.006 even for an empty world. Forcing Mesa's
    # surfaceless software path restores RTF ~1.0. Harmless on a real GPU box
    # but remove these two if you ever want hardware rendering.
    egl_surfaceless = SetEnvironmentVariable(name="EGL_PLATFORM", value="surfaceless")
    software_gl = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }

    # headless:=true runs the server only (-s) - closing the GUI window
    # otherwise shuts down the whole launch. Used for scripted/CI runs.
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r"]}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -s"]}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "vision_arm", "-z", "0.01"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # controller_manager only exists once gz_ros2_control loads the robot inside
    # Gazebo, so spawners are chained off the spawn_robot process exiting (it's a
    # short-lived "create" call, not a persistent node). Chained one-at-a-time
    # (not all three off the same exit event) - running them concurrently raced
    # on controller_manager's switch-controller lock and killed each other with
    # "can not be configured from 'active' state" errors. The first spawner also
    # needs a few seconds' head start after spawn - controller_manager's realtime
    # loop is still ramping up right after Gazebo attaches gz_ros2_control, and
    # activating immediately hits "Switch controller timed out after 5 seconds!".
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )
    after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        egl_surfaceless,
        software_gl,
        gz_resource_path,
        headless_arg,
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
        after_spawn,
        after_joint_state_broadcaster,
        after_arm_controller,
    ])
