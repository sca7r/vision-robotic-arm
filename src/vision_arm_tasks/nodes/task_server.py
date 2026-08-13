#!/usr/bin/env python3
"""Pick a coloured cube and place it somewhere. One node, one cycle:

    home -> detect -> approach -> grasp -> lift -> carry -> release -> home

Drive it with a string on /task/command:

    ros2 topic pub --once /task/command std_msgs/String "data: 'red left'"
    ros2 topic pub --once /task/command std_msgs/String "data: 'blue 0.1 -0.42'"

and read /task/result. ponytail: strings, not a custom action - nothing wants
cancel or progress feedback yet, and this is exactly what the language layer
will publish. Swap in an action when you want to abort a motion mid-flight.

MoveIt is driven from plain rclpy: /compute_ik turns a Cartesian target into
joint angles, /move_action plans and executes to those angles. No moveit_py, no
MoveIt Task Constructor - neither is installed and a pick is five moves.
"""
import math
import os
import random
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, RobotState)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

ARM = ["joint1_base_rotate", "joint2_shoulder", "joint3_elbow",
       "joint4_forearm", "joint5_wrist", "joint6_gripper_mount"]
JAWS = ["joint_jaw1", "joint_jaw2"]


def parse_command(text, places):
    """'red left' or 'red 0.1 -0.42' -> ('red', (x, y)). Raises ValueError.

    Both forms are supported on purpose: a named spot for a person talking, and
    explicit coordinates for anything generating them.
    """
    parts = text.strip().lower().split()
    if not parts:
        raise ValueError("empty command")
    colour, rest = parts[0], parts[1:]
    if not rest:
        raise ValueError(f"no destination in '{text}'")
    if len(rest) == 1:
        if rest[0] not in places:
            raise ValueError(f"unknown place '{rest[0]}', known: {sorted(places)}")
        return colour, tuple(places[rest[0]])
    if len(rest) == 2:
        try:
            return colour, (float(rest[0]), float(rest[1]))
        except ValueError:
            raise ValueError(f"'{rest[0]} {rest[1]}' is not an x y pair")
    raise ValueError(f"expected '<colour> <place>' or '<colour> <x> <y>', got '{text}'")


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        default = os.path.join(
            get_package_share_directory("vision_arm_tasks"), "config", "poses.yaml")
        self.declare_parameter("poses_file", default)
        with open(self.get_parameter("poses_file").value) as handle:
            self.cfg = yaml.safe_load(handle)

        self.cubes = {}
        self.pending = None
        self.joints = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Detection3DArray, "/detections", self._on_detections, 10)
        self.create_subscription(String, "/task/command", self._on_command, 10)
        self.result = self.create_publisher(String, "/task/result", 10)
        self.jaws = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        if not self.ik.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("no /compute_ik - is move_group running?")
        if not self.move.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("no /move_action - is move_group running?")

    def _on_joints(self, msg):
        got = dict(zip(msg.name, msg.position))
        if all(j in got for j in ARM):
            self.joints = [got[j] for j in ARM]

    def _on_detections(self, msg):
        for d in msg.detections:
            p = d.results[0].pose.pose.position
            self.cubes[d.results[0].hypothesis.class_id] = (p.x, p.y, p.z)

    def _on_command(self, msg):
        # Queue it: the cycle below makes blocking service and action calls,
        # which cannot run inside a subscription callback.
        self.pending = msg.data

    def say(self, text):
        self.get_logger().info(text)
        self.result.publish(String(data=text))

    # --- primitives ----------------------------------------------------------

    def solve_ik(self, xyz, jaws):
        """Joint angles putting the TCP at xyz, jaws pointing straight down.

        `jaws` is the jaw position the arm will be HOLDING when it makes this
        move, and it matters: collision checking with the jaws open validates a
        state the robot is never in while carrying, and closing them on a cube
        swings the jaw meshes 3 cm into the forearm's collision box. That is a
        real, configuration-dependent contact (jaw1 <-> link4_forearm shows up
        in ~25% of random states), so it has to be checked in the configuration
        that will actually be executed.

        KDL's IK is also a local numeric solver, so it succeeds or fails
        depending on where it starts. Try the arm's current state first, then
        home, then perturbations of home; a target that survives none of those
        is genuinely out of reach.
        """
        rng = random.Random(0)
        home = self.cfg["home"]
        seeds = [self.joints or home, home]
        seeds += [[a + rng.gauss(0, 0.4) for a in home] for _ in range(6)]
        # A cube is square, so which way the jaws close does not matter: yaw about
        # the vertical is free, and sweeping it turns targets that have no
        # solution at one jaw angle into ones that do. The blue cube is exactly
        # such a target - reachable, but not with the jaws left at yaw 0.
        yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        last = None
        for yaw, seed in ((y, s) for s in seeds for y in yaws):
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
            # Rotation about base Z only: -Z keeps pointing at the bench, which
            # is what makes this a top-down grasp (see gripper.xacro).
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.ik_link_name = "tcp"
            req.ik_request.pose_stamped = ps
            req.ik_request.timeout.sec = 2
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state = RobotState()
            req.ik_request.robot_state.joint_state.name = ARM + JAWS
            req.ik_request.robot_state.joint_state.position = (
                [float(v) for v in seed] + [float(jaws), float(-jaws)])
            future = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
            res = future.result()
            if res is not None and res.error_code.val == 1:
                got = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [got[j] for j in ARM]
            last = "timeout" if res is None else res.error_code.val
        raise RuntimeError(
            f"no IK for {tuple(round(v, 3) for v in xyz)} after "
            f"{len(seeds) * len(yaws)} seed/yaw combinations ({last})")

    def move_to(self, q, what):
        """Plan and execute to joint angles q. Collision-aware, via move_group."""
        request = MotionPlanRequest()
        request.group_name = "arm"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value),
                            tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for name, value in zip(ARM, q)])]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(plan_only=False)

        future = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{what}: move_group rejected the goal")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120.0)
        res = result.result()
        if res is None or res.result.error_code.val != 1:
            code = "timeout" if res is None else res.result.error_code.val
            raise RuntimeError(f"{what}: motion failed ({code})")

    def set_jaws(self, position):
        """Jaws mirror: jaw2 takes the negation of jaw1 (see gripper.xacro)."""
        self.jaws.publish(Float64MultiArray(data=[float(position), float(-position)]))
        # ponytail: sleep instead of watching /joint_states. The jaws are a
        # position controller with a fixed short stroke; wire up feedback if a
        # grasp ever needs to report that it closed on nothing.
        end = time.time() + 1.5
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go_home(self):
        self.move_to(self.cfg["home"], "home")

    # --- the cycle -----------------------------------------------------------

    def run(self, text):
        colour, (px, py) = parse_command(text, self.cfg["places"])
        if colour not in self.cubes:
            raise RuntimeError(
                f"no {colour} cube in /detections (seen: {sorted(self.cubes) or 'nothing'})")
        x, y, z = self.cubes[colour]
        lift = self.cfg["approach"]
        self.say(f"picking {colour} at ({x:.3f}, {y:.3f}, {z:.3f}) -> ({px:.3f}, {py:.3f})")

        opened, grip = self.cfg["open"], self.cfg["grip"]
        # Each waypoint carries the jaw position it will be executed with, so
        # "lift" and "approach" are separate solutions even though they are the
        # same point in space - one is flown with an empty gripper, the other
        # with a cube in it, and they collision-check differently.
        route = [
            ("approach", (x, y, z + lift), opened),
            ("descend", (x, y, z), opened),
            ("lift", (x, y, z + lift), grip),
            ("carry", (px, py, z + lift), grip),
            ("lower", (px, py, z), grip),
            ("retreat", (px, py, z + lift), opened),
        ]
        # Solve every waypoint BEFORE moving anything. A place spot that turns
        # out to be unreachable halfway through leaves the arm stranded holding
        # a cube; found now, it costs nothing and the arm has not left home.
        plan = [(what, self.solve_ik(xyz, jaws)) for what, xyz, jaws in route]

        self.set_jaws(opened)
        for what, q in plan:
            self.move_to(q, what)
            if what == "descend":
                self.set_jaws(grip)
            elif what == "lower":
                self.set_jaws(opened)
        self.go_home()

        # Verify, do not assume. A jaw that closes on nothing, or a cube that
        # slips during the lift, produces exactly the same sequence of
        # successful moves as a real pick - the arm cannot tell the difference,
        # so ask the detector where the cube actually ended up. Home is the pose
        # everything is visible from, which is what makes this check possible.
        self.cubes.pop(colour, None)
        deadline = time.time() + 10.0
        while colour not in self.cubes and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if colour not in self.cubes:
            raise RuntimeError(f"moved {colour} but cannot see it from home to confirm")
        ax, ay = self.cubes[colour][:2]
        off = math.hypot(ax - px, ay - py)
        if off > self.cfg["place_tolerance"]:
            raise RuntimeError(
                f"{colour} ended at ({ax:.3f}, {ay:.3f}), {off * 100:.1f} cm from "
                f"({px:.3f}, {py:.3f}) - the grasp slipped or closed on nothing")
        self.say(f"done: {colour} is at ({ax:.3f}, {ay:.3f}), {off * 1000:.0f} mm from target")


def main():
    rclpy.init()
    node = TaskServer()
    node.say("going home, waiting for a command on /task/command")
    node.go_home()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pending is None:
            continue
        text, node.pending = node.pending, None
        try:
            node.run(text)
        except (ValueError, RuntimeError) as exc:
            node.say(f"failed: {exc}")


if __name__ == "__main__":
    sys.exit(main())
