"""The ROS 2 side of a grasp trial: place a cube, reset the arm, run one attempt.

Everything here talks to the live stack (Gazebo, MoveIt, the task server) and
nothing here knows about learning. `residual_env.py` wraps this in a Gymnasium
environment; `eval.py` drives it directly with a zero correction to measure the
scripted baseline. Keeping the two apart means the baseline and the policy are
measured through exactly the same code.

Requires the full stack up: spawn.launch.py, move_group.launch.py,
tools/fake_detections.py, task_server.py.
"""
import math
import os
import random
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

WORLD = "vision_arm_world"
MOUNT_Z = 0.7343      # base_link height in world; keep in sync with the xacro
BENCH_Z = 0.775       # cube centre height resting on the bench, in world
ARM = ["joint1_base_rotate", "joint2_shoulder", "joint3_elbow",
       "joint4_forearm", "joint5_wrist", "joint6_gripper_mount"]

# Where cubes we are not measuring get parked, so they cannot be what the arm
# bumps into or what the detector confuses for the one under test.
PARK = {"cube_green": (0.60, -0.10), "cube_blue": (-0.60, -0.10)}

# Sampling box on the bench, in base_link x/y, and the annulus inside it.
#
# The radius bounds are MEASURED, not geometric. tools/reach_map.py reports
# 0.21 to 0.65, but that is pure geometry: it ignores collisions and whether
# KDL can actually find the solution. Scoring 60 randomised placements across
# the geometric range gave, by radius:
#
#     0.25-0.40   2/6    close in, where a top-down grasp has to fold the arm
#     0.40-0.48  22/28
#     0.48-0.56  13/23
#
# so the practical envelope is 0.40 to 0.56. Sampling outside it does not make
# the test harder in a useful way, it just adds failures no grasp correction
# can address: a residual clipped to 15 mm cannot rescue a placement that has
# no inverse kinematics solution at all. Those failures are constant noise in
# every measurement and in every training reward.
X_RANGE = (-0.28, 0.28)
Y_RANGE = (-0.56, -0.34)
R_RANGE = (0.40, 0.56)

# How far a correction may move the grasp point, per axis, in metres. This must
# match RESIDUAL_LIMIT in vision_arm_tasks/nodes/task_server.py, which is the
# authority: it clips whatever arrives on /grasp_residual regardless of what is
# published. A mismatch here is not dangerous, it just wastes action range.
# test_env.py checks the two agree.
RESIDUAL_LIMIT = 0.015


def _gz(args, tries=5):
    """Run a gz command, retrying the transport flakes.

    gz transport intermittently answers a busy server with "Host unreachable".
    That is transient and it is fatal to an unattended run of several hundred
    trials.
    """
    for attempt in range(tries):
        out = subprocess.run(args, capture_output=True, text=True).stdout
        if out.strip():
            return out
        time.sleep(0.5 * (attempt + 1))
    return ""


def set_pose(model, x, y, z):
    """Teleport a model.

    Orientation is sent explicitly: an omitted quaternion arrives as all zeros,
    which is not a rotation and is silently rejected.
    """
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
    out = _gz(["gz", "service", "-s", f"/world/{WORLD}/set_pose",
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", "10000", "--req", req])
    return "true" in out.lower()


class GraspBench:
    """One grasp attempt at a time, against the running stack."""

    def __init__(self, colour="red", place="left", seed=0, timeout=180.0,
                 radius=None):
        # radius overrides R_RANGE, for probing a band deliberately, such as the
        # close in one where straight down grasps used to fail.
        self.r_range = tuple(radius) if radius else R_RANGE
        self.colour = colour
        self.model = f"cube_{colour}"
        self.timeout = timeout
        self.rng = random.Random(seed)

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("grasp_bench")
        self.cmd = self.node.create_publisher(String, "/task/command", 10)
        self.jaws = self.node.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10)
        self.traj = self.node.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.residual = self.node.create_publisher(
            Float64MultiArray, "/grasp_residual", 10)

        cfg_path = os.path.join(
            get_package_share_directory("vision_arm_tasks"), "config", "poses.yaml")
        with open(cfg_path) as handle:
            self.cfg = yaml.safe_load(handle)
        self.home = self.cfg["home"]
        self.place = place
        self.target = tuple(self.cfg["places"][place])
        self.tolerance = self.cfg["place_tolerance"]

        self.results = []
        self.seen = {}
        self.truth = {}
        self.joints = {}
        self.node.create_subscription(
            String, "/task/result", lambda m: self.results.append(m.data), 10)
        self.node.create_subscription(
            Detection3DArray, "/detections", self._on_detections, 10)
        # Ground truth comes over a topic rather than out of `gz model`, which
        # costs about 3.7 s per call on this box. Two calls per attempt was
        # seven seconds of dead time in every trial, and grading an attempt is
        # not worth that.
        self.node.create_subscription(
            Detection3DArray, "/ground_truth", self._on_truth, 10)
        self.node.create_subscription(
            JointState, "/joint_states",
            lambda m: self.joints.update(zip(m.name, m.position)), 10)

    def _on_detections(self, msg):
        for d in msg.detections:
            p = d.results[0].pose.pose.position
            self.seen[d.results[0].hypothesis.class_id] = (p.x, p.y, p.z)

    def _on_truth(self, msg):
        for d in msg.detections:
            p = d.results[0].pose.pose.position
            self.truth[d.results[0].hypothesis.class_id] = (p.x, p.y, p.z)

    def wait_for_truth(self, timeout=10.0):
        """The graded position of the cube, from the unbiased topic."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.colour in self.truth:
                return self.truth[self.colour]
        return None

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def sample_placement(self):
        """A placement inside both the sampling box and the reachable annulus."""
        while True:
            x = self.rng.uniform(*X_RANGE)
            y = self.rng.uniform(*Y_RANGE)
            if self.r_range[0] <= math.hypot(x, y) <= self.r_range[1]:
                return x, y

    def wait_until_seen(self, x, y, tol=0.01, timeout=30.0):
        """Block until the detector agrees the cube is where we just put it.

        Waiting a fixed interval instead is a bug: the ground truth stand in
        ticks every 2 s and shells out to `gz model` three times per tick, so a
        fixed wait straddles a tick and the task server then grasps at the
        PREVIOUS trial's location. Ask, do not assume.
        """
        self.seen.pop(self.colour, None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            got = self.seen.get(self.colour)
            if got and math.hypot(got[0] - x, got[1] - y) < tol:
                return True
        return False

    def reset_arm(self, timeout=15.0):
        """Open the jaws and drive the arm home, bypassing MoveIt.

        Trials have to be independent. A failed cycle leaves the arm stranded
        wherever it gave up, and planning from there fails with
        START_STATE_IN_COLLISION or INVALID_MOTION_PLAN, so the next trial
        records a failure it did not earn. MoveIt cannot do this reset for the
        same reason: it refuses to plan out of a colliding start state. Home is
        collision free by construction and the jaws are opened first, so
        commanding the controller directly is safe here.
        """
        self.jaws.publish(Float64MultiArray(data=[0.0, 0.0]))
        self.pump(0.5)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.home]
        point.time_from_start.sec = 4
        self.traj.publish(JointTrajectory(joint_names=ARM, points=[point]))
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(j in self.joints for j in ARM) and max(
                    abs(self.joints[j] - h) for j, h in zip(ARM, self.home)) < 0.02:
                return True
        return False

    def park_others(self):
        for model, (x, y) in PARK.items():
            if model != self.model:
                set_pose(model, x, y, BENCH_Z)
        self.pump(1.0)

    def new_placement(self, tries=5):
        """Reset the arm and put the cube somewhere new. Returns its position.

        The placement is retried rather than abandoned. The detector shells out
        to `gz model` once per cube per tick and on a loaded box those calls can
        overrun the tick, so a cube that has not been confirmed usually just
        needs asking again. Six of sixty trials were lost this way before the
        retry existed, and a trial lost to instrumentation is worse than a
        trial lost to the robot: it silently shrinks the sample.
        """
        if not self.reset_arm():
            raise RuntimeError("arm did not return home")
        for attempt in range(tries):
            x, y = self.sample_placement()
            # A refused teleport is retried like an unconfirmed one. It is the
            # same gz transport flake, and aborting a sixty trial run over one
            # dropped service call wastes an hour.
            if not set_pose(self.model, x, y, BENCH_Z):
                self.node.get_logger().warn(
                    f"teleport refused, retrying ({attempt + 1}/{tries})")
                continue
            self.pump(1.0)
            if self.wait_until_seen(x, y):
                return x, y
            self.node.get_logger().warn(
                f"detector did not confirm the cube at ({x:+.3f}, {y:+.3f}), "
                f"retrying ({attempt + 1}/{tries})")
        raise RuntimeError(f"no usable placement in {tries} tries")

    def attempt(self, residual=(0.0, 0.0, 0.0)):
        """Run one full pick and place with the given grasp correction.

        Returns a dict with the verdict, whether it succeeded, and how far the
        cube travelled toward the target, which is what grades a near miss.
        """
        self.truth.pop(self.colour, None)
        start = self.wait_for_truth()
        if start is None:
            raise RuntimeError(f"no ground truth for {self.colour}, "
                               f"is fake_detections publishing /ground_truth?")

        self.residual.publish(Float64MultiArray(data=[float(v) for v in residual]))
        self.pump(0.3)
        self.results.clear()
        self.cmd.publish(String(data=f"{self.colour} {self.place}"))

        deadline = time.time() + self.timeout
        verdict = None
        while verdict is None and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            for text in self.results:
                if text.startswith("done:") or text.startswith("failed:"):
                    verdict = text
                    break
        if verdict is None:
            verdict = "failed: timed out waiting for a result"

        self.truth.pop(self.colour, None)
        final = self.wait_for_truth() or start
        tx, ty = self.target
        began = math.hypot(start[0] - tx, start[1] - ty)
        ended = math.hypot(final[0] - tx, final[1] - ty)
        # How far toward the destination the cube actually got, as a fraction.
        # A cube that never moved scores 0, one that was carried scores ~1, and
        # a grasp that nudged it scores in between. This is what separates "the
        # jaws closed on nothing" from "the jaws slipped near the end".
        progress = 0.0 if began < 1e-6 else max(0.0, min(1.0, 1.0 - ended / began))
        return {
            "verdict": verdict,
            "success": verdict.startswith("done:"),
            "planning_failed": "motion failed" in verdict or "no IK" in verdict,
            "start": tuple(start),
            "final": tuple(final),
            "progress": progress,
        }

    def close(self):
        self.node.destroy_node()
