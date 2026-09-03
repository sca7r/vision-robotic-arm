"""Publish /detections from Gazebo ground truth, so motion can be tested
without the camera.

The wrist camera costs ~45 s per frame on a box with no GPU, and perception
accuracy is already verified separately (detections land 1.5-2.4 mm from truth).
This stands in for it while testing pick and place, and it is a stub on purpose:
if a grasp works here but not with the real detector, the difference is
perception, not motion.

    python3 tools/fake_detections.py        # needs the sim running

It can also lie, on purpose:

    python3 tools/fake_detections.py --bias-seed 3 --noise 0.002

--bias draws a fixed, unknown offset once and applies it to every detection for
the life of the process, which is what a camera to base calibration error looks
like: not noise, a constant the arm cannot see and cannot average away. --noise
adds the per-frame jitter on top. Together they are the error the learned grasp
residual (src/vision_arm_rl) exists to cancel, and without them there is nothing
for it to learn, because ground truth needs no correcting.

Two topics come out. /detections is what the robot is allowed to believe, and it
carries the bias and the noise. /ground_truth carries the same poses with
neither, and exists so a test harness can grade an attempt without having to
ask Gazebo separately.

Poses arrive by subscribing to Gazebo's pose stream rather than by polling
`gz model`. Polling looked simpler and was a trap: one `gz model` call costs
about 3.7 s on this box, a ROS timer callback blocks the executor while it runs,
and a failed read retried five times took the detector off the air for twenty
seconds at a stretch. That is what made the task server periodically unable to
confirm where a cube had ended up.
"""
import argparse
import random
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

WORLD = "vision_arm_world"
MOUNT_Z = 0.7343       # base_link's height in world; keep in sync with the xacro
CUBES = ["red", "green", "blue"]


def seed_pose(model, tries=5):
    """One-off `gz model` read, to prime a cube that has not moved yet.

    The pose stream only carries entities whose pose changed, so a cube sitting
    still since the world loaded may not appear in it. This runs once per cube
    at startup and then never again, which is the only reason its cost is
    tolerable.
    """
    for _ in range(tries):
        out = subprocess.run(["gz", "model", "-m", model, "-p"],
                             capture_output=True, text=True).stdout
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("[") and line.count(" ") == 2:
                return [float(v) for v in line.strip("[]").split()]
    return None


def stream_poses(poses, world=WORLD):
    """Follow Gazebo's pose stream forever, keeping the latest pose per model.

    One long-lived subprocess whose stdout we read, instead of one subprocess
    per cube per tick. Runs on its own thread so a quiet stream never stalls
    publishing.
    """
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", f"/world/{world}/dynamic_pose/info"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    name, in_position, values = None, False, {}
    for line in proc.stdout:
        line = line.strip()
        if line.startswith('name: "'):
            name = line[len('name: "'):-1]
        elif line == "position {":
            in_position, values = True, {}
        elif in_position and line == "}":
            in_position = False
            if name and len(values) == 3:
                poses[name] = (values["x"], values["y"], values["z"])
        elif in_position and ":" in line:
            key, _, value = line.partition(":")
            # orientation has x/y/z too, which is exactly why this only reads
            # them between "position {" and its closing brace.
            if key in ("x", "y", "z"):
                values[key] = float(value)
    return proc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bias", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="fixed offset in metres, applied to every detection")
    parser.add_argument("--bias-seed", type=int,
                        help="draw the fixed offset at random instead, from this seed")
    parser.add_argument("--bias-mag", type=float, default=0.012,
                        help="half-width of the drawn offset in x and y, metres")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="per-frame Gaussian jitter, standard deviation in metres")
    parser.add_argument("--rate", type=float, default=0.2, help="seconds per tick")
    parser.add_argument("--cubes", nargs="+", default=CUBES,
                        help="which cubes to report on /detections")
    args = parser.parse_args()

    if args.bias:
        bias = tuple(args.bias)
    elif args.bias_seed is not None:
        draw = random.Random(args.bias_seed)
        # z is left alone: the bench height is known, and a camera
        # miscalibration shows up overwhelmingly in the plane the arm reaches
        # across.
        bias = (draw.uniform(-args.bias_mag, args.bias_mag),
                draw.uniform(-args.bias_mag, args.bias_mag), 0.0)
    else:
        bias = (0.0, 0.0, 0.0)
    jitter = random.Random(0)

    rclpy.init()
    node = Node("fake_detections")
    pub = node.create_publisher(Detection3DArray, "/detections", 10)
    truth_pub = node.create_publisher(Detection3DArray, "/ground_truth", 10)
    if any(bias) or args.noise:
        node.get_logger().warn(
            f"LYING: fixed bias ({bias[0] * 1000:+.1f}, {bias[1] * 1000:+.1f}, "
            f"{bias[2] * 1000:+.1f}) mm, noise {args.noise * 1000:.1f} mm")

    poses = {}
    for colour in args.cubes:
        seeded = seed_pose(f"cube_{colour}")
        if seeded:
            poses[f"cube_{colour}"] = tuple(seeded)
    node.get_logger().info(f"seeded {sorted(poses)}, following the pose stream")
    threading.Thread(target=stream_poses, args=(poses,), daemon=True).start()

    def build(colours, corrupt):
        msg = Detection3DArray()
        msg.header.frame_id = "base_link"
        msg.header.stamp = node.get_clock().now().to_msg()
        for colour in colours:
            pose = poses.get(f"cube_{colour}")
            if pose is None:
                continue
            x, y, z = pose[0], pose[1], pose[2] - MOUNT_Z
            if corrupt:
                x, y, z = (
                    v + b + (jitter.gauss(0, args.noise) if args.noise else 0.0)
                    for v, b in zip((x, y, z), bias))
            d = Detection3D()
            d.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = colour
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose.position.x = x
            hypothesis.pose.pose.position.y = y
            hypothesis.pose.pose.position.z = z
            d.results = [hypothesis]
            msg.detections.append(d)
        return msg

    def tick():
        pub.publish(build(args.cubes, corrupt=True))
        truth_pub.publish(build(args.cubes, corrupt=False))

    node.create_timer(args.rate, tick)
    rclpy.spin(node)


if __name__ == "__main__":
    sys.exit(main())
