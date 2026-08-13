"""Compare /detections against Gazebo ground truth, via the measured base pose.

The base is not at world identity (see anchor_scale in vision_arm.urdf.xacro), so
world cube positions must be transformed into the ACTUAL base_link frame before
they mean anything. Pass the base pose printed by `gz model -m vision_arm -p`.
"""
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

CUBES_WORLD = {
    "red": np.array([-0.12, -0.425, 0.775]),
    "green": np.array([0.0, -0.425, 0.775]),
    "blue": np.array([0.12, -0.425, 0.775]),
}


def base_pose():
    """Read the live base pose out of `gz model`, as (xyz, rpy)."""
    out = subprocess.run(["gz", "model", "-m", "vision_arm", "-p"],
                         capture_output=True, text=True).stdout
    nums = [line.strip().strip("[]").split()
            for line in out.splitlines() if line.strip().startswith("[")]
    xyz, rpy = (np.array([float(v) for v in n]) for n in nums[:2])
    return xyz, rpy


def to_base(point, xyz, rpy):
    r, p, y = rpy
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return (Rz @ Ry @ Rx).T @ (point - xyz)


xyz, rpy = base_pose()
print(f"base at {np.round(xyz, 4).tolist()} rpy {np.round(rpy, 4).tolist()}")
truth = {k: to_base(v, xyz, rpy) for k, v in CUBES_WORLD.items()}

rclpy.init()
node = Node("check_accuracy")
seen = {}


def cb(msg):
    for d in msg.detections:
        h = d.results[0]
        seen[h.hypothesis.class_id] = np.array([h.pose.pose.position.x,
                                                h.pose.pose.position.y,
                                                h.pose.pose.position.z])


node.create_subscription(Detection3DArray, "/detections", cb, 10)
deadline = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
for _ in range(int(deadline)):
    rclpy.spin_once(node, timeout_sec=1.0)
    if len(seen) == 3:
        break

print(f"\n{len(seen)}/3 cubes in the world model")
worst = 0.0
for label, expect in truth.items():
    if label not in seen:
        print(f"  {label:5s} NOT DETECTED   (expected {np.round(expect, 4).tolist()})")
        continue
    err = np.linalg.norm(seen[label] - expect)
    worst = max(worst, err)
    print(f"  {label:5s} got {np.round(seen[label], 4).tolist()}  "
          f"expected {np.round(expect, 4).tolist()}  err {err * 1000:.1f} mm")
print(f"\nworst error {worst * 1000:.1f} mm  "
      f"({'PASS' if len(seen) == 3 and worst < 0.01 else 'FAIL'} vs the 10 mm criterion)")
