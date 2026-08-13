"""Sweep joint1 and check the world model accumulates cubes no single frame sees.

Prints, per look pose: what the camera sees right now (from the debug annotations
we cannot read, so we use a fresh-detection heuristic: the stamp on each
Detection3D) versus everything the world model is holding.
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

POSES = [-1.4, -1.213, -1.0, -0.8, -0.6]
REST = [0.1599, 0.0012, -2.4671, -1.5579, 3.012]

rclpy.init()
node = Node("sweep")
latest = {"array": None}
node.create_subscription(Detection3DArray, "/detections",
                         lambda m: latest.update(array=m), 10)


def pump(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)


def move(j1):
    q = [j1] + REST
    goal = ("{trajectory: {joint_names: [joint1_base_rotate, joint2_shoulder, "
            "joint3_elbow, joint4_forearm, joint5_wrist, joint6_gripper_mount], "
            f"points: [{{positions: {q}, time_from_start: {{sec: 6}}}}]}}}}")
    subprocess.run(["ros2", "action", "send_goal",
                    "/arm_controller/follow_joint_trajectory",
                    "control_msgs/action/FollowJointTrajectory", goal],
                   capture_output=True, text=True)


for j1 in POSES:
    move(j1)
    pump(12.0)
    msg = latest["array"]
    if msg is None:
        print(f"joint1={j1:+.3f}  no /detections yet")
        continue
    now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    fresh, held = [], []
    for d in msg.detections:
        label = d.results[0].hypothesis.class_id
        seen = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
        held.append(label)
        if now - seen < 2.0:
            fresh.append(label)
    print(f"joint1={j1:+.3f}  in frame now: {str(sorted(fresh)):30s} "
          f"world model: {sorted(held)}")

print("\nfinal world model:")
for d in (latest["array"].detections if latest["array"] else []):
    h = d.results[0]
    p = h.pose.pose.position
    age = (latest["array"].header.stamp.sec - d.header.stamp.sec)
    print(f"  {h.hypothesis.class_id:5s} ({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f})  "
          f"last seen {age}s ago")
