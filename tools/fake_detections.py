"""Publish /detections from Gazebo ground truth, so motion can be tested
without the camera.

The wrist camera costs ~45 s per frame on a box with no GPU, and perception
accuracy is already verified separately (detections land 1.5-2.4 mm from truth).
This stands in for it while testing pick and place, and it is a stub on purpose:
if a grasp works here but not with the real detector, the difference is
perception, not motion.

    python3 tools/fake_detections.py        # needs the sim running
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

MOUNT_Z = 0.7343       # base_link's height in world; keep in sync with the xacro
CUBES = ["red", "green", "blue"]


def world_pose(model, tries=5):
    """`gz model` occasionally answers with nothing while the server is busy."""
    for _ in range(tries):
        out = subprocess.run(["gz", "model", "-m", model, "-p"],
                             capture_output=True, text=True).stdout
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("[") and line.count(" ") == 2:
                return [float(v) for v in line.strip("[]").split()]
        time.sleep(0.3)
    return None


rclpy.init()
node = Node("fake_detections")
pub = node.create_publisher(Detection3DArray, "/detections", 10)


def tick():
    msg = Detection3DArray()
    msg.header.frame_id = "base_link"
    msg.header.stamp = node.get_clock().now().to_msg()
    for colour in CUBES:
        pose = world_pose(f"cube_{colour}")
        if pose is None:
            node.get_logger().warn(f"no pose for cube_{colour}, skipping this tick")
            continue
        x, y, z = pose
        d = Detection3D()
        d.header = msg.header
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = colour
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose.position.x = x
        hypothesis.pose.pose.position.y = y
        hypothesis.pose.pose.position.z = z - MOUNT_Z
        d.results = [hypothesis]
        msg.detections.append(d)
    pub.publish(msg)
    node.get_logger().info(
        "published " + ", ".join(
            f"{d.results[0].hypothesis.class_id}"
            f"({d.results[0].pose.pose.position.x:+.3f},"
            f"{d.results[0].pose.pose.position.y:+.3f},"
            f"{d.results[0].pose.pose.position.z:+.3f})"
            for d in msg.detections))


node.create_timer(2.0, tick)
rclpy.spin(node)
