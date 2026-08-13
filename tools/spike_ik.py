"""Spike: can we get IK at the TCP for a top-down grasp above a detected cube?

tcp inherits the gripper frame (tcp_joint rpy=0): -Z points at the workspace and
the jaws close along +-Y. So "jaws pointing straight down" is identity rotation,
and yaw about base Z just spins which way the jaws close.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

TARGET = sys.argv[1] if len(sys.argv) > 1 else "green"
STANDOFF = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10

rclpy.init()
node = Node("spike_ik")
world = {}
node.create_subscription(
    Detection3DArray, "/detections",
    lambda m: world.update({d.results[0].hypothesis.class_id: d.results[0].pose.pose.position
                            for d in m.detections}), 10)

for _ in range(120):
    rclpy.spin_once(node, timeout_sec=0.5)
    if TARGET in world:
        break
if TARGET not in world:
    print(f"never saw a {TARGET} cube; world model has {sorted(world)}")
    sys.exit(1)

p = world[TARGET]
print(f"{TARGET} at ({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f}) in base_link")

client = node.create_client(GetPositionIK, "/compute_ik")
if not client.wait_for_service(timeout_sec=15.0):
    print("no /compute_ik service")
    sys.exit(1)


def try_ik(z_offset, yaw, label):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = p.x
    pose.pose.position.y = p.y
    pose.pose.position.z = p.z + z_offset
    # Rotation about base Z only: keeps tcp -Z pointing straight down.
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    req = GetPositionIK.Request()
    req.ik_request.group_name = "arm"
    req.ik_request.ik_link_name = "tcp"
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout.sec = 2
    req.ik_request.avoid_collisions = True
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    res = future.result()
    code = res.error_code.val if res else None
    ok = code == 1
    joints = ""
    if ok:
        names = res.solution.joint_state.name
        pos = res.solution.joint_state.position
        joints = "  " + str([round(v, 3) for n, v in zip(names, pos)
                             if n.startswith("joint") and "jaw" not in n])
    print(f"  {label:28s} yaw {math.degrees(yaw):+6.1f} deg -> "
          f"{'OK' if ok else f'fail (code {code})'}{joints}")
    return ok


print(f"\napproach pose, {STANDOFF * 100:.0f} cm above the cube:")
approach_ok = [try_ik(STANDOFF, y, "approach") for y in
               (0.0, math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2, math.pi)]
print("\ngrasp pose, at the cube centre:")
grasp_ok = [try_ik(0.0, y, "grasp") for y in
            (0.0, math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2, math.pi)]
print(f"\napproach solvable at {sum(approach_ok)}/6 yaws, "
      f"grasp solvable at {sum(grasp_ok)}/6 yaws")
