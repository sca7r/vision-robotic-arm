"""Does joint1 still move now that the base is welded to the world?

Sends one trajectory that only asks joint1 to turn, then reports what each joint
actually reached and where the base ended up. The failure this exists to catch is
silent: the trajectory action reports SUCCEEDED with joint1 pinned at 0.

    python3 tools/weld_check.py [target_rad]     # needs the sim already up
"""
import subprocess
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# One number = joint1 only; six comma-separated = the whole arm.
ARG = sys.argv[1] if len(sys.argv) > 1 else "-0.9"
GOAL = [float(v) for v in ARG.split(",")]
GOAL = GOAL + [0.0] * (6 - len(GOAL))
TARGET = GOAL[0]
ARM = ["joint1_base_rotate", "joint2_shoulder", "joint3_elbow",
       "joint4_forearm", "joint5_wrist", "joint6_gripper_mount"]

rclpy.init()
node = Node("weld_check")
state = {}
node.create_subscription(
    JointState, "/joint_states",
    lambda m: state.update(dict(zip(m.name, m.position))), 10)

client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
assert client.wait_for_server(timeout_sec=30.0), "no /arm_controller action server"

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ARM
goal.trajectory.points = [JointTrajectoryPoint(
    positions=GOAL,
    time_from_start=Duration(sec=5))]

future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
handle = future.result()
assert handle.accepted, "goal rejected"
result = handle.get_result_async()
rclpy.spin_until_future_complete(node, result, timeout_sec=60.0)
print(f"action error_code {result.result().result.error_code}")

for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.5)

print(f"\ncommanded joint1_base_rotate -> {TARGET:+.4f}")
for name in ARM:
    got = state.get(name, float("nan"))
    want = GOAL[ARM.index(name)]
    print(f"  {name:22s} {got:+.4f}   err {got - want:+.4f}")

pose = subprocess.run(["gz", "model", "-m", "vision_arm", "-p"],
                      capture_output=True, text=True).stdout
print("\nbase pose from gz:\n" + "\n".join(
    line for line in pose.splitlines() if line.strip()))

verdict = abs(state.get(ARM[0], 0.0) - TARGET) < 0.05
print(f"\njoint1 {'TRACKS - weld is good' if verdict else 'IS DEAD - fall back to a heavy base'}")
