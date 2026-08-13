"""Find a look pose that frames cubes AND is collision-free AND can reach them.

Three filters, cheapest first:
  1. FK + pinhole projection  (in-process, ~30us)  - does the camera see cubes?
  2. /check_state_validity     (service, ~ms)      - is the arm folded into itself?
  3. /compute_ik on each cube  (service, ~ms)      - can the arm actually grasp it?

Filter 2 is the one that was missing: the previous look pose framed two cubes but
put link4_forearm 62 mm inside the gripper, so MoveIt refused to plan from it.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node
from urdf_parser_py.urdf import URDF as UrdfModel

URDF_PATH = sys.argv[1]
CUBES = {
    "red": np.array([-0.12, -0.425, 0.0407]),
    "green": np.array([0.0, -0.425, 0.0407]),
    "blue": np.array([0.12, -0.425, 0.0407]),
}
W, H = 320, 240
FX = FY = (W / 2.0) / np.tan(1.4661 / 2.0)
CX, CY = W / 2.0, H / 2.0
MARGIN = 18
NEAR, FAR = 0.07, 0.50
TABLE_TOP = 0.0157
JOINTS = ["joint1_base_rotate", "joint2_shoulder", "joint3_elbow",
          "joint4_forearm", "joint5_wrist", "joint6_gripper_mount"]
LIMITS = [(-1.5708, 1.5708), (-0.001, 3.4907), (-0.001, 1.8326),
          (-3.0164, 2.2872), (-1.6750, 2.0378), (-3.1416, 3.1612)]

robot = UrdfModel.from_xml_file(URDF_PATH)
BY_CHILD = {j.child: j for j in robot.joints}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def chain_to(tip):
    out, link = [], tip
    while link != "base_link":
        j = BY_CHILD[link]
        out.append(j)
        link = j.parent
    return out[::-1]


CAM_CHAIN, TCP_CHAIN = chain_to("camera_optical_link"), chain_to("tcp")


def pose(q, chain):
    angles = dict(zip(JOINTS, q))
    R, p = np.eye(3), np.zeros(3)
    for j in chain:
        p = p + R @ np.array(j.origin.xyz if j.origin else [0, 0, 0])
        R = R @ rpy_matrix(*(j.origin.rpy if j.origin else [0, 0, 0]))
        if j.type == "revolute":
            a = np.array(j.axis, float)
            a, t = a / np.linalg.norm(a), angles[j.name]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = R @ (np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K)
    return R, p


def visible(q):
    R, p = pose(q, CAM_CHAIN)
    _, tcp = pose(q, TCP_CHAIN)
    if p[2] < TABLE_TOP + 0.05 or tcp[2] < TABLE_TOP + 0.02:
        return set()
    seen = set()
    for label, cube in CUBES.items():
        local = R.T @ (cube - p)
        z = local[2]
        if not NEAR < z < FAR:
            continue
        u, v = FX * local[0] / z + CX, FY * local[1] / z + CY
        if MARGIN < u < W - MARGIN and MARGIN < v < H - MARGIN:
            seen.add(label)
    return seen


rclpy.init()
node = Node("find_look_pose")
validity = node.create_client(GetStateValidity, "/check_state_validity")
ik = node.create_client(GetPositionIK, "/compute_ik")
assert validity.wait_for_service(timeout_sec=20), "no /check_state_validity"
assert ik.wait_for_service(timeout_sec=20), "no /compute_ik"


def call(client, req):
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    return future.result()


def collision_free(q):
    req = GetStateValidity.Request()
    req.robot_state = RobotState()
    req.robot_state.joint_state.name = JOINTS + ["joint_jaw1", "joint_jaw2"]
    req.robot_state.joint_state.position = list(q) + [0.0, 0.0]
    req.group_name = "arm"
    res = call(validity, req)
    return bool(res and res.valid)


def reachable(point, seed):
    """Can the arm put the TCP at `point`? Tries several wrist orientations."""
    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        for pitch in (0.0, 0.35, 0.7, 1.05):
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = point
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            ps.pose.orientation.x = sp * cy
            ps.pose.orientation.y = sp * sy
            ps.pose.orientation.z = cp * sy
            ps.pose.orientation.w = cp * cy
            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.ik_link_name = "tcp"
            req.ik_request.pose_stamped = ps
            req.ik_request.timeout.sec = 1
            req.ik_request.avoid_collisions = True
            res = call(ik, req)
            if res and res.error_code.val == 1:
                return True, (yaw, pitch)
    return False, None


rng = np.random.default_rng(0)
candidates = []
for _ in range(300000):
    q = np.array([rng.uniform(lo, hi) for lo, hi in LIMITS])
    s = visible(q)
    if len(s) >= 2:
        candidates.append((s, q))
candidates.sort(key=lambda item: -len(item[0]))
print(f"{len(candidates)} configurations frame 2+ cubes; "
      f"{sum(1 for s, _ in candidates if len(s) == 3)} frame all three")

# NOT `for ... else` with a break: the 400-check limit used to break out too,
# which skipped the else clause and made the script exit printing NOTHING when
# every candidate was rejected. That silent exit is why this tool looked like it
# hung in earlier sessions.
checked = 0
found = False
for s, q in candidates[:400]:
    checked += 1
    if not collision_free(q):
        continue
    print(f"\nCOLLISION-FREE look pose after {checked} checks: "
          f"{np.round(q, 4).tolist()}")
    print(f"  frames {sorted(s)}")
    for label in sorted(s):
        ok, orient = reachable(CUBES[label] + np.array([0, 0, 0.10]), q)
        ok2, _ = reachable(CUBES[label], q)
        print(f"  {label:5s} approach reachable: {ok} {orient}   grasp reachable: {ok2}")
    found = True
    break

if not found:
    print(f"\nNO collision-free pose among the {checked} best candidates. "
          f"Check the collision model before blaming the poses: if even the "
          f"all-zero state is invalid, some link pair collides in every "
          f"configuration and belongs in the SRDF as reason=\"Always\".")
