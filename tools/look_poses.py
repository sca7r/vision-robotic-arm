"""Find a minimal set of look poses whose union sees all three cubes.

FK to camera_optical_link, project each cube through the sim camera's
intrinsics, keep poses where a cube is fully inside the frame (the detector
rejects border-touching blobs) and inside the D405 depth window.
"""
import sys

import numpy as np
from urdf_parser_py.urdf import URDF as UrdfModel

URDF = sys.argv[1]
CUBES = {
    "red": np.array([-0.12, -0.425, 0.0407]),
    "green": np.array([0.0, -0.425, 0.0407]),
    "blue": np.array([0.12, -0.425, 0.0407]),
}
W, H = 320, 240
HFOV = 1.4661
FX = FY = (W / 2.0) / np.tan(HFOV / 2.0)
CX, CY = W / 2.0, H / 2.0
MARGIN = 18          # px: blob half-width plus slack, so it never touches a border
NEAR, FAR = 0.07, 0.50
TABLE_TOP = 0.0157

LIMITS = [(-1.5708, 1.5708), (-0.001, 3.4907), (-0.001, 1.8326),
          (-3.0164, 2.2872), (-1.6750, 2.0378), (-3.1416, 3.1612)]

robot = UrdfModel.from_xml_file(URDF)
BY_CHILD = {j.child: j for j in robot.joints}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def chain_to(tip):
    """Joints from base_link out to `tip`, root first."""
    out, link = [], tip
    while link != "base_link":
        j = BY_CHILD[link]
        out.append(j)
        link = j.parent
    return out[::-1]


CAM_CHAIN = chain_to("camera_optical_link")
TCP_CHAIN = chain_to("tcp")
ARM_JOINTS = [j.name for j in CAM_CHAIN if j.type == "revolute"]
assert len(ARM_JOINTS) == 6, ARM_JOINTS


def pose(q, chain):
    """Forward kinematics: rotation and origin of the chain tip in base_link."""
    angles = dict(zip(ARM_JOINTS, q))
    R, p = np.eye(3), np.zeros(3)
    for j in chain:
        xyz = np.array(j.origin.xyz if j.origin else [0, 0, 0])
        rpy = j.origin.rpy if j.origin else [0, 0, 0]
        p = p + R @ xyz
        R = R @ rpy_matrix(*rpy)
        if j.type == "revolute":
            axis = np.array(j.axis if j.axis else [1, 0, 0], float)
            a, t = axis / np.linalg.norm(axis), angles[j.name]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = R @ (np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K)
    return R, p


def visible(q):
    """Which cubes are fully in frame and in range from this configuration."""
    R, p = pose(q, CAM_CHAIN)
    _, tcp = pose(q, TCP_CHAIN)
    if p[2] < TABLE_TOP + 0.05 or tcp[2] < TABLE_TOP + 0.02:
        return set()          # crude: camera and jaws must clear the table
    seen = set()
    for label, cube in CUBES.items():
        local = R.T @ (cube - p)          # into the optical frame: +Z forward
        z = local[2]
        if not (NEAR < z < FAR):
            continue
        u = FX * local[0] / z + CX
        v = FY * local[1] / z + CY
        if MARGIN < u < W - MARGIN and MARGIN < v < H - MARGIN:
            seen.add(label)
    return seen


rng = np.random.default_rng(0)
found = []
for _ in range(400000):
    q = np.array([rng.uniform(lo, hi) for lo, hi in LIMITS])
    s = visible(q)
    if s:
        found.append((s, q))

print(f"{len(found)} configurations see at least one cube")
by_set = {}
for s, q in found:
    by_set.setdefault(frozenset(s), []).append(q)
for s in sorted(by_set, key=lambda k: -len(k)):
    print(f"  sees {sorted(s)}: {len(by_set[s])} configs")

# Greedy cover: fewest poses whose union is all three cubes.
remaining, sweep = set(CUBES), []
while remaining:
    best = max(found, key=lambda item: len(item[0] & remaining))
    gain = best[0] & remaining
    if not gain:
        print("cannot cover", remaining)
        break
    sweep.append(best)
    remaining -= gain
print(f"\nsweep of {len(sweep)} pose(s):")
for s, q in sweep:
    print(f"  {np.round(q, 4).tolist()}   sees {sorted(s)}")

# Of the configurations that see all three, keep the one with the most slack:
# maximise the smallest distance from any cube to a frame border, so small
# errors in the achieved pose (the base is free-floating) do not clip a cube.
alls = [q for s, q in found if len(s) == 3]
if alls:
    def slack(q):
        R, p = pose(q, CAM_CHAIN)
        worst = 1e9
        for cube in CUBES.values():
            local = R.T @ (cube - p)
            u = FX * local[0] / local[2] + CX
            v = FY * local[1] / local[2] + CY
            worst = min(worst, u, W - u, v, H - v)
        return worst
    best = max(alls, key=slack)
    R, p = pose(best, CAM_CHAIN)
    print(f"\nbest all-three pose: {np.round(best, 4).tolist()}")
    print(f"  border slack {slack(best):.1f} px, camera at {np.round(p, 3).tolist()}")
    for label, cube in CUBES.items():
        local = R.T @ (cube - p)
        print(f"  {label:5s} range {local[2]:.3f} m  pixel "
              f"({FX * local[0] / local[2] + CX:.0f}, {FY * local[1] / local[2] + CY:.0f})")
