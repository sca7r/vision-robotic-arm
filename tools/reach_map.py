"""Offline reach map: where can the arm do a top-down grasp, per mount height?

No ROS, no sim. FK only (borrowed from look_poses.py). Answers the one question
that decides the new world layout: if we bolt base_link to a work surface, how
high above that surface should the base sit for the cubes to be graspable?

    python3 tools/reach_map.py /tmp/arm.urdf

Geometry, all in base_link frame (z up):
  base_link's collision box underside is at z=+0.0157, so bolting it to a
  surface puts that surface at z = 0.0157 - riser, where `riser` is the height
  of any block between the surface and the base plate.
  A 3 cm cube resting on the surface has its centre 0.015 above it.
"""
import sys

import numpy as np
from urdf_parser_py.urdf import URDF as UrdfModel

URDF = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 400000
CUBE = float(sys.argv[3]) if len(sys.argv) > 3 else 0.03

# Negative "riser" = the surface is ABOVE the base plate, i.e. today's layout,
# where the arm stands on the floor and reaches up onto a 0.28 m table.
RISERS = [-0.2643, 0.0, 0.10, 0.20, 0.28]
BASE_UNDERSIDE = 0.0157

TILT_TOL = np.deg2rad(15)          # how far off straight-down a grasp may be
Z_TOL = 0.005                      # +-5 mm on the grasp height
CLEAR = 0.005                      # links must clear the surface by this much
NEAR_BASE = 0.15                   # ponytail: inside this radius the riser is
                                   # under the arm, so surface clearance is moot

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
    out, link = [], tip
    while link != "base_link":
        j = BY_CHILD[link]
        out.append(j)
        link = j.parent
    return out[::-1]


TCP_CHAIN = chain_to("tcp")
ARM_JOINTS = [j.name for j in TCP_CHAIN if j.type == "revolute"]
assert len(ARM_JOINTS) == 6, ARM_JOINTS


def pose(q):
    """TCP rotation, TCP origin, and every intermediate link origin."""
    angles = dict(zip(ARM_JOINTS, q))
    R, p, trail = np.eye(3), np.zeros(3), []
    for j in TCP_CHAIN:
        xyz = np.array(j.origin.xyz if j.origin else [0, 0, 0])
        rpy = j.origin.rpy if j.origin else [0, 0, 0]
        p = p + R @ xyz
        R = R @ rpy_matrix(*rpy)
        if j.type == "revolute":
            axis = np.array(j.axis if j.axis else [1, 0, 0], float)
            a, t = axis / np.linalg.norm(axis), angles[j.name]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = R @ (np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K)
        trail.append(p.copy())
    return R, p, np.array(trail)


# tcp's -Z points at the workspace (see spike_ik.py), so a straight-down grasp
# is +Z up: R[:,2] . [0,0,1] == 1.
rng = np.random.default_rng(0)
samples = []
for _ in range(N):
    q = np.array([rng.uniform(lo, hi) for lo, hi in LIMITS])
    R, p, trail = pose(q)
    if R[2, 2] < np.cos(TILT_TOL):
        continue
    samples.append((q, p, trail))

print(f"{len(samples)} of {N} samples are within {np.rad2deg(TILT_TOL):.0f} deg of straight down\n")

for riser in RISERS:
    surface = BASE_UNDERSIDE - riser
    z_grasp = surface + CUBE / 2
    hits = []
    for q, p, trail in samples:
        if abs(p[2] - z_grasp) > Z_TOL:
            continue
        far = np.linalg.norm(trail[:, :2], axis=1) > NEAR_BASE
        if np.any(trail[far, 2] < surface + CLEAR):
            continue
        hits.append(p)
    print(f"riser {riser * 100:5.1f} cm   surface z={surface:+.4f}  grasp z={z_grasp:+.4f}")
    if not hits:
        print("    NO top-down grasp anywhere on this surface\n")
        continue
    hits = np.array(hits)
    r = np.linalg.norm(hits[:, :2], axis=1)
    print(f"    {len(hits):5d} poses   radius {r.min():.3f}-{r.max():.3f} m"
          f"   x {hits[:, 0].min():+.3f}..{hits[:, 0].max():+.3f}"
          f"   y {hits[:, 1].min():+.3f}..{hits[:, 1].max():+.3f}")
    # Coarse 5 cm occupancy grid, to show the shape of the usable patch.
    cell = 0.05
    grid = {(int(np.floor(x / cell)), int(np.floor(y / cell))) for x, y in hits[:, :2]}
    print(f"    usable area ~{len(grid) * cell * cell * 1e4:.0f} cm2"
          f" ({len(grid)} cells of 5x5 cm)")
    if riser == 0.0:
        # Where to actually put objects: the best-covered cells, which are the
        # ones with the most different arm configurations that can reach them.
        counts = {}
        for x, y in hits[:, :2]:
            counts[(int(np.floor(x / cell)), int(np.floor(y / cell)))] = \
                counts.get((int(np.floor(x / cell)), int(np.floor(y / cell))), 0) + 1
        best = sorted(counts.items(), key=lambda kv: -kv[1])[:14]
        print("    best-covered cells (centre x, y, radius, n):")
        for (cx, cy), n in best:
            x, y = (cx + 0.5) * cell, (cy + 0.5) * cell
            print(f"      {x:+.3f} {y:+.3f}  r={np.hypot(x, y):.3f}  n={n}")
    print()
