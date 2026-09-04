import struct
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from urdf_parser_py.urdf import URDF

PKG_SHARE = Path(__file__).resolve().parents[1]


def _render(**args):
    xacro_path = PKG_SHARE / "urdf" / "vision_arm.urdf.xacro"
    cmd = ["xacro", str(xacro_path)] + [f"{k}:={v}" for k, v in args.items()]
    return URDF.from_xml_string(subprocess.check_output(cmd))


def _stl_vertices(path):
    with open(path, "rb") as handle:
        handle.read(80)
        count = struct.unpack("<I", handle.read(4))[0]
        raw = np.frombuffer(handle.read(count * 50), dtype=np.uint8).reshape(count, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(-1, 3)


def test_xacro_renders_valid_urdf():
    robot = _render()
    assert robot.name == "vision_arm"


def test_all_mesh_files_exist():
    robot = _render()
    for link in robot.links:
        for geom_holder in (link.visual, link.collision):
            mesh = getattr(getattr(geom_holder, "geometry", None), "filename", None)
            if mesh is None:
                continue
            relative = mesh.split(f"vision_arm_description/", 1)[1]
            assert (PKG_SHARE / relative).is_file(), mesh


# Regression for the camera frame that floated 155 mm off the part. The inherited
# pose (xyz="0 -0.09 0.1", copied from the vendor MJCF) put camera_link in free
# space. Nothing downstream can compensate for a camera pose wrong by 155 mm - every
# deprojected detection would be wrong by the same amount - and because the sensor is
# off by default, nothing would have complained.
def test_camera_link_actually_sits_on_the_gripper():
    robot = _render()
    joint = next(j for j in robot.joints if j.name == "camera_joint")
    assert joint.parent == "gripper"

    origin = np.array(joint.origin.xyz)
    verts = _stl_vertices(PKG_SHARE / "meshes" / "gripper" / "gripper.STL")
    gap = float(np.linalg.norm(verts - origin, axis=1).min())

    # The camera hangs off a bracket, so it is legitimately outside the body - but
    # tens of millimetres, not 155. 60 mm brackets the D405 half-depth plus a
    # plausible mount without admitting the old value.
    assert gap < 0.060, f"camera_link is {gap * 1000:.1f} mm from the gripper mesh"


# camera:=true must reach the model. The arg was previously declared after the
# gripper macro was expanded, and no launch file forwarded it at all.
def test_camera_arg_toggles_the_camera_solid():
    def camera_visuals(robot):
        link = next(link for link in robot.links if link.name == "camera_link")
        return [v for v in (link.visuals or [])]

    assert camera_visuals(_render(camera="false")) == []
    assert len(camera_visuals(_render(camera="true"))) == 1


def test_arm_joints_have_finite_limits():
    robot = _render()
    arm_joints = {
        "joint1_base_rotate", "joint2_shoulder", "joint3_elbow",
        "joint4_forearm", "joint5_wrist", "joint6_gripper_mount",
    }
    for joint in robot.joints:
        if joint.name in arm_joints:
            assert joint.type == "revolute", joint.name
            assert joint.limit is not None and joint.limit.lower < joint.limit.upper


# The sensor <pose> was documented at length as "REQUIRED" but never actually
# written into the file, so the camera pointed along camera_link +X (horizontally
# forward) while camera_optical_link and gz_frame_id claimed it pointed down the
# view axis. Deprojection through TF put 41% of the scene below the floor.
# xacro strips <gazebo> before the URDF parser sees it, so assert on the raw XML.
def test_gazebo_camera_sensor_is_rotated_into_the_optical_frame():
    xacro_path = PKG_SHARE / "urdf" / "vision_arm.urdf.xacro"
    xml = subprocess.check_output(["xacro", str(xacro_path), "camera:=true"])
    sensor = ET.fromstring(xml).find(".//sensor[@name='wrist_camera']")
    assert sensor is not None
    pose = [float(v) for v in sensor.find("pose").text.split()]

    # An SDF camera looks along its own +X; we need it looking along camera_link
    # -Z with +Y left = -X, which is rpy = (-pi/2, +pi/2, 0).
    assert pose[:3] == [0.0, 0.0, 0.0]
    assert np.allclose(pose[3:], [-np.pi / 2, np.pi / 2, 0.0], atol=1e-6)
    assert sensor.find("gz_frame_id").text == "camera_optical_link"


def test_every_link_com_lies_inside_its_own_mesh():
    # base_link shipped a CAD inertial origin of (0.0754, 0.0707, 0.1747) while its
    # mesh spans y [-0.2967, -0.0427] and z [0.0157, 0.1379] - a centre of mass 14 cm
    # outside the part, and outside the base's support polygon, so the model tips over
    # as soon as the base carries any real weight.
    robot = _render()
    meshes = {"base_link": "base_link.STL", "link1_base_rotate": "link1_base_rotate.STL",
              "link2_upper_arm": "link2_upper_arm.STL", "link3_elbow": "link3_elbow.STL",
              "link4_forearm": "link4_forearm.STL", "link5_wrist": "link5_wrist.STL",
              "link6_gripper_mount": "link6_gripper_mount.STL"}
    for link in robot.links:
        if link.name not in meshes or link.inertial is None:
            continue
        vertices = _stl_vertices(PKG_SHARE / "meshes" / meshes[link.name])
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        com = np.array(link.inertial.origin.xyz)
        assert np.all(com >= low) and np.all(com <= high), (
            f"{link.name} centre of mass {com.tolist()} is outside its mesh "
            f"bounding box {low.tolist()}..{high.tolist()}")


# The mount height, the workbench top and the cube heights are three numbers in
# two packages that must agree, and nothing at runtime complains if they don't -
# the arm just floats above the bench or sinks into it, and the cubes hover.
# BASE_UNDERSIDE is where base_link's collision box bottom sits in its own frame.
BASE_UNDERSIDE = 0.0157
CUBE_SIZE = 0.05


def _bench_top(robot):
    """World z of the workbench top, from the URDF alone."""
    joint = next(j for j in robot.joints if j.child == "workbench")
    box = next(l for l in robot.links if l.name == "workbench").collision
    assert joint.parent == "world", joint.parent
    return joint.origin.xyz[2] + box.geometry.size[2] / 2


def test_arm_is_welded_to_the_world_at_the_workbench_top():
    robot = _render()
    assert robot.get_root() == "world", robot.get_root()
    weld = next(j for j in robot.joints if j.child == "base_link")
    assert weld.type == "fixed", weld.type
    assert weld.origin.xyz[2] + BASE_UNDERSIDE == _bench_top(robot)


def test_cubes_rest_on_the_workbench_top():
    top = _bench_top(_render())
    world = PKG_SHARE.parents[0] / "vision_arm_gazebo" / "worlds" / "vision_arm_world.sdf"
    models = ET.parse(world).getroot().iter("model")
    cubes = [m for m in models if m.attrib["name"].startswith("cube_")]
    assert len(cubes) == 3
    for cube in cubes:
        z = float(cube.find("pose").text.split()[2])
        assert abs(z - (top + CUBE_SIZE / 2)) < 1e-9, cube.attrib["name"]
