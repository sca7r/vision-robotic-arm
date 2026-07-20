import subprocess
from pathlib import Path

from urdf_parser_py.urdf import URDF

PKG_SHARE = Path(__file__).resolve().parents[1]


def _render():
    xacro_path = PKG_SHARE / "urdf" / "vision_arm.urdf.xacro"
    xml = subprocess.check_output(["xacro", str(xacro_path)])
    return URDF.from_xml_string(xml)


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
