import xml.etree.ElementTree as ET
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]


def test_srdf_declares_arm_and_gripper_groups():
    root = ET.parse(PKG_ROOT / "config" / "vision_arm.srdf").getroot()
    groups = {g.attrib["name"] for g in root.findall("group")}
    assert {"arm", "gripper"} <= groups


def test_named_states_cover_home_open_closed():
    root = ET.parse(PKG_ROOT / "config" / "vision_arm.srdf").getroot()
    states = {s.attrib["name"] for s in root.findall("group_state")}
    assert {"home", "open", "closed"} <= states


# IK must solve for `tcp`. Ending the arm group at joint6_gripper_mount made the tip
# link the bare flange, ~140 mm short of where the gripper actually grasps.
def test_arm_group_tip_is_the_grasp_centre_not_the_flange():
    root = ET.parse(PKG_ROOT / "config" / "vision_arm.srdf").getroot()
    arm = next(g for g in root.findall("group") if g.attrib["name"] == "arm")
    chain = arm.find("chain")
    assert chain is not None, "arm group must be a chain so the tip link is explicit"
    assert chain.attrib["tip_link"] == "tcp"
    assert chain.attrib["base_link"] == "base_link"

    ee = next(e for e in root.findall("end_effector") if e.attrib["group"] == "gripper")
    assert ee.attrib["parent_link"] == "tcp"


# jaw2 is a real commanded joint, not a <mimic>. If MoveIt only knows about jaw1, its
# idea of a closed gripper has one jaw still wide open.
def test_both_jaws_are_planned_and_mirrored():
    root = ET.parse(PKG_ROOT / "config" / "vision_arm.srdf").getroot()
    gripper = next(g for g in root.findall("group") if g.attrib["name"] == "gripper")
    assert {j.attrib["name"] for j in gripper.findall("joint")} == {
        "joint_jaw1", "joint_jaw2",
    }

    for name, expected in (("open", (0.0, 0.0)), ("closed", (-0.052, 0.052))):
        state = next(s for s in root.findall("group_state") if s.attrib["name"] == name)
        values = {j.attrib["name"]: float(j.attrib["value"]) for j in state.findall("joint")}
        assert (values["joint_jaw1"], values["joint_jaw2"]) == expected
