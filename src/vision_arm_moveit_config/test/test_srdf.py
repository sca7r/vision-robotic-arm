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
