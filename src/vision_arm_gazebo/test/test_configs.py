import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]


def test_world_is_valid_xml_with_expected_models():
    tree = ET.parse(PKG_ROOT / "worlds" / "vision_arm_world.sdf")
    names = {m.attrib["name"] for m in tree.getroot().iter("model")}
    assert {"cube_red", "cube_green", "cube_blue"} <= names


def test_controllers_yaml_declares_expected_controllers():
    config = yaml.safe_load((PKG_ROOT / "config" / "controllers.yaml").read_text())
    params = config["controller_manager"]["ros__parameters"]
    assert "arm_controller" in params
    assert "gripper_controller" in params


def test_bridge_yaml_parses():
    config = yaml.safe_load((PKG_ROOT / "config" / "bridge.yaml").read_text())
    assert isinstance(config, list) and len(config) > 0
