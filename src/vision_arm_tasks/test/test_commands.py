import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "task_server", Path(__file__).resolve().parents[1] / "nodes" / "task_server.py")
task_server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_server)
parse_command = task_server.parse_command

PLACES = {"left": [-0.20, -0.42], "right": [0.20, -0.42]}


def test_named_place():
    assert parse_command("red left", PLACES) == ("red", (-0.20, -0.42))


def test_explicit_coordinates():
    assert parse_command("blue 0.1 -0.42", PLACES) == ("blue", (0.1, -0.42))


def test_case_and_whitespace_are_ignored():
    assert parse_command("  GREEN   Right  ", PLACES) == ("green", (0.20, -0.42))


# Every rejection below reached the arm as a motion in an earlier design, which
# is why they are checked: a typo'd place name must not silently become a move.
@pytest.mark.parametrize("text", [
    "",
    "red",                    # no destination
    "red nowhere",            # unknown place name
    "red 0.1",                # half a coordinate pair
    "red over there please",  # too many words
    "red x y",                # unparseable coordinates
])
def test_bad_commands_raise(text):
    with pytest.raises(ValueError):
        parse_command(text, PLACES)


# --- grasp orientation -------------------------------------------------------

grasp_quaternion = task_server.grasp_quaternion


def rotate(q, v):
    """Apply quaternion q (x, y, z, w) to vector v."""
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2])
def test_no_tilt_is_the_old_top_down_quaternion(yaw):
    """Straight down must be untouched: it is what the vast majority of grasps use."""
    x, y, z, w = grasp_quaternion(yaw, 0.0)
    assert x == pytest.approx(0.0) and y == pytest.approx(0.0)
    assert z == pytest.approx(math.sin(yaw / 2))
    assert w == pytest.approx(math.cos(yaw / 2))


@pytest.mark.parametrize("yaw", [0.0, math.pi / 3, -math.pi / 2])
@pytest.mark.parametrize("pitch", [0.0, math.radians(20), math.radians(50)])
def test_tool_axis_matches_the_standoff_offset(yaw, pitch):
    """plan_pick backs off along the tool axis, so the two must agree exactly.

    If they drift apart the arm approaches along one line and grasps along
    another, which looks like a mysterious grazing failure rather than a bug.
    """
    got = rotate(grasp_quaternion(yaw, pitch), (0.0, 0.0, -1.0))
    want = (-math.sin(pitch) * math.cos(yaw),
            -math.sin(pitch) * math.sin(yaw),
            -math.cos(pitch))
    assert got == pytest.approx(want, abs=1e-9)


@pytest.mark.parametrize("pitch", [0.0, math.radians(20), math.radians(35), math.radians(50)])
def test_tilt_is_the_angle_off_vertical(pitch):
    down = rotate(grasp_quaternion(0.0, pitch), (0.0, 0.0, -1.0))
    assert math.acos(-down[2]) == pytest.approx(pitch, abs=1e-9)


# --- the /grasp_residual trust boundary --------------------------------------

clip_residual = task_server.clip_residual
LIMIT = task_server.RESIDUAL_LIMIT


def test_residual_inside_the_envelope_is_kept():
    assert clip_residual([0.001, -0.002, 0.0]) == (0.001, -0.002, 0.0)


def test_residual_outside_the_envelope_is_clamped():
    assert clip_residual([1.0, -1.0, 0.5]) == (LIMIT, -LIMIT, LIMIT)


# A policy mid-training publishes NaN, and a topic can be published by anything.
# Applying part of such a message would move the grasp somewhere nobody asked for.
@pytest.mark.parametrize("values", [
    [],
    [0.001, 0.002],
    [0.001, 0.002, 0.003, 0.004],
    [float("nan"), 0.0, 0.0],
    [float("inf"), 0.0, 0.0],
])
def test_malformed_residuals_are_dropped(values):
    assert clip_residual(values) is None
