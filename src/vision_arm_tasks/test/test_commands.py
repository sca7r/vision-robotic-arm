import importlib.util
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
