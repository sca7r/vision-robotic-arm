import importlib.util
from pathlib import Path

import numpy as np

import cv2

SPEC = importlib.util.spec_from_file_location(
    "cube_detector",
    Path(__file__).resolve().parents[1] / "nodes" / "cube_detector.py")
cube_detector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cube_detector)

BANDS = {
    "red": [0, 120, 70, 10, 255, 255, 170, 120, 70, 179, 255, 255],
    "green": [40, 120, 70, 80, 255, 255],
    "blue": [100, 120, 70, 130, 255, 255],
}
# BGR patches at the same saturation the Gazebo cube materials render at.
PATCHES = {"red": (0, 0, 220), "green": (0, 220, 0), "blue": (220, 0, 0)}


def test_detect_blobs_finds_each_cube_at_its_centre():
    image = np.full((240, 320, 3), 90, np.uint8)  # grey table
    expected = {}
    for i, (label, bgr) in enumerate(PATCHES.items()):
        x = 40 + i * 90
        cv2.rectangle(image, (x, 100), (x + 30, 130), bgr, -1)
        expected[label] = (x + 15, 115)

    found = {d[0]: (d[1], d[2]) for d in
             cube_detector.detect_blobs(image, BANDS, min_area=60)}
    assert set(found) == set(expected)
    for label, (u, v) in expected.items():
        assert abs(found[label][0] - u) <= 1
        assert abs(found[label][1] - v) <= 1


def test_detect_blobs_rejects_specks_below_min_area():
    image = np.full((240, 320, 3), 90, np.uint8)
    cv2.rectangle(image, (10, 10), (13, 13), PATCHES["red"], -1)  # 16 px
    assert cube_detector.detect_blobs(image, BANDS, min_area=60) == []


def test_patch_depth_ignores_invalid_samples():
    depth = np.full((10, 10), np.nan, np.float32)
    depth[4:7, 4:7] = 0.30
    depth[5, 5] = 0.0  # Gazebo reports 0 for "no return"
    assert cube_detector.patch_depth(depth, 5, 5, 1) == float(np.float32(0.30))
    assert cube_detector.patch_depth(depth, 0, 0, 1) is None


def test_detect_blobs_rejects_blobs_touching_the_border():
    # Same cube, once well inside the frame and once running off the right edge.
    inside = np.full((240, 320, 3), 90, np.uint8)
    cv2.rectangle(inside, (150, 100), (180, 130), PATCHES["red"], -1)
    assert [d[0] for d in cube_detector.detect_blobs(inside, BANDS, 60)] == ["red"]

    clipped = np.full((240, 320, 3), 90, np.uint8)
    cv2.rectangle(clipped, (300, 100), (330, 130), PATCHES["red"], -1)
    assert cube_detector.detect_blobs(clipped, BANDS, 60) == []
