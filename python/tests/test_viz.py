"""Tests of the rendering helpers on synthetic data (no compiled code required)."""

from __future__ import annotations

import json

import matplotlib.pyplot as plt
import numpy as np
import pytest
from PIL import Image

from manipulatorcore_viz import FrameRecorder, apply_style, load_demo, pose_matrix, save_gif
from manipulatorcore_viz.data import ee_position, joint_points, link_segments, subsample
from manipulatorcore_viz.draw2d import draw_frame_2d, draw_planar_arm, draw_velocity_ellipse, setup_planar_axes
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_robot_3d, draw_triad, scene_bounds, setup_axes_3d


def identity_pose(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> list[float]:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T.ravel().tolist()


@pytest.fixture
def frame() -> dict:
    return {
        "t": 0.0,
        "q": [0.0, 0.0],
        "joints": [[0, 0, 0], [0, 0, 0], [1, 0, 0], [1.8, 0, 0]],
        "links": [[0, 0, 0, 1, 0, 0, 0.04, 0], [1, 0, 0, 1.8, 0, 0, 0.04, 1]],
        "ee": identity_pose(1.8),
    }


def test_pose_matrix_and_accessors(frame):
    T = pose_matrix(frame["ee"])
    assert T.shape == (4, 4)
    np.testing.assert_allclose(ee_position(frame), [1.8, 0, 0])
    assert joint_points(frame).shape == (4, 3)
    assert link_segments(frame).shape == (2, 8)


def test_subsample_keeps_endpoints():
    items = list(range(100))
    picked = subsample(items, 7)
    assert len(picked) == 7
    assert picked[0] == 0 and picked[-1] == 99
    assert subsample(items[:3], 10) == [0, 1, 2]


def test_load_demo(tmp_path):
    (tmp_path / "x.json").write_text(json.dumps({"a": 1}), encoding="utf-8")
    assert load_demo(tmp_path, "x") == {"a": 1}


def test_gif_roundtrip(tmp_path):
    frames = [Image.new("RGB", (64, 48), (i * 20, 100, 200)) for i in range(6)]
    path = save_gif(frames, [40] * 6, tmp_path / "a.gif", colors=16)
    with Image.open(path) as im:
        assert im.n_frames == 6
        assert im.size == (64, 48)
    with pytest.raises(ValueError):
        save_gif([], [], tmp_path / "empty.gif")


def test_recorder_hold_extends_last_frame(tmp_path):
    apply_style()
    fig = plt.figure(figsize=(2, 2), dpi=50)
    rec = FrameRecorder()
    rec.capture(fig, 40)
    rec.hold(100)
    plt.close(fig)
    assert rec.durations == [140]
    assert rec.save(tmp_path / "b.gif").exists()


def test_planar_and_spatial_drawing_do_not_fail(frame):
    apply_style()
    fig, ax = plt.subplots(figsize=(3, 3), dpi=50)
    setup_planar_axes(ax, 2.0)
    draw_planar_arm(ax, frame)
    draw_frame_2d(ax, frame["ee"])
    draw_velocity_ellipse(ax, np.array([1.8, 0.0]), [0.5, 0.1, 0.3], scale=0.5)
    plt.close(fig)

    fig = plt.figure(figsize=(3, 3), dpi=50)
    ax3 = fig.add_subplot(projection="3d")
    center, half = scene_bounds([frame])
    setup_axes_3d(ax3, center, half)
    draw_robot_3d(ax3, frame, highlight={1: "#ff0000"})
    draw_triad(ax3, frame["ee"])
    draw_obstacles_3d(
        ax3,
        [
            {"type": "box", "pose": identity_pose(0.5, 0.5, 0.2), "half_extents": [0.1, 0.1, 0.2], "name": "b"},
            {"type": "sphere", "pose": identity_pose(1, 1, 1), "radius": 0.1, "name": "s"},
            {"type": "capsule", "pose": identity_pose(), "a": [0, 0, 0], "b": [0, 0, 1], "radius": 0.1},
            {"type": "halfspace", "pose": identity_pose(), "normal": [0, 0, 1], "offset": 0.0},
        ],
    )
    plt.close(fig)
