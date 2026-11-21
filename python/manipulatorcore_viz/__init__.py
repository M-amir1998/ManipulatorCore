"""Rendering utilities for ManipulatorCore demos.

The C++ demo application ``mc_demos`` computes every quantity (kinematics, plans, simulations) and
exports JSON. This package only draws that data with matplotlib (Agg backend, CPU only) and encodes
compact animations, so every figure in the documentation is reproducible from code.
"""

from manipulatorcore_viz.animate import FrameRecorder, save_gif
from manipulatorcore_viz.data import load_demo, pose_matrix
from manipulatorcore_viz.style import PALETTE, apply_style

__all__ = ["PALETTE", "FrameRecorder", "apply_style", "load_demo", "pose_matrix", "save_gif"]
__version__ = "0.1.0"
