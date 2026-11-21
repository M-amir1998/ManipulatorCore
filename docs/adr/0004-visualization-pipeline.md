# ADR 0004 — Visualization pipeline

## Status

Accepted.

## Context

Animations are essential to explain manipulation algorithms, but figures produced by a separate
reimplementation (for example kinematics redone in Python) can silently disagree with the library,
and hand-made assets cannot be reproduced. Rendering must work headless on a CPU.

## Decision

* The C++ application `mc_demos` computes everything: kinematics, plans, simulations and the
  world-space geometry of every link in every frame. It exports JSON.
* The Python package `manipulatorcore_viz` only draws that data (matplotlib with the Agg backend) and
  encodes GIFs with Pillow using one global palette per animation.
* `scripts/render/render_all.py` regenerates every asset in `docs/assets` from source. Raw frames,
  videos and intermediate data are never committed.
* Executed motions shown in animations come from the simulator, never from replaying the reference.

## Consequences

* Figures cannot drift from the implementation: changing an algorithm changes its figures when they
  are regenerated.
* Rendering needs no GPU or OpenGL and runs in CI when requested (`RENDER_DOCS=true` in Jenkins).
* mplot3d draws capsules as round-capped thick lines without true 3-D occlusion; this is adequate for
  schematic, scientific figures.
