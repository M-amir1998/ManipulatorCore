# Regenerating the figures

Every animation and figure in `docs/assets` is produced from code in two steps.

1. **Compute** — the C++ application `mc_demos` runs the library (IK, planning, control in the
   simulator, …) and writes one JSON document per demo, including the world-space geometry of every
   link in every frame.
2. **Draw** — the scripts in `scripts/render` read the JSON and render with matplotlib (Agg, CPU
   only), encoding compact GIFs with a single palette per animation.

```bash
cmake --preset release && cmake --build --preset release
python scripts/render/render_all.py --demos build/release/bin/mc_demos
```

Individual steps:

```bash
build/release/bin/mc_demos list                        # available demos
build/release/bin/mc_demos build/demo_data hero rrt_2r # compute selected demos
python scripts/render/render_hero_demo.py --data build/demo_data --out docs/assets
```

| Asset | Demo | Script |
| --- | --- | --- |
| `hero.gif` | `hero` | `render_hero_demo.py` |
| `fk_ik_2r.gif` | `fk_ik_2r` | `render_fk_ik.py` |
| `singularity.gif` | `singularity` | `render_singularity.py` |
| `rrt_2r.gif`, `planners_2r.png` | `rrt_2r` | `render_rrt_arm.py` |
| `cartesian_paths.gif` | `cartesian_paths` | `render_cartesian.py` |
| `computed_torque.gif` | `computed_torque` | `render_computed_torque.py` |
| `impedance.gif`, `admittance.gif` | `impedance`, `admittance` | `render_impedance.py` |
| `nullspace.gif` | `nullspace` | `render_nullspace.py` |
| `collision.gif` | `collision` | `render_collision.py` |
| `workspace.png` | `workspace` | `render_workspace.py` |
| `narrow_passage.png`, `ik_local_minimum.png`, `cartesian_vs_cspace.gif` | `narrow_passage`, `ik_local_minimum`, `cartesian_vs_cspace` | `render_failures.py` |
| `gravity_hold.gif`, `operational_space.gif` | `gravity_hold`, `operational_space` | `render_gravity_osc.py` |

In Jenkins, rendering only runs when the `RENDER_DOCS` parameter is `true`.
