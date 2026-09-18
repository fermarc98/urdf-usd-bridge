# urdf-usd-bridge

Converted URDF robots are kinematically faithful and **dynamically
underdetermined**. They arrive with no drive gains, no armature, sometimes no
inertia tensor, and occasionally a joint welded shut — and each physics backend
fills those holes differently, so the same file behaves three different ways.

`urdf-usd-bridge` is the missing second pass. It reads a converted asset,
reports exactly what physics data is and is not there, and writes what is
missing into **separate USD layers you can diff, mute, or hand-tune**. The
input is never modified. It is not a competing converter.

> **Status: pre-alpha.** The repairs work and are measured (§Measured), but the
> corpus is 12 real robots from two sources, Newton still fails on 5 of them,
> and several gaps from the analysis are untouched. Read §Honest limits before
> depending on it.

---

## What it does, measured

A 5-DoF SO-101 arm, converted from its public URDF, **collapses or diverges in
all three backends** as the converter emits it. After `fix` it holds its pose:

| Robot | Backend | Converter output | Repaired |
|---|---|---|---|
| SO-101 arm | PhysX | diverged (3.71 rad) | **0.105 rad** |
| SO-101 arm | MuJoCo | diverged | **0.105 rad** |
| SO-100 arm | PhysX | 2.19 rad | **0.302 rad** |
| Unitree Go2 | PhysX / Newton / MuJoCo | 2.79 rad / diverged / diverged | **0.029 rad** |

Across the 84-cell benchmark below, hold-pose drift **improved in 35 cells and
got worse in none**, and **19 runs that diverged on the converter's output
converged after repair**.

And cross-backend agreement — the actual product claim — improved by three
orders of magnitude where it could be computed: three independent engines ended
up **7.9e-2 rad apart** on the converter's output and **7.6e-5 rad apart** after
repair.

Full method, metric definitions and the negative results:
[`docs/PHASE4_REPORT.md`](docs/PHASE4_REPORT.md).

### Benchmark: 14 robots, 3 backends, hold-pose drift in radians

`DIV` = diverged (NaN or runaway). Lower is better; the two columns per backend
are the converter's output and the repaired asset.

| Robot | DoF | PhysX | | Newton | | MuJoCo | |
|---|---|---|---|---|---|---|---|
| | | base | **rep** | base | **rep** | base | **rep** |
| SO-101 arm | 6 | DIV | **1.05e-1** | DIV | DIV | DIV | **1.04e-1** |
| SO-100 arm | 6 | 2.19e+0 | **3.02e-1** | DIV | DIV | DIV | **3.02e-1** |
| Unitree Z1 arm | 6 | DIV | **1.19e-1** | DIV | DIV | 1.53e+0 | **1.16e-1** |
| Unitree A1 | 12 | 1.51e+0 | **2.48e-2** | DIV | **2.38e-2** | 1.79e+0 | **2.37e-2** |
| Unitree Go1 | 12 | 2.08e+0 | **8.28e-1** | DIV | **2.05e-2** | DIV | **2.05e-2** |
| Unitree Go2 | 12 | 2.79e+0 | **2.90e-2** | DIV | **2.85e-2** | DIV | **2.85e-2** |
| Unitree AlienGo | 12 | 9.36e-1 | 8.36e-1 | DIV | **1.76e-2** | DIV | **1.76e-2** |
| Unitree B2 | 12 | 2.35e+0 | **1.49e-2** | DIV | **1.47e-2** | 2.15e+0 | **1.47e-2** |
| Unitree Laikago | 12 | 1.36e+0 | **4.82e-1** | DIV | **1.61e-2** | 2.00e+0 | **1.61e-2** |
| Unitree H1 | 19 | DIV | **6.42e-2** | DIV | DIV | DIV | **6.44e-2** |
| Unitree G1 | 23 | DIV | **6.51e-1** | DIV | DIV | DIV | **6.60e-1** |
| fixture (a) | 2 | 1.34e-1 | **4.00e-2** | 1.35e-1 | **4.01e-2** | 1.36e-1 | **4.00e-2** |
| fixture (b) | 3 | 5.00e-2 | **3.02e-2** | DIV | **3.02e-2** | 9.57e-2 | **3.02e-2** |
| fixture (d) | 2 | 5.00e-2 | 5.00e-2 | DIV | 2.52e-1 | 5.02e-2 | 5.03e-2 |

Over these 84 cells: **`pose_drift_max` improved in 35 and got worse in none**,
and **19 runs that diverged converged after repair**.

Two things this table shows that a press release would not:

* **Every single baseline diverges in Newton**, all 14 of them. The repair
  fixes 9; the 5 it does not are all serial arms and humanoids, which is
  [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md) issue 3.
* **PhysX on AlienGo barely moves** (0.94 → 0.84 rad) and **fixture (d) does
  not improve at all**. Both are real; neither is hidden.
* **Two robots are missing from the table** because a guard refused to measure
  them: the dexterous hand has no joint gravity can load at any pose, and
  fixture (c) has a joint deliberately left welded. Refusing beats reporting a
  zero that means nothing.

Reproduce: `<isaac>/python.sh scripts/run_sim_matrix.py --out bench --suite hold_pose --exploratory`

---

## Quickstart

```bash
pip install 'urdf-usd-bridge[core]'       # usd-core: Linux, Windows, macOS
pip install 'urdf-usd-bridge[convert]'    # + the converter: Linux, Windows only
```

```bash
# 1. What is this asset missing?
urdf-usd-bridge inspect robot.usda

# 2. What would you change? (writes nothing)
urdf-usd-bridge fix robot.usda --backend physx --dry-run

# 3. Do it. robot.usda is not touched.
urdf-usd-bridge fix robot.usda --out out/ --backend physx

# Or convert and repair in one step, from the URDF.
urdf-usd-bridge convert robot.urdf out/
```

`out/robot_stabilized.usda` sublayers your original plus one `Stability*.usda`
per backend. Mute them and you have the input back, byte for byte.

Three runnable examples are in [`examples/`](examples/), from "needs nothing but
`usd-core`" to "needs a GPU".

---

## The part that bites everyone

The same physical gain is spelled three ways, in two different angle
conventions:

| Backend | Attribute | Convention |
|---|---|---|
| PhysX | `drive:angular:physics:stiffness` | per **degree** |
| MuJoCo | `mjc:gainPrm` | per **radian** |
| Newton | `UsdPhysics.DriveAPI` (it converts internally) | per **degree** |

Get it wrong and you are off by 57.3× — which is a live bug in shipping
software, not a hypothetical: see [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md)
issue 2, reproduced on Isaac Sim 6.1.0. This project computes once in SI,
converts once per backend, and has a test that fails on a 57.3× error in either
direction.

That is also why `--backend all` needs a `Physics` variant set to keep the
conventions in separate layers, and refuses without one.

---

## Measured, unmeasured, and on what

**Tested on:** Ubuntu 22.04.5 x86-64, RTX 4090 (driver 580.178.04), Isaac Sim
**6.1.0-rc.26**, Newton **1.5.0**, Warp **1.16.0**, MuJoCo **3.11.0**,
`usd-core` **26.8**, `urdf-usd-converter` **0.3.2** and **0.3.3**,
`usd-exchange` **2.3.0** (Isaac parity) and **3.0.0**, `newton-usd-schemas`
**0.4.1** and **0.5.0**, Python 3.10–3.12.

### Measured

| Claim | Evidence |
|---|---|
| Drive gains stop a real arm collapsing | 35 cells improved, 0 worse; 19 divergences prevented |
| Repair improves cross-backend agreement | 7.9e-2 → 7.6e-5 rad on fixture (a) |
| `damping_ratio = 1.0` is the right default | minimises step settle time (0.175 s) with zero overshoot |
| Stable gains need `f_n ≤ control_rate/12` cross-backend, `/6` for PhysX or MuJoCo alone | dt sweep, exact across four frequencies × three rates |
| `armature_fraction` makes **no difference** to stability | swept 0 → 1.0, a hundredfold range: no change to the divergence threshold |
| The converter always emits `convexHull` for mesh colliders | fixture (d), three converter columns |
| Isaac Sim 6.1.0 drops `<dynamics damping>` | reproduced, with Isaac's own warning as evidence |

### Unmeasured

* **`armature_floor`** (1e-4). The low-inertia case it exists for was not
  exercised. Every report says so; it is not dressed up as a measurement.
* **Everything about contact.** Collision approximation and physics materials
  are untouched, and the drop-test numbers are dominated by them.
* **Anything outside the 12-robot corpus**, which is three arms, six
  quadrupeds, two humanoids and a hand — all from two sources.

---

## Honest limits

* **Newton diverges on every serial arm and humanoid we tried** (5 of 14
  robots), repaired or not, while PhysX and MuJoCo run the same files. All six
  quadrupeds are fine, so it is not a size effect. Isolated as far as black-box testing allows — it is not
  our gains, not the timestep, and not the mass ratio — and written up as
  [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md) issue 3.
* **Driven joints overshoot their stops harder** than undriven ones, in 6 of 20
  cells. `limits.compliance` would address it and is deliberately still
  report-only, because the value cannot be derived from the asset without the
  effective inertia at `qpos0`.
* **MuJoCo will not compile an asset containing a `[0,0]` joint.** We refuse to
  guess an ambiguous limit by default; `--force-unlock` is the named way out,
  and it says in the record that the range is a guess.
* **G4 (collision filtering), G5 (physics materials) and G6 (scene defaults)
  are not implemented.** See [`docs/ANALYSIS.md`](docs/ANALYSIS.md).
* **The drop and limit suites need a GPU**, and so does every number in the
  tables above. The metric definitions themselves are pure and tested in CI.

---

## Platform support

| Capability | Linux | Windows | macOS |
|---|---|---|---|
| `inspect`, `fix` (pure `pxr`) | yes | yes | yes |
| `convert` (`urdf-usd-converter`) | yes | yes | **no** — `usd-exchange` has no macOS wheel |
| Newton / MuJoCo simulation | yes, GPU | untested | no |
| PhysX simulation, Isaac Sim import | **Linux + NVIDIA GPU** | no | no |

Anything needing a tier you do not have **skips with a reason**. See
[`docs/VERIFY.md`](docs/VERIFY.md).

---

## Documentation

| | |
|---|---|
| [`docs/API.md`](docs/API.md) | CLI, Python API, the rules, the output layout, the unit table |
| [`docs/ANALYSIS.md`](docs/ANALYSIS.md) | prior-art analysis and the catalogue of gaps (G1–G10) |
| [`docs/VERIFY.md`](docs/VERIFY.md) | what can be verified where, and how to run each tier |
| [`docs/PHASE3_REPORT.md`](docs/PHASE3_REPORT.md) | what each repair does, and how it is authored |
| [`docs/PHASE4_REPORT.md`](docs/PHASE4_REPORT.md) | the measurements, including the ones that found nothing |
| [`docs/PHASE5_REPORT.md`](docs/PHASE5_REPORT.md) | the 12-robot corpus, and what the Newton failure is *not* |
| [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md) | three reproducible defects found in dependencies |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | setup, house style, and the rule that a repair must earn its place |

---

## Acknowledgements

Builds on, and interoperates with:

- [`newton-physics/urdf-usd-converter`](https://github.com/newton-physics/urdf-usd-converter)
  (Apache-2.0) — the URDF→USD converter whose output we inspect and extend.
- [`newton-physics/mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter)
  (Apache-2.0) — the reference for MJC schema authoring, and the source of the
  clearest statement of the per-degree/per-radian split.
- [NVIDIA Isaac Sim](https://github.com/isaac-sim/IsaacSim) (Apache-2.0) — its
  URDF importer and asset transformer define the multi-backend package layout
  we stay compatible with.
- [`newton-physics/newton`](https://github.com/newton-physics/newton) (Apache-2.0)
  and [MuJoCo](https://github.com/google-deepmind/mujoco) (Apache-2.0) — the
  solvers the harness measures against.

Corpus robots are fetched at run time and never vendored:
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0) and
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) (BSD-3-Clause).

No upstream code has been copied. Where a formula or interface encoding was
taken from an upstream file it is recorded in
[`THIRD_PARTY.md`](THIRD_PARTY.md) with what was taken and what differs.

**This project is not affiliated with, endorsed by, or sponsored by NVIDIA, the
Newton project, Google DeepMind, Unitree, or Disney.** Names identify the
software this project interoperates with, nothing more.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE).
