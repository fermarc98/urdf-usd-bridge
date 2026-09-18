# urdf-usd-bridge

[![CI](https://github.com/fermarc98/urdf-usd-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/fermarc98/urdf-usd-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/urdf-usd-bridge.svg)](https://pypi.org/project/urdf-usd-bridge/)
[![Python](https://img.shields.io/pypi/pyversions/urdf-usd-bridge.svg)](https://pypi.org/project/urdf-usd-bridge/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

Converted URDF robots are kinematically faithful and **dynamically
underdetermined**. They arrive with no drive gains, no armature, sometimes no
inertia tensor, and occasionally a joint welded shut — and each physics backend
fills those holes differently, so the same file behaves three different ways.

`urdf-usd-bridge` is the missing second pass. It reads a converted asset,
reports exactly what physics data is and is not there, and writes what is
missing into **separate USD layers you can diff, mute, or hand-tune**. The
input is never modified. It is not a competing converter.

> **Status: v0.1.0, the first release.** The repairs are measured
> ([`docs/BENCHMARK.md`](docs/BENCHMARK.md)), but the corpus is 12 real robots
> from two sources, Newton still fails on 5 of them, and several gaps are
> untouched. Read [Limits](#limits) before depending on it.

---

## What it does

```bash
urdf-usd-bridge inspect robot.usda          # what physics data is actually here?
urdf-usd-bridge fix robot.usda --out out/   # write the missing dynamics, non-destructively
urdf-usd-bridge convert robot.urdf out/     # both, straight from the URDF
```

- **`inspect`** reports every namespace a quantity could be spelled in,
  separately, and distinguishes *absent* from *schema fallback* from
  *authored*. An authored zero and an unauthored one mean different things.
- **`fix`** derives PD gains, armature and inertia tensors from the robot's own
  geometry, restores joint limits the converter welded shut, and authors them as
  `over` prims in new layers above your untouched original — in each backend's
  own unit convention, because the same physical gain is spelled three ways.
- **`convert`** runs `urdf-usd-converter` and then `fix`, in one command.

`out/robot_stabilized.usda` sublayers your original plus one `Stability*.usda`
per backend. Mute them and you have the input back, byte for byte.

**[`docs/HOW_IT_WORKS.md`](docs/HOW_IT_WORKS.md)** explains what each rule does
and why it is that rule rather than a different one.

## Install

```bash
pip install 'urdf-usd-bridge[core]'       # inspect + fix. Linux, Windows, macOS
pip install 'urdf-usd-bridge[convert]'    # + convert. Linux, Windows only
```

| Extra | Pulls in | For | Platforms |
|---|---|---|---|
| `core` | `usd-core` | `inspect`, `fix` | Linux, Windows, macOS |
| `convert` | `urdf-usd-converter`, `usd-exchange` | `convert` | Linux, Windows |
| `exchange` | `usd-exchange` | `pxr` from the Isaac-parity distribution | Linux, Windows |
| `schemas` | `newton-usd-schemas` | resolving `newton:*` / `mjc:*` attribute types | any |
| `mujoco`, `newton` | `mujoco`, `newton[sim]` | the simulation harness | Linux + NVIDIA GPU |

The package itself depends only on `numpy`: which OpenUSD distribution provides
`pxr` is yours to choose, because a robotics environment usually already has
one.

**macOS:** `[core]` installs and `[convert]` cannot — `usd-exchange` publishes
no macOS wheel at any version, and `urdf-usd-converter` requires it. So you can
`inspect` and `fix` an asset on macOS but not convert one there. v0.1.0 was
verified on Linux only; see [Platforms](#platforms).

## Quickstart

```bash
# 1. What is this asset missing?
urdf-usd-bridge inspect robot.usda

# 2. What would you change? (writes nothing, same code path as a real run)
urdf-usd-bridge fix robot.usda --backend physx --dry-run

# 3. Do it. robot.usda is not touched.
urdf-usd-bridge fix robot.usda --out out/ --backend physx
```

Three runnable examples are in [`examples/`](examples/), graded from "needs
nothing but `usd-core`" to "needs a GPU".

---

## What is measured

A 5-DoF SO-101 arm, converted from its public URDF, **collapses or diverges in
all three backends** as the converter emits it. After `fix` it holds its pose:

| Robot | Backend | Converter output | Repaired |
|---|---|---|---|
| SO-101 arm | PhysX | diverged (3.71 rad) | **0.105 rad** |
| SO-101 arm | MuJoCo | diverged | **0.105 rad** |
| SO-100 arm | PhysX | 2.19 rad | **0.302 rad** |
| Unitree Go2 | PhysX / Newton / MuJoCo | 2.79 rad / diverged / diverged | **0.029 rad** |

Across **14 robots × 3 backends**, hold-pose drift **improved in 35 cells and
got worse in none**, and **19 runs that diverged on the converter's output
converged after repair**. Cross-backend agreement — the actual product claim —
improved from **7.9e-2 rad to 7.6e-5 rad** where three engines could be compared
before and after.

Two things a press release would leave out, and this one does not: **every
baseline diverges in Newton**, all 14, and the repair fixes only 9 of them; and
one tuning constant was swept over a hundredfold range and found to **make no
difference at all**, which is recorded rather than buried.

**→ [`docs/BENCHMARK.md`](docs/BENCHMARK.md)** — the full 84-cell table, the
agreement numbers, every measured default with its basis, the cells where repair
made things worse, and what is not measured.

### Tested on

| Dependency | Versions the measurements were produced on |
|---|---|
| Isaac Sim | 6.1.0-rc.26 |
| Newton | 1.5.0 (Isaac-bundled) and 1.6.0 (PyPI) |
| Warp | 1.16.0 with Newton 1.5.0, 1.17.0 with 1.6.0 |
| MuJoCo / MuJoCo Warp | 3.11.0 and 3.12.0 |
| `usd-core` | 26.8 |
| `urdf-usd-converter` | 0.3.2 (Isaac parity) and 0.3.3 |
| `usd-exchange` | 2.3.0 (Isaac parity) and 3.0.0 |
| `newton-usd-schemas` | 0.4.1 (Isaac parity) and 0.5.0 |
| Python | 3.10, 3.11, 3.12 |

Host: Ubuntu 22.04.5 x86-64, RTX 4090, driver 580.178.04. Every bound in
`pyproject.toml` names a version from this table — nothing is pinned
defensively against a version that was never run.

## Limits

* **Newton diverges on every serial arm and humanoid we tried** (5 of 14
  robots), repaired or not, while PhysX and MuJoCo run the same files. All six
  quadrupeds are fine, so it is not a size effect. Reproduces identically on
  Newton 1.6.0. Filed upstream; not fixable from the asset side.
* **Driven joints overshoot their stops harder** than undriven ones, in 6 of 20
  cells. `limits.compliance` is deliberately still report-only, because the
  value cannot be derived from the asset alone.
* **MuJoCo will not compile an asset containing a `[0,0]` joint.** `fix` refuses
  to guess an ambiguous limit by default; `--force-unlock` is the named way out.
* **Collision filtering, physics materials and scene defaults are not
  implemented**, so contact behaviour is untouched and the drop-suite numbers
  measure backend defaults rather than anything this project does.
* **One tuning constant, `armature_floor`, is unmeasured** and says so in every
  report.

**→ [`docs/ROADMAP.md`](docs/ROADMAP.md)** — each of these with what would
settle it, plus what is deliberately not planned.

### Platforms

| Capability | Linux | Windows | macOS |
|---|---|---|---|
| `inspect`, `fix` | **verified** | installs, untested | installs, untested for 0.1.0 |
| `convert` | **verified** | installs, untested | **cannot install** |
| Newton / MuJoCo simulation | **verified**, GPU | untested | no |
| PhysX simulation, Isaac Sim import | **Linux + NVIDIA GPU** | no | no |

"installs, untested" means exactly that: dependency resolution was checked
against PyPI, nothing was run there. Anything needing a tier you do not have
**skips with a reason** — see [`docs/VERIFY.md`](docs/VERIFY.md).

---

## Found in the process

Building this turned up three defects in the software it builds on. All three
are filed with reproductions and written up in
[`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md).

| Issue | What |
|---|---|
| [IsaacSim#841](https://github.com/isaac-sim/IsaacSim/issues/841) | Isaac Sim 6.1.0 silently drops URDF `<dynamics damping>` and `<dynamics friction>` — the converter writes `newton:damping`, the importer reads `urdf:dynamics:damping`, and the two never meet |
| [IsaacSim#842](https://github.com/isaac-sim/IsaacSim/issues/842) | per-degree `UsdPhysics` drive gains copied into per-radian MJCF slots: a 57.3× error, reachable today through the documented workaround for #841 |
| [newton#4269](https://github.com/newton-physics/newton/issues/4269) | `SolverFeatherstone` diverges on every serial arm and humanoid we tried, while `SolverMuJoCo` runs the identical model |

## Documentation

| | |
|---|---|
| [`docs/HOW_IT_WORKS.md`](docs/HOW_IT_WORKS.md) | what each repair does, and why it is that repair |
| [`docs/BENCHMARK.md`](docs/BENCHMARK.md) | every measured number, including the negative results |
| [`docs/API.md`](docs/API.md) | CLI, Python API, the rules, the output layout, the unit table |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | what is not done, why, and what would settle it |
| [`docs/VERIFY.md`](docs/VERIFY.md) | what can be verified where, and how to run each tier |
| [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md) | the three defects found in dependencies |
| [`CHANGELOG.md`](CHANGELOG.md) | what changed, and which claims are measured |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | setup, house style, and the rule that a repair must earn its place |
| [`docs/history/`](docs/history/) | development records: the prior-art analysis, the design proposals and the phase reports. Provenance, not user documentation |
| [`THIRD_PARTY.md`](THIRD_PARTY.md) | every derivation from an upstream file, and the audit log |

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

## Citing

If you use this in published work, [`CITATION.cff`](CITATION.cff) has the
metadata; GitHub's "Cite this repository" button renders it.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).
