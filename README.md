# urdf-usd-bridge

A stability layer and cross-backend validator for URDF-derived OpenUSD robot
assets — so the same asset simulates the same way in **PhysX**, **Newton**, and
**MuJoCo** (MJC schemas / MuJoCo Warp).

> **Status: pre-alpha.** Phase 3 ships the first slice of repairs — drive
> gains, armature, inertia and joint limits — as `over` prims in a separate
> layer. **Nothing has been simulated yet**: the tuning constants behind the
> derived gains are documented engineering choices, not measurements, and every
> report says so. See [`docs/PHASE3_REPORT.md`](docs/PHASE3_REPORT.md).

## What this is, and what it is not

URDF→USD conversion is a solved problem. `newton-physics/urdf-usd-converter`
does it well, and Isaac Sim's URDF importer builds a rich multi-backend package
on top of it. Neither, however, authors the data a solver actually needs to be
*stable*: drive gains, armature, valid inertia tensors, collision filtering,
physics materials, limit compliance, or scene defaults. URDF does not carry most
of it, so the converted asset is kinematically faithful and dynamically
underdetermined — and each backend fills the gaps differently.

`urdf-usd-bridge` is the missing second pass. It reads a converted asset,
reports exactly what physics data is and is not there, and writes the missing
data into **separate USD layers** you can diff, mute, or hand-tune. The input is
never modified. It is not a competing converter.

See [`docs/ANALYSIS.md`](docs/ANALYSIS.md) for the full prior-art analysis and
the catalogue of stability gaps this project targets.

## Install

The package codes against the `pxr` (OpenUSD) Python API only, and does not pick
a distribution for you. Install exactly one:

```bash
pip install 'urdf-usd-bridge[core]'      # usd-core — Linux, Windows, macOS
pip install 'urdf-usd-bridge[exchange]'  # usd-exchange — Linux, Windows only
```

> `usd-exchange` publishes no macOS wheels, so `[exchange]` and `[convert]`
> cannot be installed on macOS. Inspection works on every platform.

## Usage

```bash
# What physics data does this asset actually carry?
urdf-usd-bridge inspect robot.usda

# Isaac Sim 6.1.0 packages author no default Physics variant selection.
urdf-usd-bridge inspect robot.usda --variant Physics=physx

# Machine-readable, for CI.
urdf-usd-bridge inspect robot.usda --json -o report.json
```

`inspect` reports, per joint, every namespace a quantity could be spelled in —
`drive:*:physics:damping`, `newton:damping`, `mjc:damping`,
`urdf:dynamics:damping`, `physxJoint:jointFriction`, `newton:armature`,
`mjc:armature` — and never collapses them, because mismatched spellings between
producer and consumer are themselves a live bug class.

```bash
# See what would change, write nothing.
urdf-usd-bridge fix robot.usda --backend physx --dry-run

# Author the repairs into out/, leaving robot.usda untouched.
urdf-usd-bridge fix robot.usda --out out/ --backend physx

# An Isaac Sim 6.x package has a Physics variant set, so all three backends
# can be authored at once, each inside its own variant.
urdf-usd-bridge fix robot.usda --out out/ --backend all

# Convert and repair in one step.
urdf-usd-bridge convert robot.urdf out/ --backend physx
```

`fix` writes a new root layer that sublayers your original plus one
`Stability*.usda` per backend, and a record for every rule it evaluated — prim,
attribute, old value, new value, reason, confidence, and the arithmetic behind
it. `--json` emits the lot. Muting the stability layers restores the input
exactly.

**Backends spell the same gain differently**: `UsdPhysics.DriveAPI` stores
angular gains per *degree*, while MuJoCo actuator gains and Newton PD gains are
per *radian*. `fix` computes once in SI and converts once per backend, and the
test suite asserts the `180/pi` ratio between them. That is why `--backend all`
needs a `Physics` variant set to keep the conventions apart, and refuses without
one.

## Platform support

| Capability | Linux | Windows | macOS |
|---|---|---|---|
| `inspect`, `fix` (pure `pxr`) | yes | yes | yes |
| `convert` (`urdf-usd-converter`) | yes | yes | **no** — `usd-exchange` has no macOS wheel |
| MuJoCo / Newton simulation | yes | partial | untested |
| PhysX simulation, Isaac Sim import | **Linux + NVIDIA GPU only** | no | no |

Anything needing an Isaac Sim / Kit / PhysX runtime is isolated under
`scripts/` and marked `isaacsim` / `gpu` in the test suite. See
[`docs/VERIFY.md`](docs/VERIFY.md).

## Tested versions

Pinned and documented; see [`docs/VERIFY.md`](docs/VERIFY.md) and
[`docs/PHASE2_REPORT.md`](docs/PHASE2_REPORT.md) for what has actually been run.

| Component | Version |
|---|---|
| `urdf-usd-converter` | 0.3.2 (Isaac Sim 6.1.0 parity), 0.3.3 (upstream) |
| `usd-exchange` | 2.3.0 (Isaac parity column), 3.0.0 (resolved) |
| `usd-core` | ≥ 25.5 (tested: 26.8) |
| `newton-usd-schemas` | 0.4.1 (Isaac parity column), 0.5.0 (resolved) |
| `newton` | 1.5.0 |
| `mujoco` / `mujoco-warp` | 3.11.0 |
| Isaac Sim | 6.1.0 (tested: 6.1.0-rc.26), 6.0.1 (comparison, not installed), 5.1.0 (historical) |

`scripts/run_converter_matrix.py` runs three columns: `0.3.2`, `0.3.3`, and
`0.3.2-isaac`, which pins the exact `usd-exchange` and `newton-usd-schemas`
versions Isaac Sim 6.1.0 ships so that "what the converter does" and "what an
Isaac user gets" are separate measurements.

## Acknowledgements

This project builds on, and is designed to interoperate with:

- [`newton-physics/urdf-usd-converter`](https://github.com/newton-physics/urdf-usd-converter)
  (Apache-2.0) — the URDF→USD data converter whose output we inspect and extend.
- [`newton-physics/mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter)
  (Apache-2.0) — the reference for MJC schema authoring.
- [NVIDIA Isaac Sim](https://github.com/isaac-sim/IsaacSim) (Apache-2.0) — its
  URDF importer and asset transformer define the multi-backend package layout we
  stay compatible with.

No upstream code has been copied. Where a formula or an interface encoding was
taken from an upstream file, it is recorded in
[`THIRD_PARTY.md`](THIRD_PARTY.md) with what was taken and what differs; any
code that is ever adapted keeps its original copyright header.

Two defects found in Isaac Sim 6.1.0 while building this are written up, with
reproductions, in [`docs/UPSTREAM_ISSUES.md`](docs/UPSTREAM_ISSUES.md).

**This project is not affiliated with, endorsed by, or sponsored by NVIDIA,
the Newton project, Google DeepMind, or Disney.** Names are used only to
identify the software this project interoperates with.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE).
