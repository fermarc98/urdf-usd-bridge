# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Because the point of this project is measurement, entries distinguish what was
**measured** from what was reasoned about, and record results that came out
negative alongside the ones that did not.

## [Unreleased]

Nothing yet. See [`docs/ROADMAP.md`](docs/ROADMAP.md) for what is planned.

## [0.1.0] — 2026-09-18

First release. Inspection, repair and a cross-backend measurement harness.

### Added

**`inspect`** — read-only reporting on what physics data a converted asset
actually carries. Reports every namespace a quantity could be spelled in,
separately, and distinguishes *absent* from *schema fallback* from *authored*.
Pure `pxr`, so it should run wherever OpenUSD does; for this release that was
verified on Linux only.

**`fix`** — the stability layer. Authors repairs as `over` prims in new layers
above an untouched input:

- `drives.derive-gains` — PD gains from each joint's equivalent inertia and a
  target natural frequency, in each backend's own convention.
- `drives.mirror-passive` — the URDF's `<dynamics damping/friction>` translated
  into each backend's spelling, with the unit conversion each one needs.
- `armature.default` — `max(α·I_eq, β·I_eq_max)` in all three namespaces.
- `inertia.derive-from-geometry` — a tensor from collision geometry when a body
  has mass and no usable inertia.
- `inertia.principal-axes-identity` — repairs the zero-quaternion defect that
  `urdf-usd-converter` 0.3.2 ships and Isaac Sim 6.1.0 therefore inherits.
- `inertia.make-physical` — clamps negative moments and enforces
  `I1 + I2 ≥ I3`, always by raising inertia.
- `limits.restore-missing` — unlocks joints the converter welded at `[0,0]`
  because the URDF omitted `<limit>`, distinguished from a deliberate `[0,0]`
  by whether any limit evidence survives in the asset.
- `limits.report-ambiguous`, `limits.compliance`, `inertia.mass-floor`,
  `drives.no-drive-for-fixed` — report rather than guess.

**`convert`** — `urdf-usd-converter` followed by `fix`, in one command.

**A cross-backend simulation harness** (`scripts/run_sim_matrix.py`) running
PhysX, Newton and MuJoCo Warp from one interpreter, with hold-pose, drop and
limit-sweep suites, a declared scene contract, and two guards that refuse to
report a metric that would be meaningless.

**Documentation**: [`docs/API.md`](docs/API.md),
[`docs/ANALYSIS.md`](docs/ANALYSIS.md), [`docs/VERIFY.md`](docs/VERIFY.md),
per-phase reports, [`CONTRIBUTING.md`](CONTRIBUTING.md), and three runnable
[`examples/`](examples/).

### Measured

On 14 robots × 3 backends (84 cells), hold-pose drift **improved in 35 cells
and got worse in none**, and **19 runs that diverged on the converter's output
converged after repair**. Cross-backend agreement on fixture (a) improved from
7.9e-2 rad to **7.6e-5 rad**. Full method in
[`docs/PHASE4_REPORT.md`](docs/PHASE4_REPORT.md).

Tuning constants were measured rather than assumed:

- `target_frequency` is **derived** from the backend selection:
  `control_rate/6` for PhysX or MuJoCo alone, `control_rate/12` for Newton or
  any multi-backend asset. Phase 3 assumed `control_rate/4`, which the dt sweep
  contradicted — 10 Hz at a 60 Hz rate diverges in Newton.
- `damping_ratio = 1.0` confirmed: minimises step settle time (0.175 s) with
  zero overshoot.

### Measured and found to do nothing

- **`armature_fraction` has no measurable effect on the stability margin.**
  Swept over 0, 0.01, 0.1 and 1.0 — a hundredfold range — and it changed
  neither the divergence threshold nor the timestep at which it occurs. The
  rule ships at its original value with that recorded as its provenance.

### Known not measured

- **`armature_floor`** — the low-inertia case it exists for is not represented
  in the corpus. A dexterous hand was added specifically to test it and the
  gravity guard refused the robot, because no pose loads any of its joints.
- Everything about contact: collision approximation and physics materials are
  untouched, and the drop-suite numbers are dominated by them.

### Found upstream

Three defects in dependencies, all filed with reproductions:

- [IsaacSim#841](https://github.com/isaac-sim/IsaacSim/issues/841) — Isaac Sim
  6.1.0 silently drops URDF `<dynamics damping>` and `<dynamics friction>`.
- [IsaacSim#842](https://github.com/isaac-sim/IsaacSim/issues/842) — per-degree
  drive gains copied into per-radian MJCF slots: a 57.3× error.
- [newton#4269](https://github.com/newton-physics/newton/issues/4269) —
  `SolverFeatherstone` diverges where `SolverMuJoCo` does not. Reproduces
  identically on Newton 1.5.0 and 1.6.0.

### Verified on

Ubuntu 22.04.5 x86-64 with an RTX 4090 (driver 580.178.04), Python 3.10–3.12,
`usd-core` 26.8, `urdf-usd-converter` 0.3.2 and 0.3.3, `usd-exchange` 2.3.0 and
3.0.0, `newton-usd-schemas` 0.4.1 and 0.5.0, Isaac Sim 6.1.0-rc.26, Newton
1.5.0 and 1.6.0, MuJoCo 3.11.0 and 3.12.0.

**Not verified on macOS or Windows.** The wheel is `py3-none-any` and `[core]`
resolves on both, but nothing was run there. `[convert]` cannot be installed on
macOS at all, because `usd-exchange` publishes no macOS wheel at any version.

[Unreleased]: https://github.com/fermarc98/urdf-usd-bridge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/fermarc98/urdf-usd-bridge/releases/tag/v0.1.0
