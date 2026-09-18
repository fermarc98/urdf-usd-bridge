# Phase 4 design — proving the repairs with simulation

> **Development record.** Written during the phase it describes and kept for
> provenance, not maintained since. Where it disagrees with the current
> documentation, the current documentation is right — start at
> [`docs/history/README.md`](README.md).

**Date:** 2026-09-18
**Status:** proposal. No harness code written yet.
**Host it targets:** Ubuntu 22.04.5, RTX 4090, Isaac Sim 6.1.0-rc.26.
**Reviewer decisions needed:** §11. Two of them change what gets measured.

Phase 3 shipped repairs whose tuning constants are declared *unmeasured*. This
phase measures them, or replaces them. It also has to be willing to conclude
that a repair made no difference — §10 says how that gets reported.

---

## 1. What I verified before designing

A cross-backend harness is only worth designing if all three backends can
actually be driven from one place. I checked on this machine first. Everything
in this section is an observation, not a plan.

| Question | Answer |
|---|---|
| Does Isaac Sim bundle Newton and MuJoCo? | **Yes** — `exts/isaacsim.pip.newton/pip_prebundle` carries `newton 1.5.0`, `mujoco 3.11.0`, `mujoco_warp 3.11.0`, `newton_usd_schemas 0.4.1`: exactly the Isaac-pinned stack |
| Can Newton import our stabilized USD? | **Yes** — `newton.ModelBuilder().add_usd(...)` returned 3 bodies, 3 joints, 2 DOFs for fixture (a) |
| Does Newton see our armature? | **Yes** — `joint_armature = [0.00075125, 0.004]`, our exact values |
| Can PhysX be stepped headlessly? | **Yes** — `SimulationApp({"headless": True})` + `World` + `Articulation`, ~10 s warm start |
| Does PhysX receive our drive gains? | **Yes** — `get_gains()` → stiffness `[299.547, 1594.928]`, damping `[11.035, 54.768]` |
| Is network available for a robot corpus? | **Yes** — GitHub raw and PyPI both reachable |

Two of those readings are worth dwelling on.

**Our unit table is confirmed by two independent consumers.** We author
`drive:angular:physics:stiffness = 5.228089` per degree. Newton's importer
divides by `DegreesToRadian` and gets `299.547`; Isaac's `get_gains()` reports
`299.547`. Both recover the `K_si` we computed, to six figures. The per-degree
convention in `docs/history/PHASE3_DESIGN.md` §3 is not a reading of the docs any more.

**G1 reproduces once more, in PhysX.** The unrepaired fixture (a) articulation
reports `stiffness = [0, 0]`, `damping = [0, 0]`. The repaired one reports the
values above. That is the before/after this phase is built to quantify.

---

## 2. Three findings that change the design

### 2.1 `--backend newton` is inert in Newton 1.5.0

Phase 3 authors `NewtonActuator` + `NewtonPDControlAPI` (`newton:kp`/`kd`) for
the Newton backend, on the strength of the schema documentation. Newton 1.5.0's
importer **does not read it**. Measured, same asset, three roots:

| Root we author | Newton `joint_target_ke` |
|---|---|
| `_stabilized_physx.usda` (`UsdPhysics.DriveAPI`, per degree) | **`[299.547, 1594.93]`** |
| `_stabilized_mujoco.usda` (`MjcActuator` `gainPrm`, per radian) | `[0, 0]` |
| `_stabilized_newton.usda` (`NewtonActuator` `kp`, per radian) | `[0, 0]` |

`newton/_src/utils/import_usd.py` reads `UsdPhysics.DriveAPI` (line 1663, then
`/= DegreesToRadian` at 1691) and, separately, `mjc:gainPrm`/`biasPrm` (line
5165). Nothing anywhere reads `NewtonActuator`. That is consistent with the
schema calling itself EXPERIMENTAL — it is a forward declaration, not a
consumed interface.

**Consequence:** Newton's actual input for drive gains is
`UsdPhysics.DriveAPI`. The Newton backend layer should author that, and keep
`NewtonActuator` as an additional forward-looking opinion rather than its only
one. This is a Phase 3 correction that Phase 4 forces, and it is **decision N1**
(§11) because it changes what `--backend newton` means.

### 2.2 MJC actuators need the MuJoCo schema plugin registered

The `mujoco` root also read back as `[0, 0]`. The MJC path in Newton's importer
scans for `MjcActuator` prims; under a bare `python.sh` the
`omni.usd.schema.mujoco` plugin is not registered, so those prims have no type
and the scan finds nothing. The harness must register the plugin explicitly
(`Plug.Registry().RegisterPlugins(<isaac>/exts/omni.usd.schema.mujoco/plugins/mjcPhysics/resources)`)
and **assert that it took**, because the failure mode is silence: a MuJoCo run
with no gains looks exactly like a MuJoCo run whose gains were dropped.

That assertion is itself a useful test, and it is the mechanism behind the
`NO-GAINS` actuator column `inspect` has reported since Phase 2.

### 2.3 The test pose has to load the joints

My first PhysX probe showed the repaired arm drifting 1.06 rad in one second,
which looked like the repair failing. It was not. Two scene artefacts:

* the default ground plane intersected the arm at its home pose, so contact
  forces, not gravity, moved it;
* with the ground plane removed, fixture (a) at its home pose drifts **0.000000
  in both the repaired and the unrepaired asset** — the shoulder axis is
  vertical there, so gravity exerts no torque about it and an undriven joint
  holds just as well as a driven one.

**A hold-pose test at the home pose of fixture (a) measures nothing.** The suite
must command a pose that actually loads each joint, and the scene contract (§4)
has to be explicit rather than inherited from a helper's defaults. This is the
single easiest way to produce a confident, meaningless result, and it nearly
did.

---

## 3. Architecture

```
src/urdf_usd_bridge/sim/
    __init__.py
    scene.py        # the scene contract: gravity, ground, dt, solver settings
    corpus.py       # robot list; fetches public URDFs, caches, never commits
    metrics.py      # PURE: trajectory arrays -> metric dict. No backend imports
    runner.py       # suite x backend x asset matrix, JSON out
    backends/
        base.py     # Backend protocol: load(), step(), read(), close()
        physx.py    # Isaac Sim: SimulationApp + World + Articulation
        newton.py   # newton.ModelBuilder.add_usd + SolverFeatherstone
        mujoco.py   # the same import + SolverMuJoCo (MuJoCo Warp)
    suites/
        hold_pose.py  drop.py  limit_sweep.py  gain_sweep.py  dt_sweep.py

scripts/run_sim_matrix.py     # the one command
tests/sim/                    # marked `gpu`; skip cleanly without one
docs/history/PHASE4_REPORT.md
```

`metrics.py` is pure by design: it takes arrays and returns numbers, so every
metric is unit-testable on synthetic trajectories with no GPU, no Isaac Sim and
no robot. That is what keeps the majority of this phase testable in CI.

**One interpreter.** All three backends run under Isaac Sim's `python.sh`,
because Isaac bundles Newton and MuJoCo Warp. `src/` goes on `sys.path` the way
`scripts/verify_isaac_regression.py` already does. Newton and MuJoCo can also
run in a plain venv with `newton[sim]`; whether MuJoCo Warp works without CUDA
there is **unverified** and is the fallback path, not the primary one.

### One command

```bash
python scripts/run_sim_matrix.py --out sim_artifacts/ --json results.json
```

with `--suite`, `--backend`, `--asset`, `--repaired/--baseline` filters for
narrowing, and `--quick` for a smoke subset. It writes:

```
sim_artifacts/
  results.json          every run, every metric, plus the environment block
  <asset>/<backend>/<suite>/<variant>/trajectory.npz    raw arrays
  summary.md            the table that goes into the report
```

`results.json` carries the same environment block Phase 2.5 established (OS,
GPU, driver, Isaac build, package versions) plus a per-run `needs_gpu` flag, so
§9's reproducibility claims are machine-checkable rather than prose.

---

## 4. The scene contract

Every number in this phase is meaningless without the scene it was produced in,
and §2.3 is what happens when a helper's defaults get inherited silently. So the
scene is declared once, in `scene.py`, applied identically to all three
backends, and echoed into every result record.

| Parameter | Value | Why |
|---|---|---|
| gravity | `-9.81 m/s^2` on `-Z` | stage `upAxis` is Z for both converters |
| ground | analytic plane at `z = 0`, friction 1.0, restitution 0.0 | a plane, not a box: no thin-feature contact artefacts |
| `dt` | `1/240 s` default; the dt sweep overrides it | the rate Isaac's `World` defaults to |
| solver iterations | position 32, velocity 1 (PhysX); Newton/MuJoCo defaults **recorded, not matched** | see the honesty note below |
| settle time before measuring | 0.5 s with drives active | removes the initial-contact transient from every metric |
| episode length | 5 s unless a suite says otherwise | |
| initial pose | per asset, from `corpus.py` — **a loaded pose**, never the home pose alone | §2.3 |
| base | fixed for arms, floating for the drop suite | |

**An honesty note on solver settings.** Matching iteration counts across three
different solvers is not possible in any meaningful sense — Featherstone,
PhysX's TGS and MuJoCo's Newton solver do not have comparable knobs. The
contract therefore fixes what *is* comparable (gravity, ground, dt, pose,
episode) and **records** the rest. Every cross-backend number in §7 is
"under each backend's defaults at a common dt", and the report will say exactly
that rather than implying the solvers were equalised.

---

## 5. The suites

Each suite runs on each asset in both variants — **baseline** (converter output,
unrepaired) and **repaired** — on each backend. A run that diverges or NaNs is a
result, not an error, and is recorded as such.

### 5.1 hold-pose

Fixed base. Joints commanded to a **loaded pose** (§2.3): each revolute joint set
to the midpoint of its range, or 30° from home if unlimited, chosen so gravity
exerts a non-zero torque about every actuated axis. Targets held for 5 s.

Reports the torque each joint sees at t=0 so the "is this pose actually loaded"
question is answered by the artefact, not assumed.

### 5.2 drop

Floating base, released 10 cm above the plane, zero initial velocity, 5 s.
Drives active at the same targets as hold-pose, so the asset has to survive
contact *and* hold itself together.

### 5.3 limit sweep

One joint at a time, commanded 20% beyond each limit, held 2 s, released.
Un-actuated joints are held at their loaded pose. Skips joints our own
`limits.report-ambiguous` flagged, since "what should this joint's limit be" is
exactly what we declined to guess.

### 5.4 gain sweep

`f_n` in {2, 5, 10, 20, 40} Hz × `zeta` in {0.5, 0.7, 1.0, 1.4}, re-running
`fix` for each combination and then hold-pose + a 10° step command. This is the
suite that produces the measured `f_n` and `zeta` (§8).

### 5.5 dt sweep

`dt` in {1/30, 1/60, 1/120, 1/240, 1/500, 1/1000}, hold-pose at each. The
largest `dt` that survives is the asset's **stability margin**, and comparing
that number between baseline and repaired is the cleanest single statement this
phase can make.

Also run with `alpha`/`beta` varied to isolate the armature contribution (§8).

---

## 6. Metric definitions

Precise, because "drift" and "jitter" mean nothing otherwise. All metrics are
computed in SI (radians, metres) from the recorded trajectory, after the 0.5 s
settle window. `q` is joint position, `v` joint velocity, `x` base pose.

| Metric | Definition | Units |
|---|---|---|
| `pose_drift_max` | `max_t max_j |q_j(t) - q_j(target)|` | rad / m |
| `pose_drift_final` | `max_j |q_j(T) - q_j(target)|` | rad / m |
| `velocity_rms` | `sqrt(mean_t mean_j v_j(t)^2)` | rad/s |
| `velocity_peak` | `max_t max_j |v_j(t)|` | rad/s |
| `jitter` | `sqrt(mean_t mean_j (v_j(t) - v_j(t-1))^2) / dt` — RMS joint acceleration, the thing that reads as buzz | rad/s^2 |
| `energy_drift` | `(E(T) - E(t_settle)) / max(E(t_settle), eps)`, `E` = kinetic + potential | dimensionless |
| `diverged` | any `|q|`, `|v|` non-finite, or `velocity_peak > 1e3`, or `pose_drift_max > pi` | bool |
| `settle_time` | first `t` after which `max_j |v_j| < 1e-3` for 0.5 s continuously; `inf` if never | s |
| `penetration_max` | `max_t max_i (-signed_distance_i(t))`, clamped at 0 | m |
| `base_height_error` | `|z_base(T) - z_rest_expected|` | m |
| `limit_overshoot` | `max_t max_j max(q_j - upper_j, lower_j - q_j, 0)` | rad / m |
| `limit_escape` | `limit_overshoot > 0.05 rad` sustained for > 0.1 s | bool |
| `chatter_freq` | dominant frequency of `v_j` while at a limit, by FFT of the post-settle window | Hz |
| `step_overshoot` | `(max_t q_j(t) - q_target) / (q_target - q_start)` for the step command | dimensionless |
| `step_settle_time` | first `t` with `|q_j - q_target| < 0.02 * |q_target - q_start|` and staying there | s |
| `dt_max_stable` | largest swept `dt` with `diverged == False` **and** `pose_drift_max` below the per-asset threshold | s |

`diverged` deliberately has three triggers: NaN is the obvious one, but a solver
that gains energy usually shows up as a velocity blow-up first, and a robot that
quietly folds in half shows up as drift.

**Pass criteria are relative, never absolute.** The claim this project makes is
"the repaired asset is no worse, and is better where the repair applies". So the
primary comparison is always repaired vs baseline on the same backend, same
asset, same scene, and the report states per metric whether the repair improved
it, made no measurable difference (`< 5%` relative change), or made it worse.

---

## 7. Cross-backend agreement, as its own metric

Same asset, same commands, same scene, three backends. Agreement is computed
per (asset, suite, variant):

| Metric | Definition |
|---|---|
| `agreement_pose` | `max over backend pairs of max_j |q_j^A(T) - q_j^B(T)|` |
| `agreement_settle` | `max pairwise |settle_time^A - settle_time^B|` |
| `agreement_base_height` | `max pairwise |z_base^A(T) - z_base^B(T)|` (drop suite) |
| `agreement_dt_margin` | `max pairwise |log2(dt_max^A) - log2(dt_max^B)|` — octaves apart |
| `backends_diverged` | how many of the three diverged, 0–3 |

The headline claim to test is: **does repairing reduce cross-backend
disagreement?** That is `agreement_*` on the repaired asset versus the baseline.
It is a stronger claim than "each backend got better on its own", and it is the
one that justifies the project's existence, so it is reported separately and
first.

A caveat stated up front: with `NewtonActuator` unread (§2.1), the current
Newton and MuJoCo runs would both be *undriven* regardless of repair, and
agreement would look artificially good for the wrong reason. Decision N1 has to
land before these numbers mean anything.

---

## 8. Replacing the unmeasured defaults

The brief asks for measured values, or a reason a default survived. The
procedure:

1. Run the **gain sweep** on every asset × backend. For each `(f_n, zeta)` cell
   record `pose_drift_max`, `step_overshoot`, `step_settle_time`, `jitter` and
   `diverged`.
2. Choose `f_n` as the **largest value that does not diverge on any asset or
   backend at the default dt, with a one-step margin** — i.e. the swept value
   below the first failure, not the failure itself.
3. Choose `zeta` as the value minimising `step_settle_time` subject to
   `step_overshoot <= 0.05`.
4. Run the **dt sweep** with `alpha`, `beta` at {0, default, 10x default} to
   isolate armature. Choose them by the largest `dt_max_stable` that does not
   measurably change `pose_drift_final` — armature that buys margin without
   changing the robot's behaviour is doing its job; armature that changes the
   pose is too big.
5. **If a default survives, say why**, with the sweep row that supports it. A
   default surviving is a real outcome and is not padding.

Then the constants move out of "unmeasured":

* `Defaults` in `repair/base.py` gains a `provenance` field per constant —
  either `measured: <sweep-id>` or `unmeasured`.
* The layer `customLayerData` `tuning_status` becomes
  `"measured on <corpus> across <backends>, see docs/history/PHASE4_REPORT.md"`, and the
  `unmeasured: true` flag in each drive record clears.
* `tests/unit/test_repair_drives.py::test_every_derived_record_is_marked_unmeasured`
  inverts into a test that the provenance string matches the measured constants,
  so the two cannot drift apart.

**If the sweep cannot justify a value** — no clear optimum, or the optimum
differs by more than a factor of two between backends or between robots — the
constant stays `unmeasured` and the report says so. A measured-looking number
with no measurement behind it would be worse than the honest default.

---

## 9. The corpus, and reproducibility

Four fixtures, plus **two public robots fetched at run time and never
committed**:

| Asset | Source | Why |
|---|---|---|
| fixtures (a)–(d) | this repo | the cases the repairs were written for |
| **Franka Panda** (7-DoF fixed-base arm) | `robot_descriptions` package, which fetches and caches from the upstream repo | a real serial arm with real inertias; the hold-pose and gain-sweep workhorse |
| **Unitree Go2** (12-DoF floating-base quadruped) | same | the drop suite needs a floating base and real contact |

`robot_descriptions` handles fetching, mesh resolution and caching in
`~/.cache/robot_descriptions`. Licences are recorded in the results file, and
nothing is vendored. If the package proves awkward, the fallback is a pinned
`git clone --depth 1` per robot into the cache directory — same properties, more
code. `corpus.py` records each robot's upstream URL and commit in
`results.json`, so a result can be traced to an exact input.

Fetch failures **skip with a clear reason** rather than failing the run, matching
how `tests/converter` already behaves when the matrix has not been built.

### What needs a GPU

Marked per run in `results.json` and in the report:

| Component | GPU? |
|---|---|
| `metrics.py` and its unit tests | **no** — pure functions on arrays |
| harness plumbing, corpus fetch, JSON schema | **no** |
| Newton `SolverFeatherstone` | CPU possible via Warp, **unverified**; GPU on this host |
| MuJoCo Warp | **almost certainly yes** — unverified whether a CPU fallback exists |
| PhysX via Isaac Sim | **yes**, plus a ~10 GB install |

CI keeps running T0/T1 as today and adds only the pure metric tests. The
simulation matrix is a local/manual command, and every number it produces is
labelled with the host block that produced it.

---

## 10. Reporting a repair that did not help

The brief asks for this explicitly, so it gets a mechanism rather than good
intentions. Each repair rule is assigned the metric it is supposed to move:

| Rule | Metric it must move |
|---|---|
| `drives.derive-gains` | `pose_drift_max`, `step_settle_time` |
| `drives.mirror-passive` | `velocity_rms`, `jitter` |
| `armature.default` | `dt_max_stable`, `jitter` |
| `inertia.derive-from-geometry` | `diverged`, `energy_drift`, `agreement_pose` |
| `inertia.principal-axes-identity` | `diverged` on the 0.3.2 column specifically |
| `inertia.make-physical` | `energy_drift`, `diverged` |
| `limits.restore-missing` | reachable workspace — joint range actually traversed in the limit sweep |

Each rule is then run **on and off** against its own metric, everything else
held constant, so a rule's contribution is isolated rather than inferred from
the bundle. `docs/history/PHASE4_REPORT.md` gets one row per rule with the measured
delta and a verdict: **proved**, **no measurable difference**, or **harmful**.
A rule with no measurable difference stays in the code with its verdict recorded
— it may still matter on a robot class the corpus does not cover — but the
README stops claiming it helps.

I will say plainly which repairs fall into each bucket, including if that turns
out to be most of them.

---

## 11. Decisions I need from you

**N1 — what `--backend newton` authors.** Newton 1.5.0 ignores
`NewtonActuator` (§2.1). Options: (a) author `UsdPhysics.DriveAPI` for Newton
*and* keep `NewtonActuator` as a forward-looking extra; (b) author only
`DriveAPI` and drop the Newton actuator until a release consumes it; (c) leave
Phase 3 as-is and report Newton as undriven.
*Recommendation: (a).* It makes Newton actually driven, costs one extra layer
opinion, and keeps the experimental schema present for whoever adopts it. It
does mean the Newton and PhysX layers become near-identical, which is worth
saying out loud.

**N2 — the corpus.** Panda + Go2 as proposed, or different robots? A quadruped
is what makes the drop and contact metrics meaningful, but it is also the most
likely to diverge for reasons that have nothing to do with our repairs (contact
parameters, which are G4/G5 and explicitly out of scope). An alternative second
robot is a simple gripper, which exercises the low-inertia armature floor
instead.
*Recommendation: Panda + Go2*, with the gripper added only if Go2's contact
behaviour swamps the signal.

**N3 — scope check.** Five suites × 6 assets × 3 backends × 2 variants is 180
runs before the sweeps, and the gain sweep alone is 20 cells × 6 assets × 3
backends. That is a lot of compute and a lot of report. Happy with the full
matrix, or should the sweeps be restricted to a subset (say Panda plus fixtures
(a) and (c)) with the rest running only hold-pose and drop?
*Recommendation: full matrix for hold-pose and drop; sweeps on a subset.*

**N4 — do the measured defaults ship in Phase 4, or land as a separate change?**
Replacing the constants changes every asset `fix` produces from then on.
*Recommendation: measure and report in Phase 4, change the defaults in the same
phase, with the old values recorded in the report so any asset built before the
change can be explained.*

---

## 12. What would make this phase fail honestly

Stated in advance so the report cannot quietly avoid them:

* **The fixtures are too small to be interesting.** Three-body arms may hold
  their pose in every backend regardless of repair (§2.3 already showed one
  case). If so, the real robots carry the phase and the fixtures only prove the
  repairs are not *harmful*.
* **Contact dominates the drop suite.** Convex-hull collision and absent physics
  materials are G4/G5, out of scope here. If drop metrics are driven by those
  rather than by drives and inertia, the drop suite measures the wrong thing and
  the report must say so rather than claim the difference for our repairs.
* **Cross-backend agreement may be poor for reasons we do not own** — different
  contact models, different default solver settings, different integrators. The
  agreement metric may end up measuring the ecosystem rather than the asset. If
  so, that is still a publishable finding, but it is not the same claim.
* **A GPU-only result is not independently reproducible** by a reader without
  the same hardware. Everything that can be checked without a GPU is pushed into
  `metrics.py` and its tests for exactly this reason.
