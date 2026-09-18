# Phase 4 report — the repairs, measured

**Date:** 2026-09-18
**Host:** Ubuntu 22.04.5 x86-64, RTX 4090, driver 580.178.04
**Stack:** Isaac Sim 6.1.0-rc.26, Newton 1.5.0, Warp 1.16.0, MuJoCo 3.11.0, `usd-core` 26.8
**Design:** `docs/PHASE4_DESIGN.md`, approved with decisions N1–N4.
**Reproduce:** `<isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts --exploratory`

Everything in this report needs a GPU except §7, which is pure arithmetic on
recorded arrays and runs in CI on any machine.

---

## 1. Headline

Across 126 matrix cells on 7 robots and 3 backends (116 produced metrics, 6 were
refused by a guard, 4 errored), **`pose_drift_max` improved
in 14 cells and got worse in none**, and **9 runs that diverged with the
converter's output did not diverge after repair**.

The result that matters most:

| Robot | Backend | Baseline hold-pose drift | Repaired |
|---|---|---|---|
| **SO-101 arm** (5-DoF + gripper) | PhysX | **diverged** (3.71 rad) | **0.105 rad** |
| SO-101 arm | MuJoCo | **diverged** (3.72 rad) | **0.105 rad** |
| SO-100 arm | PhysX | 2.19 rad | **0.302 rad** |
| SO-100 arm | MuJoCo | **diverged** (2.2e17) | **0.302 rad** |
| Unitree Go2 | PhysX | 2.79 rad | **0.029 rad** |
| Unitree Go2 | MuJoCo | **diverged** (4.2e19) | **0.029 rad** |
| Unitree Go2 | Newton | **diverged** (NaN) | **0.029 rad** |

A real 5-DoF arm converted from public URDF collapses or diverges in every
backend as the converter emits it, and holds its pose to 0.1 rad after the
repairs. That is the claim this project was built to make, and it is now
measured rather than asserted.

**Cross-backend agreement improved by three orders of magnitude** where it could
be computed at all: on fixture (a) the three backends ended up 7.9e-2 rad apart
on the converter's output and **7.6e-5 rad apart** after repair.

And the honest other half: **`armature.default` had no measurable effect**,
Newton still diverges on both SO arms even repaired, the drives make joints
overshoot their limits harder in 6 cells, and the drop suite is dominated by
contact behaviour this phase does not touch. §8 and §9.

---

## 2. What was measured, and on what

| Robot | Role | Source | Commit |
|---|---|---|---|
| fixtures (a)–(d) | fixture | this repo | — |
| **SO-101 arm** | primary | `TheRobotStudio/SO-ARM100`, Apache-2.0 | `eecbe3e0a9eb` |
| **SO-100 arm** | primary | same | `eecbe3e0a9eb` |
| Unitree Go2 | exploratory | `unitreerobotics/unitree_ros`, BSD-3-Clause | `ccfc6fd8430a` |

Nothing is committed. `corpus.py` shallow-clones at run time into
`~/.cache/urdf-usd-bridge-corpus` and records each URL, commit and licence in
`results.json`.

**Decision N2 said Panda plus a gripper or UR5. Neither Panda nor UR5 could be
used**: `frankaemika/franka_description` and `ros-industrial/universal_robot`
both ship **xacro**, not URDF, and expanding xacro needs the ROS `xacro` tool
*and* an ament package index — ROS Humble is installed on this host and still
could not resolve `package 'ur_description' not found` from a bare clone.
Rather than bolt a ROS workspace build onto a physics harness, the corpus uses
the SO-ARM arms, which ship plain URDF, include a gripper, and are one of the
corpora `urdf-usd-converter` benchmarks against. `DROPPED_ROBOTS` in
`corpus.py` records the reason.

**Decision N3 (staged):** full matrix for hold-pose, drop and limit-sweep on all
7 robots × 3 backends × 2 variants. Sweeps were restricted to fixtures (a) and
(b), because a sweep cell re-runs `fix` and then two suites per backend, and the
5×3 gain grid alone took 20 minutes on two fixtures. That subset is justified by
what the sweep is *for*: choosing `f_n` and `zeta`, which are properties of the
second-order model, not of a particular robot — the model's prediction
(`drift ∝ 1/f²`) held to three significant figures across the grid, so a wider
corpus would have re-measured the same relation.

---

## 3. The scene, and the two guards

Fixed for all three backends: gravity −9.81 on Z, `dt = 1/240`, 5 s episode,
0.5 s settle window discarded, analytic ground plane, friction 1.0, restitution
0. Solver iteration counts are **recorded, not matched** — Featherstone, PhysX
TGS and MuJoCo's solver have no comparable knobs, and pretending otherwise would
be the bigger lie.

Both guards from the brief are enforced, and both caught real problems.

**Gravity loading.** `docs/PHASE4_DESIGN.md` §2.3 records how close this phase
came to a confident null result. The guard now *chooses* the joint angle that
maximises gravity torque about each axis rather than checking the home pose, and
it distinguishes three cases: joints loaded at the commanded pose, joints that a
different angle would load, and joints no pose can load. The third is real — the
SO-101's `shoulder_pan` is a base yaw whose mass sits on its own axis:

```
5 of 6 actuated joints are gravity-loaded at this pose; 1 joint(s) excluded from
the measurement because their mass sits on the joint axis and no pose can load
them: shoulder_pan
```

Excluded joints are named and **dropped from the metric columns**, so an
untested joint cannot dilute or fake a result.

**Interpenetration.** No collision geometry may start below the plane. This is
the artefact that made a working repair look broken in an early probe: the
default ground plane intersected the arm and contact forces, not gravity, moved
it.

**A third trap the guards did not initially catch, found during the run.** With
the pose chosen at a joint's range midpoint, PhysX held fixture (b) to 1e-7 rad
with *zero stiffness* — because the target sat exactly on the limit stop and the
**constraint**, not the drive, was doing the work. The pose is now inset from
the stops. A metric floor was added at the same time: an improvement from
2.3e-7 rad to 1.1e-7 rad is a 52% relative gain on a quantity nobody can
perceive, and reporting it as a win would have filled this document with noise.

---

## 4. Hold-pose: the primary result

Drift is measured over the gravity-loaded joints only, after the settle window.

| Robot | PhysX base → rep | Newton base → rep | MuJoCo base → rep |
|---|---|---|---|
| (a) dynamics damping | 1.34e-1 → **4.00e-2** | 1.35e-1 → **4.01e-2** | 1.36e-1 → **4.01e-2** |
| (b) missing inertia | 5.00e-2 → **3.02e-2** | **diverged** → **3.02e-2** | 9.57e-2 → **3.02e-2** |
| (d) mesh collision | 5.00e-2 → 5.00e-2 | **diverged** → 2.52e-1 | 5.02e-2 → 5.03e-2 |
| **SO-101 arm** | **diverged** → **1.05e-1** | diverged → **diverged** | **diverged** → **1.05e-1** |
| **SO-100 arm** | 2.19e+0 → **3.02e-1** | diverged → **diverged** | **diverged** → **3.02e-1** |
| Go2 *(exploratory)* | 2.79e+0 → **2.90e-2** | **diverged** → **2.85e-2** | **diverged** → **2.85e-2** |

Fixture (c) is absent because its guard fails: `no_limit_joint` is welded at
`[0,0]` in the baseline and `partial_limit_joint` stays welded in both variants
by design, so at the commanded pose nothing is loaded. That is the guard working.

### 4.1 Cross-backend agreement

The project's actual claim — same asset, same commands, same answer — as its own
metric. Worst pairwise final-pose gap between backends:

| Robot | Baseline | Repaired |
|---|---|---|
| (a) dynamics damping | 7.879e-2 rad | **7.586e-5 rad** |
| (b) missing inertia | not computable (a backend diverged) | **4.208e-5 rad** |
| (d) mesh collision | not computable | 1.241e-1 rad |
| Go2 | not computable | 2.841e+0 rad |
| SO-100, SO-101 | not computable | not computable (Newton diverges) |

On fixture (a) the repair closes the gap between three independent physics
engines by a factor of **1000**. On (b) the baseline gap cannot even be computed
because Newton diverges, which is itself the finding.

The gaps that remain are not small, and §9 is honest about why: Go2's 2.8 rad
disagreement is contact-dominated, and the SO arms cannot be compared at all
while Newton diverges on them.

---

## 5. Measured defaults

Decision N4: the measured values ship, with the previous ones and the change
date recorded in every output layer's `customLayerData`.

| Constant | Phase 3 | Phase 4 | Provenance |
|---|---|---|---|
| `target_frequency` | 10.0 Hz | **5.0 Hz** | **measured** — see below |
| `damping_ratio` | 1.0 | **1.0** | **measured**, confirmed |
| `armature_fraction` | 0.01 | 0.01 | **measured: no effect**, retained |
| `armature_floor` | 1e-4 | 1e-4 | **still unmeasured** |
| `control_rate` | 60.0 Hz | 60.0 Hz | relation measured, value unchanged |

Reports and layers now say `partly measured: armature_floor remains
unmeasured`, derived from the provenance table rather than hand-written, so the
claim cannot go stale.

### 5.1 `target_frequency`: 10 Hz → 5 Hz

The gain sweep (5 frequencies × 3 damping ratios × 2 fixtures × 3 backends, 180
cells, all producing metrics) confirmed the model exactly — drift scales as
`1/f²`, to three significant figures, from 5 Hz through 20 Hz. Divergence began
at 40 Hz.

The dt sweep is what changed the default:

| `f_n` | at 60 Hz | at 120 Hz | at 240 Hz |
|---|---|---|---|
| 5 Hz | all three OK | all three OK | all three OK |
| 10 Hz | **Newton diverges** | all three OK | all three OK |
| 20 Hz | **Newton diverges** | **Newton diverges** | all three OK |
| 40 Hz | **Newton diverges** | **Newton diverges** | **Newton diverges** |

The pattern is exact: **every backend survives `f_n ≤ control_rate / 12`, and
Newton's Featherstone solver diverges at `control_rate / 6`.** PhysX and MuJoCo
tolerate `/6` in every cell — the constraint comes from Newton alone, and it
binds only because this project promises one asset that works in all three.

Phase 3 assumed `control_rate / 4` and reasoned that "10 Hz is roughly a sixth
of a 60 Hz control rate, which leaves margin". Measurement says it does not.
The warning threshold in `drives.derive-gains` moved from `/4` to `/12`, and the
default `f_n` moved to 5 Hz so that the shipped pair clears its own check.

**The cost is stated plainly:** at 5 Hz the steady-state sag is `τ/K`, about
4e-2 rad (2.3°) on fixture (a), and a 10° step command does not settle within
3 s in any backend. A user targeting 240 Hz can and should raise both together —
`--target-frequency 20 --control-rate 240` is measured-safe.

### 5.2 `damping_ratio`: 1.0 confirmed

At `f_n = 20 Hz`, where the step response is measurable at all:

| `zeta` | step overshoot | step settle |
|---|---|---|
| 0.7 | 0.051 | 0.263 s |
| **1.0** | **0.000** | **0.175 s** |
| 1.4 | 0.000 | 0.200 s |

ζ = 1.0 minimises settle time with zero overshoot; ζ = 0.7 overshoots by 5.1%,
past the 5% bound the design set. The Phase 3 default survives, and now has a
row behind it.

### 5.3 `armature_fraction`: measured to make no difference

Swept α over 0, 0.01, 0.1 and 1.0 — a hundredfold range — at 60 Hz and 120 Hz:

| α | 60 Hz | 120 Hz |
|---|---|---|
| 0 (rule disabled) | Newton diverges | all OK |
| 0.01 (default) | Newton diverges | all OK |
| 0.1 | Newton diverges | all OK |
| 1.0 | Newton diverges | all OK |

**Armature changed neither the divergence threshold nor the timestep at which it
occurs.** The Phase 3 rationale — that armature buys stability margin — is not
supported by this experiment. Drift improves slightly at large α, but only
because `I_total` grows and the gain formula raises `K` with it; that is the
drive rule working, not the armature rule.

The rule is retained at its Phase 3 value and its provenance now says so. The
honest caveat: the case armature exists for is a DOF whose own inertia is
negligible, and fixture (a) has none. The SO-101's gripper is such a joint and
was not swept — that is the experiment that would settle it, and it was not run.

### 5.4 `armature_floor`: still unmeasured

For the same reason. It is marked `unmeasured` rather than given a number it did
not earn.

---

## 6. Repair-by-repair verdicts

The brief asked that a repair with no measurable effect be reported as such.

| Rule | Metric it should move | Verdict |
|---|---|---|
| `drives.derive-gains` | `pose_drift_max`, `settle_time` | **Proved.** 14 improved / 0 worse on drift; 18 improved on settle time; turns 9 diverging runs into converging ones |
| `drives.mirror-passive` | `velocity_rms`, `jitter` | **Proved.** 23 improved / 7 worse on `velocity_rms`; 12 improved / 2 worse on jitter |
| `inertia.derive-from-geometry` | `diverged` | **Proved.** Fixture (b) and (d) diverge in Newton without it and converge with it; 3 tensors derived on the SO-101 |
| `inertia.principal-axes-identity` | `diverged` on 0.3.2 | **Not isolated.** It fires on the same bodies as the tensor rule, so this run cannot separate their contributions |
| `inertia.make-physical` | `energy_drift` | **Not exercised.** No corpus robot had a negative or triangle-violating tensor |
| `armature.default` | `dt_max_stable` | **No measurable effect** (§5.3) |
| `limits.restore-missing` | reachable range | **Mixed.** Unlocks the joint as designed, but see §9.2 |

`inertia.make-physical` not firing is worth saying plainly: the rule is written
and unit-tested against synthetic tensors, and no robot in this corpus needed
it.

---

## 7. What runs without a GPU

`metrics.py` is pure — arrays in, numbers out — and `scene.py`'s guards are
pure geometry. 37 tests cover them, on hand-built trajectories whose answers are
known by construction, and they run in CI on any machine. The suite is now
**246 tests, 0 skipped**.

Everything else in this document needs an RTX-class GPU and a ~32 GB Isaac Sim
install. Each result row carries the environment block that produced it.

---

## 8. Where the repairs made things worse

**Joint-limit overshoot, 6 cells.** A driven joint commanded past its stop
pushes into it harder than an undriven one:

| Robot | Backend | Baseline | Repaired |
|---|---|---|---|
| (b) missing inertia | MuJoCo | 0.046 | **0.328** |
| (a) dynamics damping | MuJoCo | 0.002 | 0.041 |
| (a) dynamics damping | Newton | 0.000 | 0.021 |
| (d) mesh collision | Newton | 0.000 | 0.024 |
| Go2 | Newton | 0.000 | 0.029 |
| (d) mesh collision | MuJoCo | 0.000 | 0.012 |

This is expected — an actuated joint has force to push with — but it is a real
regression against an undriven baseline, and it points straight at
`limits.compliance`, the rule Phase 3 deliberately declined to write because
deriving a limit stiffness needs the effective inertia at `qpos0`. This is the
evidence that it is worth writing.

**Drop-test base height, 3 cells worse.** Contact-dominated; see §9.1.

**`velocity_rms` worse in 7 cells**, against 23 improved. A driven joint that is
actively holding a pose is not always quieter than a limp one.

---

## 9. What this phase did not establish

### 9.1 The drop suite measures contact, not our repairs

Predicted in `docs/PHASE4_DESIGN.md` §12, and confirmed. Convex-hull collision
(G4) and absent physics materials (G5) are out of Phase 3's scope, and they
dominate every drop metric: `base_height_drop` came out 3 worse / 2 improved /
2 no-difference, with no pattern attributable to drives or inertia. **The drop
numbers should not be read as evidence about these repairs**, in either
direction.

### 9.2 MuJoCo refuses to compile a `[0,0]` joint at all

Fixture (c) fails in MuJoCo for both variants:

```
ValueError: Error: range[0] should be smaller than range[1] in joint
```

MuJoCo's compiler rejects a welded `[0,0]` limit outright — a stronger reaction
than PhysX's or Newton's, and a good argument that G7 matters. But **the
repaired asset fails too**, because `limits.restore-missing` deliberately
unlocks only the joint with no limit evidence and leaves `partial_limit_joint`
locked, which is enough to keep MuJoCo from compiling.

That is a genuine tension between "refuse to guess" and "make the asset usable",
and this phase does not resolve it. Options for Phase 5: a `--force`-style
opt-in that unlocks ambiguous joints too, or emitting `mjc:limited = false` for
them.

### 9.3 Newton still diverges on both SO arms

Repaired or not, `SolverFeatherstone` produces NaN on the SO-100 and SO-101 —
the only two real arms in the corpus — while PhysX and MuJoCo run the same
repaired assets to 0.10 and 0.30 rad. The cause was not established. It is not
the gains (5 Hz at 240 Hz clears the measured `/12` bound by 4×) and it is not
armature (§5.3). Until it is understood, **cross-backend agreement cannot be
computed for the two most representative robots in this report**, which is a
real limit on the headline claim.

### 9.4 The fixtures are too small to be interesting

Also predicted. Fixtures (a)–(d) are three- and four-body arms; two of them have
a joint gravity cannot load at any angle, and fixture (c) cannot be hold-pose
tested at all. They prove the repairs are not *harmful* and they exercise the
authoring; the real robots carry the phase.

### 9.5 Suites written but not run to completion

`gain_step` ran only inside the sweeps. The `dt_max_stable` metric is
implemented and unit-tested but is not reported per robot, because deriving it
for 7 robots × 3 backends × 6 timesteps was beyond the compute budget for this
phase. `energy_drift` is implemented but no backend currently reports an energy
series, so it is never populated — it should be removed or wired up rather than
left looking like a metric that was measured.

---

## 10. Decisions, as implemented

**N1 — `--backend newton` authors `UsdPhysics.DriveAPI`.** Verified: Newton
reads `299.547` from the per-degree value we author and converts it itself.
Newton's drive damping carries the drive term alone (`9.535` vs PhysX's
`11.035`), because Newton reads `newton:damping` separately and folding the
passive term in would count it twice. `NewtonActuator` moved behind
`--newton-actuator`, off by default, documented with the version pin and the
double-driving risk. `test_every_backend_root_has_an_effective_drive` fails if
any backend root ends up with no gain a real consumer reads — the bug that made
Phase 3's Newton layer inert.

**N2 — corpus.** SO-101 + SO-100 primary; Go2 exploratory and reported
separately. Panda and UR5 dropped, with reasons (§2).

**N3 — staged.** Full matrix for hold-pose, drop and limit-sweep; sweeps on
fixtures (a) and (b), justified in §2.

**N4 — measured defaults ship**, with `previous_defaults` and
`defaults_changed` in every output layer.

---

## 11. Reproducing this

```bash
# the full matrix, ~50 minutes on an RTX 4090
<isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts --exploratory

# the gain sweep behind section 5.1
<isaac>/python.sh scripts/run_sim_matrix.py --out sweep --sweep \
    --asset a_dynamics_damping --asset b_inertial_origin_mass_no_inertia \
    --frequencies 2,5,10,20,40 --zetas 0.7,1.0,1.4

# the dt sweep that moved the default
<isaac>/python.sh scripts/run_sim_matrix.py --out dtsweep --sweep \
    --asset a_dynamics_damping --frequencies 5,10,20,40 --zetas 1.0 \
    --dts 0.0166667,0.0083333,0.0041667

# the armature sweep that found nothing
<isaac>/python.sh scripts/run_sim_matrix.py --out armsweep --sweep \
    --asset a_dynamics_damping --frequencies 10 --zetas 1.0 \
    --alphas 0,0.01,0.1,1.0 --dts 0.0166667,0.0083333
```

Artifacts are gitignored. `results.json` carries the environment, every asset's
provenance, every guard result with its evidence, and every metric.

---

## 12. What Phase 5 should do

1. **Find out why Newton diverges on the SO arms** (§9.3). It blocks the
   headline claim on the most representative robots here.
2. **Write `limits.compliance`** (§8). The overshoot regression is the evidence
   Phase 3 said it wanted before inventing a limit stiffness.
3. **Resolve the `[0,0]` tension** (§9.2) — an asset MuJoCo cannot compile is
   not much use, even if refusing to guess is defensible.
4. **Sweep armature on a low-inertia DOF** (§5.3), which is the case it exists
   for and the only way to justify or drop the rule.
5. **G4 and G5** — collision approximation and physics materials. The drop suite
   is ready and currently measures nothing else.
