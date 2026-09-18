# Benchmark

Every measured number the project claims, in one place, with the negative
results left in.

Two runs produced what is here. The **14-robot matrix** (Phase 5) is the
headline; the **parameter sweeps** (Phase 4, on two fixtures) are what chose the
tuning defaults. They are kept apart below because their scopes differ and
merging them would overstate both. Method and raw output:
[`history/PHASE4_REPORT.md`](history/PHASE4_REPORT.md),
[`history/PHASE5_REPORT.md`](history/PHASE5_REPORT.md).

---

## Headline

On **14 robots × 3 backends = 84 cells**, hold-pose drift **improved in 35 and
got worse in none**, and **19 runs that diverged on the converter's output
converged after repair**.

Cross-backend agreement — the actual product claim — improved from
**7.9e-2 rad to 7.6e-5 rad** on the fixture where all three backends could be
compared before and after.

## What "hold-pose drift" means

The robot is placed in a gravity-loaded pose and commanded to stay there. After
a 0.5 s settle window, `pose_drift_max` is the largest absolute joint-angle
deviation from the command, in radians, over the remaining 4.5 s.

Nothing else changes between the two columns: same scene, same `dt`, same
episode, same commands. The only difference is whether the stability layers are
composed in or muted.

The scene is fixed for all three backends: gravity −9.81 on Z, `dt = 1/240`,
5 s episode, analytic ground plane, friction 1.0, restitution 0. Solver
iteration counts are **recorded, not matched** — Featherstone, PhysX TGS and
MuJoCo's solver have no comparable knobs, and pretending otherwise would be the
bigger lie.

## The table

`DIV` = diverged (NaN or runaway). Lower is better. **base** is the converter's
output; **rep** is the same asset with the stability layers.

| Robot | DoF | PhysX base | PhysX **rep** | Newton base | Newton **rep** | MuJoCo base | MuJoCo **rep** |
|---|---|---|---|---|---|---|---|
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

### The rows that do not flatter us

- **Every single baseline diverges in Newton** — all 14. The repair fixes 9.
  The 5 it does not are all serial arms and humanoids; that is
  [`UPSTREAM_ISSUES.md`](UPSTREAM_ISSUES.md) issue 3, and it caps the
  cross-backend claim at 9 of 14 robots.
- **PhysX on AlienGo barely moves**: 0.94 → 0.84 rad.
- **Fixture (d) does not improve at all**, and in Newton the repaired run is
  *worse* than a baseline that diverged.
- **Two robots are missing entirely** because a guard refused to measure them
  (see below).

### Cross-backend agreement

Worst pairwise final-pose gap between the three engines on the same asset:

| Robot | Baseline | Repaired |
|---|---|---|
| fixture (a) | 7.879e-2 rad | **7.586e-5 rad** |
| fixture (b) | not computable (a backend diverged) | **4.208e-5 rad** |
| fixture (d) | not computable | 1.241e-1 rad |
| Go2 | not computable | 2.841e+0 rad |
| SO-100, SO-101 | not computable | not computable (Newton diverges) |

On fixture (a) the repair closes the gap between three independent physics
engines by a factor of **1000**. On (b), the baseline gap cannot even be
computed, because Newton diverges — which is itself the finding.

The remaining gaps are not small, and the reasons are known: Go2's 2.8 rad is
contact-dominated, and the arms cannot be compared at all while Newton diverges
on them.

## The guards that refuse to report a number

Two guards run before any metric is recorded, and both changed results.

**Gravity loading.** A joint whose mass sits on its own axis cannot be loaded by
gravity at any pose, so hold-pose drift on it measures nothing. The guard
chooses the angle that *maximises* gravity torque rather than trusting the home
pose, and names the joints it excludes:

```
5 of 6 actuated joints are gravity-loaded at this pose; 1 joint(s) excluded from
the measurement because their mass sits on the joint axis and no pose can load
them: shoulder_pan
```

Excluded joints are dropped from the metric columns, so an untested joint cannot
dilute or fake a result. Two robots are excluded from the benchmark entirely on
these grounds: the **Unitree dexterous hand** (no pose loads any joint) and
**fixture (c)** (a joint deliberately left welded).

**Interpenetration.** No collider may start below the ground plane. This is the
artefact that made a working repair look broken in an early probe — the default
plane intersected the arm, and contact forces, not gravity, moved it.

A third trap was found mid-run and fixed: with the test pose at a joint's range
midpoint, PhysX held fixture (b) to 1e-7 rad with **zero stiffness**, because
the target sat on the limit stop and the *constraint* was doing the work. Test
poses are now inset from the stops, and the metrics carry an absolute floor so
that "2.3e-7 → 1.1e-7 rad" is not reported as a 52% win.

---

## The measured defaults

| Constant | Ships as | Basis |
|---|---|---|
| `target_frequency` | **derived** | `control_rate/6` for PhysX or MuJoCo alone; `control_rate/12` for Newton or any multi-backend asset |
| `damping_ratio` | 1.0 | minimises step settle time with zero overshoot |
| `armature_fraction` | 0.01 | measured to have **no effect**; retained at its original value |
| `armature_floor` | 1e-4 | **unmeasured** |
| `control_rate` | 60 Hz | an assumption, checked but never used in a formula |

### `target_frequency`, and why Phase 3's assumption was wrong

The gain sweep (5 frequencies × 3 damping ratios × 2 fixtures × 3 backends =
180 cells) confirmed the second-order model exactly: drift scales as `1/f²` to
three significant figures from 5 Hz through 20 Hz.

The **dt sweep** is what changed the default:

| `f_n` | at 60 Hz | at 120 Hz | at 240 Hz |
|---|---|---|---|
| 5 Hz | all three OK | all three OK | all three OK |
| 10 Hz | **Newton diverges** | all three OK | all three OK |
| 20 Hz | **Newton diverges** | **Newton diverges** | all three OK |
| 40 Hz | **Newton diverges** | **Newton diverges** | **Newton diverges** |

The pattern is exact: **every backend survives `f_n ≤ control_rate/12`, and
Newton's Featherstone solver diverges at `control_rate/6`.** PhysX and MuJoCo
tolerate `/6` everywhere — the constraint comes from Newton alone, and it binds
only because this project promises one asset that works in all three.

Phase 3 had assumed `control_rate/4` and reasoned that 10 Hz at 60 Hz "leaves
margin". It does not.

**The cost of the safe value, stated plainly:** at 5 Hz the steady-state sag is
`τ/K`, about 4e-2 rad (2.3°) on fixture (a), and a 10° step command does not
settle within 3 s in any backend. If you run at 240 Hz you can and should raise
both together — `--target-frequency 20 --control-rate 240` is measured-safe.

### `damping_ratio = 1.0`

At `f_n = 20 Hz`, where the step response is measurable at all:

| `ζ` | step overshoot | step settle |
|---|---|---|
| 0.7 | 0.051 | 0.263 s |
| **1.0** | **0.000** | **0.175 s** |
| 1.4 | 0.000 | 0.200 s |

Minimises settle time with zero overshoot. 0.7 overshoots by 5.1%, past the 5%
bound the design set. The default survived the measurement, which is a
different thing from never having been measured.

---

## Measured and found to do nothing

**`armature_fraction` has no measurable effect on the stability margin.** Swept
over 0, 0.01, 0.1 and 1.0 — a hundredfold range, including 0, which disables the
rule — at 60 Hz and 120 Hz:

| `α` | 60 Hz | 120 Hz |
|---|---|---|
| 0 (rule disabled) | Newton diverges | all OK |
| 0.01 (default) | Newton diverges | all OK |
| 0.1 | Newton diverges | all OK |
| 1.0 | Newton diverges | all OK |

Armature changed neither the divergence threshold nor the timestep at which it
occurs. The Phase 3 rationale — that armature buys stability margin — is **not
supported**. Drift does improve slightly at large `α`, but only because
`I_total` grows and the gain formula raises `K` with it: that is the drive rule
working, not the armature rule.

The rule ships at its original value with this recorded as its provenance,
rather than being quietly deleted or quietly kept. The honest caveat: the case
armature exists for is a DOF whose own inertia is negligible, and the swept
fixture has none.

## Where repair made things worse

**Joint-limit overshoot, 6 of 20 cells.** A driven joint commanded past its stop
pushes into it harder than an undriven one:

| Robot | Backend | Baseline | Repaired |
|---|---|---|---|
| fixture (b) | MuJoCo | 0.046 | **0.328** |
| fixture (a) | MuJoCo | 0.002 | 0.041 |
| fixture (a) | Newton | 0.000 | 0.021 |
| fixture (d) | Newton | 0.000 | 0.024 |
| Go2 | Newton | 0.000 | 0.029 |
| fixture (d) | MuJoCo | 0.000 | 0.012 |

Expected — an actuated joint has force to push with — but a real regression
against an undriven baseline, and the evidence that `limits.compliance` is worth
having. It is still not derivable from the asset alone
([`ROADMAP.md`](ROADMAP.md) §2).

**`velocity_rms` worse in 7 cells**, against 23 improved. A joint actively
holding a pose is not always quieter than a limp one.

**Drop-test base height worse in 3 cells.** Contact-dominated; see below.

## Repair-by-repair verdicts

| Rule | Metric it should move | Verdict |
|---|---|---|
| `drives.derive-gains` | `pose_drift_max`, `settle_time` | **Proved.** Improved drift in every cell it touched and worsened none; turns diverging runs into converging ones |
| `drives.mirror-passive` | `velocity_rms`, jitter | **Proved.** 23 improved / 7 worse on `velocity_rms`; 12 improved / 2 worse on jitter |
| `inertia.derive-from-geometry` | `diverged` | **Proved.** Fixtures (b) and (d) diverge in Newton without it and converge with it |
| `inertia.principal-axes-identity` | `diverged` on converter 0.3.2 | **Not isolated.** It fires on the same bodies as the tensor rule, so this run cannot separate their contributions |
| `inertia.make-physical` | tensor validity | **Not exercised.** No corpus robot had a negative or triangle-violating tensor. The rule is unit-tested against synthetic tensors and has never fired on a real one |
| `armature.default` | `dt_max_stable` | **No measurable effect** (above) |
| `limits.restore-missing` | reachable range | **Mixed.** Unlocks the joint as designed, but MuJoCo then refuses to compile a `[0,0]` joint at all in the baseline, so the comparison is not like-for-like |

## What is not measured

- **`armature_floor`.** The low-inertia case it exists for is not represented in
  the corpus. A dexterous hand was added specifically to test it, and the
  gravity guard refused the robot.
- **Everything about contact.** Collision approximation and physics materials
  are untouched, so the drop-suite numbers are dominated by backend contact
  defaults rather than by anything this project does. They are not reported as
  a result.
- **Anything outside the corpus**: three arms, six quadrupeds, two humanoids and
  a hand, plus four fixtures — and the real robots come from only **two**
  repositories, so they share two vendors' conventions.

---

## The corpus

Fetched at run time, never vendored. Each robot's URL, commit and licence is
recorded in `results.json`.

| Morphology | Robots |
|---|---|
| Arms | SO-101, SO-100, Unitree Z1 |
| Quadrupeds | A1, Go1, Go2, AlienGo, B2, Laikago |
| Humanoids | H1 (19 DoF), G1 (23 DoF) |
| Hand | Unitree dexterous hand |
| Fixtures | (a) dynamics damping, (b) missing inertia, (c) welded joint, (d) mesh collision |

Sources: [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0)
and [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
(BSD-3-Clause). All 12 real robots convert and repair with **zero failures** —
891 repairs in total, from 36 on the SO-100 to 138 on the G1.

`DROPPED_ROBOTS` in `sim/corpus.py` records what did not make it: Franka Panda
and UR5 ship xacro only, and expanding xacro needs an ament package index that a
bare clone plus a ROS install could not satisfy.

## Environment

Ubuntu 22.04.5 x86-64, RTX 4090 (driver 580.178.04), Isaac Sim 6.1.0-rc.26,
Newton 1.5.0, Warp 1.16.0, MuJoCo 3.11.0, `usd-core` 26.8,
`urdf-usd-converter` 0.3.2.

The Newton divergence was separately re-checked on **Newton 1.6.0 / Warp 1.17.0
/ MuJoCo 3.12.0** from PyPI: all eight probe robots diverge at the same
millisecond as on 1.5.0.

## Reproducing

All of this needs an NVIDIA GPU and an Isaac Sim install; all three backends run
under Isaac's interpreter.

```bash
# the 14-robot matrix, ~50 minutes on an RTX 4090
<isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts --exploratory

# the gain sweep
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
provenance, every guard result with its evidence, and every metric — including
the cells that got worse.

The metric definitions themselves are pure Python and run without a GPU:
`sim/metrics.py`, covered by unit tests on hand-built trajectories whose answers
are known by construction. A difference between backends can never come from a
difference in how a metric was computed.
