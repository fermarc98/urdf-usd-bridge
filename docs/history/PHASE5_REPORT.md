# Phase 5 report — closing the holes, and making it usable

> **Development record.** Written during the phase it describes and kept for
> provenance, not maintained since. Where it disagrees with the current
> documentation, the current documentation is right — start at
> [`docs/history/README.md`](README.md).

**Date:** 2026-09-18
**Host:** Ubuntu 22.04.5 x86-64, RTX 4090, Isaac Sim 6.1.0-rc.26
**Tests:** 262 pass, 0 skipped. ruff and black clean. `references/` untouched.

---

## 1. Headline

The corpus went from 2 real robots to **12**, and the benchmark from 3 to
**14 robots × 3 backends**. On that larger corpus the result held and got
stronger: **`pose_drift_max` improved in 35 cells and got worse in none**, and
**19 diverging runs converged after repair**.

Two holes are closed, one is closed with an answer nobody wanted, and the
upstream reports are filed:

| Phase 5 item | Outcome |
|---|---|
| Newton divergence on the SO arms | **Cause narrowed, not identified.** Five hypotheses ruled out by measurement; it is not ours. Filed as upstream issue 3 |
| Joint-limit overshoot | **Stays report-only**, with the measurement embedded in the record |
| Corpus ≥ 10 robots | **12 real robots**, all converting and repairing cleanly |
| Usability | README, quickstart, 3 examples, benchmark table, CONTRIBUTING, `docs/API.md` |
| File upstream issues | **Filed** after review: IsaacSim#841, #842, newton#4269. §6 |

---

## 2. Newton divergence: what it is not

Phase 4 left this as "cause not established". Phase 5 established what it is
**not**, which is most of the value, and found the failure is far broader than
two arms.

### It is not ours

| Hypothesis | Test | Result |
|---|---|---|
| Our drive gains | zero `joint_target_ke`/`kd` | **still diverges** (t=0.267 s) |
| Our repairs at all | the converter's raw output | **still diverges** (t=0.263 s) |

### It is not the usual suspects

| Hypothesis | Test | Result |
|---|---|---|
| Timestep / explicit integration | dt 1/240 → 1/1000 → 1/4000 | **still diverges** at every dt |
| The 1e-9 kg marker link | raise its mass to 1e-6 … 1e-2 kg | no change below 1e-2; 1e-2 delays by 6 steps |
| Mass-ratio conditioning | `collapse_fixed_joints=True` drops the ratio 1.47e8 → 8.67 | **diverges at the identical time**, 0.0875 s |
| One bad robot | SO-100, mass ratio 49, no marker link | **also diverges** |

That third row is the one that killed the most attractive theory. The SO-101
really does have a `gripper_frame_link` with `mass="1e-9"` and a zero inertia
tensor in its source URDF, and it really does give a mass ratio of 1.5e8 — and
removing it changes nothing at all.

**A smaller timestep not helping** is what separates this from ordinary
explicit-integration instability, and it is the single most diagnostic fact
here.

### What does change it

| Change | Effect |
|---|---|
| gravity off | **stable** |
| joint limits widened to ±1e6 | diverges at 0.863 s instead of 0.086 s |
| `joint_armature = 1e-1` on every DOF | **stable** |
| same `Model`, `SolverMuJoCo` instead | **stable** for the full run |

The armature that suppresses it is ~1000× the inertia of the bodies it
regularises (the model's own inertias span 1e-6 to 1.6e-4 kg·m²), so it is
evidence about conditioning, not a workaround anyone can use.

### And it is not two arms — it is a morphology

The 14-robot benchmark turned a two-robot curiosity into a pattern:

| Morphology | Robots | Newton, repaired |
|---|---|---|
| Quadruped, 12 DoF | a1, aliengo, b2, go1, go2, laikago | **all 6 converge** |
| Serial arm, 6 DoF | so100, so101, z1 | **all 3 diverge** |
| Humanoid, 19 / 23 DoF | h1, g1 | **both diverge** |
| Fixture, 2–3 DoF | (a), (b), (d) | all converge |

A 12-DoF quadruped converges while a 6-DoF arm does not, so this is not size.
It splits by morphology — and the failing cases are long serial chains, which
is the case a reduced-coordinate articulated-body algorithm ought to be best
at. We cannot explain that, and the issue says so.

*(An earlier draft of this section claimed a chain-depth correlation. The
script that computed depth returned zero for every robot — the root-detection
was wrong, because the converter's synthesised `root_joint` makes the base link
appear as somebody's child. The claim was dropped rather than repaired, since
the morphology split stands on its own.)*

**Also worth stating**: every one of the 14 baselines diverges in Newton. The
repair fixes 9 of them. So Newton is both the backend our repairs help most and
the one that still fails hardest.

Written up as `docs/UPSTREAM_ISSUES.md` issue 3, against
`newton-physics/newton`, with the full elimination table and an explicit
section on what we could not determine.

---

## 3. Joint-limit overshoot: still report-only, now with evidence

Phase 4 measured driven joints overshooting their stops harder than undriven
ones in 6 of 20 cells, worst 0.046 → 0.328 rad. The brief asked for
`limits.compliance` implemented with swept values, or a statement of why it is
still not derivable.

**It is still not derivable, and the reason has not changed:**
`newton:limitStiffness` is an effort per unit penetration, and converting
MuJoCo's `solreflimit` to it requires the effective inertia at `qpos0`. That is
a pose-dependent quantity, and baking it into a static attribute would make the
asset wrong at every other pose. `mujoco-usd-converter` declines it for exactly
this reason (`joint.py:99-104`) and is right to.

A sweep would not fix this. Sweeping would find *a* value that works for one
robot at one pose, which is precisely the overfitting the refusal exists to
avoid — and the project's rule is that a repair must be justified by a
measurement that generalises.

What changed: the rule's record now carries the Phase 4 measurement, so a
reader sees the size of the problem rather than only the refusal.

```
joint has finite limits and no limit compliance, so PhysX treats the stop as
rigid while MuJoCo derives a soft one from solreflimit ... Measured in Phase 4:
a repaired, driven joint overshoots its stop harder than the undriven baseline
in 6 of 20 cells (worst 0.046 -> 0.328 rad). Still not repaired, because a limit
stiffness cannot be derived from the asset alone ...
```

The honest alternative for a future phase is **per-joint compliance measured at
simulation time** — a calibration pass, not a static repair.

---

## 4. Corpus: 12 real robots

All fetched at run time, none vendored, each with its URL, commit and licence
in `results.json`.

| Morphology | Robots |
|---|---|
| Arms | SO-101, SO-100, Unitree Z1 |
| Quadrupeds | A1, Go1, Go2, AlienGo, B2, Laikago |
| Humanoids | H1 (19 DoF), G1 (23 DoF) |
| Hand | Unitree dexterous hand |

All 12 convert and repair with **zero failures** — 891 repairs in total, from
36 on the SO-100 to 138 on the G1.

Robots from the same repository share one clone, so twelve robots cost two
downloads.

**`DROPPED_ROBOTS` records what did not make it** and why: Franka Panda and UR5
both ship xacro only, and expanding xacro needs a ROS ament package index —
ROS Humble is installed on this host and still could not resolve
`package 'ur_description' not found` from a bare clone. `corpus.XACRO_NOTE`
writes down the workspace-build path for whoever needs it, so the option is a
decision rather than a rediscovery.

One deliberate behaviour change: a missing URDF now **skips with the path it
wanted** instead of falling back to the first `.urdf` in the repository.
Substituting a different robot and reporting it under the requested name is
worse than measuring nothing.

### Two robots the guards refused

The dexterous hand has **no joint gravity can load at any pose**, and fixture
(c) has a joint deliberately left welded. Both are excluded from the benchmark
with a named reason rather than contributing a zero that means nothing — which
is the guard doing its job, and also why the hand could not settle the
`armature_floor` question it was added for.

---

## 5. The three approved decisions

**Backend-dependent defaults.** `target_frequency` is now derived:
`control_rate / 6` for PhysX or MuJoCo alone, `control_rate / 12` for a
multi-backend asset. Both the chosen value and the cross-backend value are
recorded in the layer, with the basis string.

One deviation, and it is deliberate: the decision said `/6` for any single
backend, but **`/6` is the exact value the dt sweep measured Newton diverging
at**. Shipping it for `--backend newton` would mean shipping a default the
measurement says fails, so Newton alone keeps `/12` and the basis string says
why:

```
control_rate / 12, measured for newton alone (Newton diverged at /6, so it
keeps the cross-backend divisor)
```

**`--force-unlock`.** Ambiguous `[0,0]` joints are now `refused` at **error**
severity — the command exits non-zero — and the message names both the
consequence and the way out:

```
joint is locked at [0, 0] and a <limit> element demonstrably existed, so the
value is genuinely ambiguous and this rule will not guess it. **MuJoCo refuses
to compile an asset containing a [0, 0] joint** ("range[0] should be smaller
than range[1]"), so the asset is unusable there until this is resolved. Either
fix the <limit> in the URDF, or pass --force-unlock ...
```

With the flag, the joints unlock, the records are marked `forced: true` with
`confidence: low`, and the reason says the range is a guess.

**`energy_drift` removed.** No backend adapter ever produced an energy series,
so it was a metric that looked measured and never was. Removed from
`metrics.py`, from `Trajectory`, and from its tests. The velocity trigger in
`diverged()` covers the case it was meant to catch, and a comment records why
it went.

---

## 6. Upstream issues: filed

All three are filed:

| # | Issue | Target |
|---|---|---|
| 1 | https://github.com/isaac-sim/IsaacSim/issues/841 | `isaac-sim/IsaacSim` |
| 2 | https://github.com/isaac-sim/IsaacSim/issues/842 | `isaac-sim/IsaacSim` |
| 3 | https://github.com/newton-physics/newton/issues/4269 | `newton-physics/newton` |

They were filed by the project owner rather than from this machine, which has
no `gh` and no GitHub credentials — posting to a public tracker under someone's
account is not something to improvise. The tooling that prepared them stays in
the repo for the next one:

* `docs/UPSTREAM_ISSUES.md` opens with a **Filing status** table — target repo,
  title, status, URL — now carrying all three issue links.
* `scripts/file_upstream_issues.py` splits the document into filable bodies,
  prints the exact `gh issue create` commands, and files them with `--file`.
* `tests/unit/test_upstream_issues.py` fails if the table and the document
  drift apart, if a row claims to be filed without a URL, or if the
  "source reading only, NOT reproduced" label on issue 2b is ever dropped.

To file the next one:

```bash
gh auth login
python scripts/file_upstream_issues.py            # review the bodies first
python scripts/file_upstream_issues.py --file
# then paste the URLs into the Filing status table
```

---

## 7. Usability

| Deliverable | Where |
|---|---|
| README with measured claims, a benchmark table and honest limits | `README.md` |
| Quickstart | `README.md` §Quickstart — three commands |
| Examples | `examples/` — 3 scripts, graded by what they need |
| Benchmark table | `README.md` — 14 robots × 3 backends, with the bad rows left in |
| CONTRIBUTING | `CONTRIBUTING.md` — setup, style, and the rule that a repair must earn its place |
| API docs | `docs/API.md` — CLI, Python API, rules, output layout, unit table |

The README states its versions, separates **measured** from **unmeasured**, and
has an "Honest limits" section naming the Newton failure, the overshoot
regression, the MuJoCo compile refusal and the three unimplemented gaps.
Examples 1 and 2 were run end to end; example 3 needs the GPU and shares the
harness code the benchmark exercised.

---

## 8. What is still open

1. **Newton on serial chains** — narrowed, not solved, and now upstream's to
   look at. It caps the cross-backend claim at 9 of 14 robots.
2. **`armature_floor` is still unmeasured.** The dexterous hand was added to
   settle it and the gravity guard refused the robot, so the case the constant
   exists for remains untested.
3. **`limits.compliance`** wants a calibration pass, not a static repair (§3).
4. **G4, G5, G6** — collision filtering, physics materials, scene defaults.
   Still untouched, and the drop suite currently measures little else.
5. **The corpus is two sources.** Twelve robots from SO-ARM100 and unitree_ros
   is better than two, but it is not diverse provenance, and every URDF from
   one vendor shares that vendor's conventions.
