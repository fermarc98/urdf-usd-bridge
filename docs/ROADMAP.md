# Roadmap

What v0.1.0 does not do, why, and what would settle each item. Nothing here is
a promise of a date. Items are ordered by how much they limit the claim the
project actually makes.

The gap numbering (`G1`…`G7`) is the one from
[`docs/ANALYSIS.md`](ANALYSIS.md) §4.

## Where v0.1.0 stands

| Gap | State |
|---|---|
| G1 drives | **done** — gains derived per backend, in each one's units |
| G2 inertia | **done** — derived, de-zeroed, made physical |
| G3 armature | **done**, with one constant unmeasured (below) |
| G7 joint limits | **partly** — restored where safe, refused where ambiguous, compliance still report-only (below) |
| G4 collision | **not started** |
| G5 physics materials | **not started** |
| G6 scene and unit assertions | **not started** |

---

## 1. `armature_floor` is unmeasured

The only tuning constant that ships without a measurement behind it. It is the
`β` in `max(α·I_eq, β·I_eq_max)`, and it exists for the case where a DOF's own
equivalent inertia is negligible — a fingertip, a light wrist roll — so the
fraction term gives a number too small to regularise anything.

**Why it is still unmeasured.** A dexterous hand was added to the corpus
specifically to exercise it, and the gravity guard refused the robot: no pose
loads any of its joints, so hold-pose drift there measures nothing and the
guard is right to say so.

**What would settle it.** A robot with genuinely low-inertia DOFs that *are*
gravity-loadable — a light wrist or a long-fingered hand mounted on an arm, so
the fingers hang. Then sweep `β` the way `α` was swept and look for a
divergence-threshold shift. If none appears, `β` should be deleted rather than
kept at an arbitrary value, which is the outcome the `α` sweep already produced
for the fraction term.

## 2. `limits.compliance` needs a calibration pass, not a repair rule

Phase 4 measured the case *for* doing something: a repaired, driven joint
overshoots its stop harder than the undriven baseline in 6 of 20 cells, worst
0.046 → 0.328 rad. PhysX treats a stop as rigid; MuJoCo derives a soft one from
`solreflimit`; so the same asset behaves differently at its limits in each.

**Why it is not a static attribute.** `newton:limitStiffness` is an effort per
unit penetration, and converting MuJoCo's `solreflimit` into it requires the
effective inertia at the joint — a *pose-dependent* quantity. Baking one value
into the asset makes it wrong at every other pose.
`mujoco-usd-converter` declines it for the same reason (`joint.py:99-104`), and
a sweep would only find a value that overfits one robot at one pose.

**What would settle it.** A calibration mode: drive each joint into its stop
under the harness, measure the penetration each backend produces, and solve for
the per-joint stiffness that matches them — writing the result as a *measured*
per-asset attribute with the pose it was measured at recorded alongside. That
is a new kind of output (a measurement, not a derivation) and needs its own
design.

## 3. G4 — collision geometry and pair filtering

Every collider the converter emits is a `convexHull`, nothing authors
`physics:filteredPairs` or collision groups, and self-collision is one global
boolean. A concave gripper jaw becomes a hull that intersects its neighbour at
the home pose, and the robot pops apart on step 0.

**This is the single largest hole**, and it is why the drop suite's numbers are
the weakest in the benchmark: they are dominated by contact behaviour the
project does not touch, so they measure the backends' contact defaults more
than they measure any repair.

**What it needs.** Detection first — step once, report non-adjacent colliders
in contact at rest — then filtering for the pairs that are structurally
guaranteed to clash. Automatic re-approximation (hull → SDF or convex
decomposition) is a bigger change and should follow the detector, not precede
it.

## 4. G5 — physics materials

No `UsdPhysics.MaterialAPI` is ever authored, so each backend applies its own
default friction (PhysX ≈ 0.5, MuJoCo 1.0). A wheeled robot that drives in one
backend spins its wheels in the other, and that reads as a broken controller
rather than an underspecified asset.

URDF has no standard friction field, so there is nothing to derive from — which
makes this a *reporting* problem first (say loudly that friction is
unspecified and therefore backend-dependent) and only then a question of
whether a default is worth authoring. Authoring one would be inventing physics,
which this project does not do without a measurement.

## 5. G6 — scene defaults and a unit assertion

Nothing authors a `UsdPhysics.Scene`, so gravity, timestep and solver settings
come from whatever host opens the asset: the same file is stable at 240 Hz and
explodes at 60 Hz, with nothing in it to warn you. The derived gains already
*assume* a control rate, and that assumption is currently recorded only in
`customLayerData`.

Separately, degrees-vs-radians is a demonstrated, recurring bug class at every
boundary — it is one of the three issues filed upstream. A converter that does
not assert its conventions will keep regressing them.
`tests/unit/test_repair_units.py` asserts ours; nothing asserts the asset's.

## 6. Corpus breadth

Twelve real robots, from **two** repositories (SO-ARM100 and unitree_ros). That
is enough to show the morphology split, and not enough to claim generality:
every URDF from one vendor shares that vendor's conventions, and a repair tuned
against those conventions can look universal while being parochial.

**What it needs.** Robots from different origins — a Franka, a UR, an ABB, a
mobile base — which mostly means solving the xacro problem. Franka and UR both
ship xacro only; expanding it needs an ament package index, and a bare clone
plus a ROS install was not enough here (`package 'ur_description' not found`).
`corpus.XACRO_NOTE` records the workspace-build path for whoever takes it on,
and `DROPPED_ROBOTS` records every robot that did not make it and why.

## 7. Newton on serial chains

`SolverFeatherstone` diverges on serial arms and humanoids where `SolverMuJoCo`,
on the identical `Model`, does not. Five hypotheses were ruled out by
measurement — our gains, our repairs at all, timestep, the 1e-9 kg marker link,
mass-ratio conditioning — and it reproduces identically on Newton 1.5.0 and
1.6.0.

It is filed as
[newton#4269](https://github.com/newton-physics/newton/issues/4269) and is
upstream's to explain. On this side, the thing to do is **re-run the corpus on
each Newton release** and record the result, so the day it changes is a
measurement and not a surprise. That caps the cross-backend claim at 9 of 14
robots until it moves.

## 8. Platforms

`inspect` and `fix` are pure `pxr` and the wheel is `py3-none-any`, so they
*should* run anywhere OpenUSD does. For v0.1.0 that is verified on Linux only.

| Platform | State |
|---|---|
| Linux x86-64 | verified: all tiers, T0 through T4 |
| macOS | **untested.** `[core]` resolves (`usd-core` and `numpy` publish macOS wheels); `[convert]` cannot, because `usd-exchange` publishes none. Needs a machine, not a change |
| Windows | **untested.** Both extras resolve. Same situation |

---

## Not planned

Stating these so nobody waits for them.

- **A GUI, or a Kit extension.** This is a library and a CLI.
- **Authoring values that cannot be derived or measured.** A friction
  coefficient URDF never specified will be reported, not invented.
- **Vendoring or forking `urdf-usd-converter`.** The repairs are an overlay on
  its output on purpose: it stays replaceable, and its bugs stay visible and
  filable rather than absorbed.
- **Supporting MJCF or SDF input.** The URDF→USD path is not finished yet.
