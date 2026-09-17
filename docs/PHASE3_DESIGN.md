# Phase 3 design — the stability layer, first slice

**Date:** 2026-09-18
**Status:** proposal. No library code written yet, per the Phase 3 brief.
**Scope:** G1 (drives), G3 (armature), G2 (inertia), G7 (joint limits). Nothing
else — no collision filtering, no physics materials, no `PhysicsScene` defaults.
**Reviewer decisions needed:** §9. Four of them change the shape of the code.

Every upstream claim below is cited to a file and line in `references/`, to an
installed schema definition, or to a value observed on this machine in Phase 2.5.
Where something is an engineering choice rather than a fact, it says so and gives
the default and the reasoning.

---

## 1. What `fix` is, in one paragraph

`fix` opens a converted robot, works out what each joint and body *should* carry,
and writes those values into **new layers that sit above the original asset**. The
original files are never opened for write. The result is a new root layer that
composes the untouched original plus our repairs, and a machine-readable report
saying what changed, from what, to what, and why. Muting our layers restores the
input exactly.

---

## 2. Output shape and the non-destructive mechanism

### 2.1 Verified mechanism

The mechanism is a new root layer whose `subLayerPaths` are our stability layers
followed by the original root. Earlier sublayers are stronger, so our `over`
opinions win; the original is referenced by relative path and never written.

I verified this on this machine against both layouts we support, using the real
assets from Phase 2.5:

| Layout | Asset | Over composed? | Original byte-identical after? |
|---|---|---|---|
| `newton-atomic` | `tests/_artifacts/0.3.2/a_dynamics_damping/` | yes | yes |
| `isaac-package` | Isaac Sim 6.1.0 stock import of fixture (a) | yes | yes |

**One trap, found during that check and worth stating loudly:** `defaultPrim`,
`upAxis`, `metersPerUnit` and `kilogramsPerUnit` are *root-layer metadata*, not
composed from sublayers. A naive new root layer opens with `defaultPrim = None`,
which silently disables every variant selection and makes the Isaac package
compose as a geometry-only asset. Our root layer must copy all four from the
input. First attempt got this wrong and the asset looked empty; the fix is four
lines and a test.

### 2.2 Proposed output

```
<out>/
  <name>_stabilized.usda      # new root: subLayers + defaultPrim + stage metrics
  Stability.usda              # backend-neutral repairs
  Stability_physx.usda        # PhysX-only opinions      (--backend physx|all)
  Stability_mujoco.usda       # MuJoCo-only opinions     (--backend mujoco|all)
  Stability_newton.usda       # Newton actuators         (--backend newton, opt-in)
  stability_report.json       # the full repair record
```

`Stability.usda` carries everything that is true regardless of backend: repaired
mass and inertia, repaired joint limits, `newton:*` passive dynamics, and
`newton:armature`. The per-backend layers carry only what that backend spells
differently.

### 2.3 Why the split is not optional

I tested the obvious simpler design — one layer with everything in it — against
an Isaac 6.1.0 package, and it produces a wrong asset:

```
physx     applied=['PhysicsDriveAPI:angular']  stiffness=12.5  MjcActuator prims: 0
mujoco    applied=['PhysicsDriveAPI:angular']  stiffness=12.5  MjcActuator prims: 2
physics   applied=['PhysicsDriveAPI:angular']  stiffness=12.5  MjcActuator prims: 0
```

Isaac's `mujoco.usda` deliberately **deletes** `PhysicsDriveAPI` and
`PhysicsJointStateAPI` (`isaacsim_structure.json`, the *Delete Physics Drive and
Joint State APIs* rule) precisely so MuJoCo actuation comes only from
`MjcActuator`. A root-level sublayer is stronger than that deletion, so a single
combined layer resurrects the PhysX drive inside the `mujoco` variant, and the
joint ends up with both a per-degree `DriveAPI` gain **and** a per-radian
`MjcActuator` gain. Whichever the consumer reads, one of them is wrong by 57.3×.

There is a second, independent reason, which §4.2 works through: PhysX has
nowhere to put passive joint damping *except* the drive, so the PhysX drive
damping must equal `d_passive + d_drive`, while Newton and MuJoCo keep the two
terms separate. Those are different numbers for the same conceptual quantity.
They cannot share a layer.

**Two ways to wire the per-backend layers. This is decision D1 (§9).**

- **D1-a, flat (simpler):** all layers are root sublayers. Correct for the
  `newton-atomic` layout, which has no variant set. On an Isaac package it still
  leaks PhysX drives into the `mujoco` variant, so `--backend all` carries a
  documented caveat and `--backend <one>` is the clean path.
- **D1-b, variant-scoped (recommended):** when the asset has a `Physics` variant
  set, author each backend's opinions inside the matching variant in our root
  layer; fall back to flat when there is no variant set. `--backend all` is then
  correct on both layouts. Costs roughly one `GetVariantSet(...).GetVariantEditContext()`
  wrapper and a test per layout.

I recommend **D1-b**, because `--backend all` is the mode that makes the
cross-backend-agreement claim testable in Phase 4, and under D1-a that mode is
knowingly wrong on the layout Isaac users actually have.

---

## 3. Units: the table the whole phase turns on

Degrees-vs-radians drift is the recurring bug class in `docs/ANALYSIS.md` §G6,
and Phase 2.5 confirmed one instance of it at runtime. Every number we author
goes through one conversion table, stated here, asserted in code, and tested.

| Quantity | Attribute | Angular convention | Authority |
|---|---|---|---|
| Passive damping (neutral/Newton) | `newton:damping` | **per degree** | `newton-usd-schemas` 0.5.0 `generatedSchema.usda`: *"Units: effort \* seconds / degrees (angular DOFs)"* |
| Passive damping (MuJoCo) | `mjc:damping` | **per radian** | `mujoco-usd-converter` `joint.py:83` writes MuJoCo's native value unscaled; `joint.py:97,108` scales the *Newton* copy by π/180 |
| Drive stiffness / damping | `UsdPhysics.DriveAPI` | **per degree** | `gain_tuner_drive_math.py` module docstring; `asset_utils.py` `_set_stiffness_on_joints` multiplies by π/180 |
| MuJoCo actuator gains | `mjc:gainPrm`, `mjc:biasPrm` | **per radian** | mirrors MJCF `gainprm`/`biasprm` (`omni.usd.schema.mujoco` `generatedSchema.usda:434`) |
| Newton actuator gains | `newton:kp`, `newton:kd` | **per radian** | `NewtonActuator` doc: *"Newton actuators use radians, diverging from `UsdPhysicsDriveAPI` which uses degrees"* |
| Armature | `newton:armature`, `mjc:armature`, `physxJoint:armature` | **not angle-scaled** — kg·m² | `newton:armature` doc: *"Units: mass \* distance \* distance (angular DOFs)"*; `mujoco-usd-converter` `joint.py:82,96` writes the same number to both |
| Coulomb friction | `newton:friction`, `mjc:frictionloss`, `physxJoint:jointFriction` | **not angle-scaled** — N·m | `newton:friction` doc: *"Units: effort"* |
| Limit stiffness / damping | `newton:limitStiffness`, `newton:limitDamping` | **per degree** | `newton-usd-schemas` docs, same file |
| Joint limits, targets | `physics:lowerLimit`, `drive:*:physics:targetPosition` | **degrees** | `UsdPhysics` |

Everything is computed once in SI (radians, metres, kilograms) and converted on
the way out, exactly once, in `model/units.py`. `PER_RADIAN_TO_PER_DEGREE`
already exists there from Phase 2.

### 3.1 Two upstream unit bugs this table exposes

Both are in Isaac Sim 6.1.0's `urdf_to_mjc_physx_conversion_utils.py`, both are
consequences of copying a stored USD value straight into an MJC attribute:

1. `create_mjc_actuator_from_physics` sets `gain_prm = [stiffness, 0, …]` and
   `bias_prm = [0, -stiffness, -damping, …]` (lines 282–283) from the
   **per-degree** `DriveAPI` gains into **per-radian** MJCF gain slots. A drive
   tuned correctly for PhysX is 57.3× too soft in MuJoCo.
2. `convert_physx_to_mjc` sets `mjc:ref` from `targetPosition` (line 337), USD
   degrees into MuJoCo radians.

Neither fires today in the default URDF path, because G1 means stiffness and
damping are absent and the function takes its `else` branch. **They become live
the moment anyone — including us — populates the PhysX drive.** So our MuJoCo
authoring must write `MjcActuator` gains itself, in radians, and must not rely on
Isaac's converter to derive them. This is a design constraint, not a footnote.

I have not filed this upstream; it belongs in the same issue as G1, and §11 of
`docs/VERIFY.md` already lists what to attach.

---

## 4. The repair rules

Common vocabulary for every rule below:

- **Trigger** — the exact state that makes the rule fire.
- **Confidence** — `high`: the repaired value is the only defensible one (an
  identity quaternion for an absent rotation). `medium`: derived from a
  documented model with a stated assumption (drive gains from a target
  frequency). `low`: a judgement call where the input is genuinely ambiguous;
  these are **report-only by default** and need an explicit opt-in.
- **Authored-value policy** — no rule overwrites a value the input authored and
  that passes validation. It records a `skipped` entry instead. `--force`
  overrides, and every forced change is marked `forced: true` in the report.

### 4.0 The articulation model these rules need

Three rules need the *equivalent inertia about a joint axis*, `I_eq`. Computed
from the composed stage:

1. Build the joint graph from each joint's `physics:body0` / `physics:body1`
   relationships, rooted at the prim carrying `PhysicsArticulationRootAPI`.
2. For joint *j*, collect every body in the subtree below it.
3. `I_eq(j) = Σ_i ( a·I_i·a + m_i · d_i² )` where `a` is the joint axis in world
   space, `I_i` is body *i*'s inertia tensor in world orientation, `m_i` its
   mass, and `d_i` the perpendicular distance from the joint axis to body *i*'s
   centre of mass. This is the standard parallel-axis sum, evaluated at the
   asset's default pose.
4. Prismatic joints use `m_eq(j) = Σ_i m_i` instead.

Assumptions, stated because they matter: this is the inertia at the **default
pose**, it ignores the coupling terms an articulated mass matrix would carry, and
it is therefore a lower bound on what the joint really sees through a trajectory.
That is the right direction to be wrong in for gain selection — it yields
conservative (softer) gains. Isaac's own gain tuner takes an `m_eq` scalar the
same way (`gain_tuner_drive_math.py:122-140`).

`I_eq` is computed from the **repaired** inertia values, so the inertia rules run
before the drive rules. Rule order is fixed and documented: inertia → limits →
armature → drives.

---

### 4.1 Inertia (G2)

Four rules, all writing to `Stability.usda`.

#### `inertia.principal-axes-identity`

- **Trigger:** `physics:principalAxes` is authored and its norm is zero
  `(0,0,0,0)`.
- **Repair:** write the identity quaternion `(1, 0, 0, 0)`.
- **Confidence:** `high`. A zero quaternion has no meaning; the attribute's own
  schema fallback is `(0,0,0,0)`, and the only interpretation of "no rotation
  information" is identity.
- **Why it exists:** this is the converter 0.3.2 bug that Isaac Sim 6.1.0 ships,
  confirmed behaviourally in Phase 2.5 — fixture (b) produced
  `principalAxes = (0,0,0,0)` with `authored: true` on both no-inertia links
  under 0.3.2, and `authored: false` under 0.3.3. It is the single cheapest,
  most demonstrable win in this phase.
- **Note:** on 0.3.3 output the attribute is *unauthored*, so this rule does not
  fire; the fallback is the same zero quaternion, and
  `inertia.derive-from-geometry` authors a real one.

#### `inertia.derive-from-geometry`

- **Trigger:** `physics:mass > 0` and `physics:diagonalInertia` is absent, or
  authored as `(0,0,0)`.
- **Repair:** compute the tensor from the body's collision geometry at uniform
  density, using the analytic tensor for `Cube`, `Sphere`, `Cylinder`, `Capsule`
  and `Cone`, and the oriented bounding box of the points for `Mesh`. Scale the
  result so its total equals the authored `physics:mass`. Write
  `physics:diagonalInertia`, `physics:principalAxes`, and `newton:inertia`
  (the 6-vector `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`) consistently.
- **Fallback when the body has no collision geometry at all:** a solid sphere of
  the same mass whose radius is 1% of the articulation's overall bounding-box
  diagonal, i.e. `I = 2/5·m·r²`. Confidence `low`, report-only by default.
- **Confidence:** `medium` with geometry, `low` without.
- **Why not leave it to PhysX:** PhysX auto-computes inertia from colliders when
  the tensor is zero, which quietly makes PhysX simulate a different robot from
  MuJoCo (which errors or substitutes its own) and Newton (which takes the zeros
  and produces a singular spatial inertia). Authoring one tensor into the asset
  is what makes the three agree.

#### `inertia.make-physical`

- **Trigger:** any diagonal entry is negative, or the triangle inequality
  `I₁ + I₂ ≥ I₃` fails on any permutation.
- **Repair:** clamp negatives to zero, then, if the triangle inequality still
  fails, raise the two smaller entries by the minimum amount that satisfies it.
  Never lower the largest entry: raising inertia is always the conservative,
  stability-increasing direction.
- **Confidence:** `medium`. The repair is minimal in the sense that it is the
  smallest change that makes the tensor physical, but the input was
  self-contradictory, so no repair is uniquely correct.

#### `inertia.mass-floor`

- **Trigger:** a rigid body has `physics:mass` absent or `≤ 0` **and** no
  `physics:density` from which one could be derived.
- **Repair:** report only. Default is **not** to invent a mass.
- **Confidence:** `low`, and this is deliberate. A fabricated mass changes the
  robot's dynamics everywhere; a reported one makes the user fix their URDF.
  `--enable inertia.mass-floor` opts in to a density-based estimate at
  1000 kg/m³.

**Mass ratio** is diagnosed (`summary.mass_ratio_max`, already in `inspect`) and
reported, never repaired. The fix for a bad mass ratio is solver iteration counts
or model surgery, both out of scope for this slice.

---

### 4.2 Drives (G1)

This is the rule set the phase exists for. It has two halves that are often
confused, and keeping them apart is most of the design.

**Half one — mirror what the URDF said.** The URDF authored
`<dynamics damping="1.5" friction="0.3">`. The converter put it in
`newton:damping` / `newton:friction`. Isaac reads a spelling that no longer
exists, so PhysX and MuJoCo see nothing. Mirroring is a *lossless translation*,
not an invention.

**Half two — supply actuation the URDF never had.** Neither converter authors any
`DriveAPI` gains at all, so the robot has no way to hold a pose. This half is a
genuine addition and is therefore opt-out-able and fully logged.

#### `drives.mirror-passive`

- **Trigger:** `newton:damping` or `newton:friction` is authored on a revolute or
  prismatic joint, and the corresponding per-backend attribute is absent.
- **Repair, per backend:**

  | Backend | Attribute | Value | Layer |
  |---|---|---|---|
  | MuJoCo | `mjc:damping` | `newton:damping × 180/π` (angular), unchanged (linear) | `Stability_mujoco.usda` |
  | MuJoCo | `mjc:frictionloss` | `newton:friction` unchanged | `Stability_mujoco.usda` |
  | PhysX | `physxJoint:jointFriction` | `newton:friction` unchanged | `Stability_physx.usda` |
  | PhysX | drive damping | folded into `drives.derive-gains` below | `Stability_physx.usda` |

- **Confidence:** `high`. Every one of these is a unit conversion of a value the
  user authored, with the conventions cited in §3.
- **The PhysX asymmetry:** PhysX has no passive joint-damping attribute. Its only
  velocity-proportional term is the drive's damping. So on the PhysX path,
  passive damping must be folded into `DriveAPI.damping`; on the MuJoCo and
  Newton paths it stays a separate joint-level term. Same physics, different
  totals per attribute — which is exactly why §2.3 splits the layers.

#### `drives.derive-gains`

- **Trigger:** a revolute or prismatic joint has no drive stiffness authored
  (absent, or authored zero — which is what Isaac's `add_joint_schemas()` leaves
  behind).
- **Formula**, in SI, adapted from `gain_tuner_drive_math.py:122-140`:

  ```
  I_total = I_eq + armature                       # armature from §4.3, already applied
  K_si    = I_total · (2π·f_n)²
  D_si    = 2·ζ·√(I_total · K_si)                 #  = 2·ζ·(2π·f_n)·I_total
  ```

  with defaults **`f_n = 10 Hz`** and **`ζ = 1.0`**. Worked end to end for a joint
  with `I_eq = 0.0125 kg·m²` in an articulation whose `I_eq_max = 0.05`:

  ```
  armature = max(0.01·0.0125, 1e-4·0.05) = 1.25e-4      I_total = 0.012625
  K_si = 0.012625 · (2π·10)²  = 49.8415  N·m/rad   ->  0.869898  N·m/deg
  D_si = 2·1.0·(2π·10)·0.012625 = 1.5865 N·m·s/rad ->  0.027690  N·m·s/deg
  ```
- **Authored per backend:**

  | Backend | Attribute | Value |
  |---|---|---|
  | PhysX | `drive:<angular\|linear>:physics:stiffness` | `K_si × π/180` (angular) |
  | PhysX | `drive:<angular\|linear>:physics:damping` | `(D_si + d_passive_si) × π/180` |
  | MuJoCo | `mjc:gainPrm` | `[K_si, 0, …]`, `gainType = "fixed"` |
  | MuJoCo | `mjc:biasPrm` | `[0, -K_si, -D_si, …]`, `biasType = "affine"` |
  | Newton | `newton:kp` / `newton:kd` on a `NewtonActuator` | `K_si` / `D_si`, radians |

  The MJC encoding is the position-control form documented in
  `urdf_to_mjc_physx_conversion_utils.py:280-287`; we write it in radians, which
  is where that function has its bug (§3.1). MuJoCo's passive `mjc:damping`
  already carries `d_passive`, so the actuator carries `D_si` alone.
- **`maxForce`:** set from `urdf:limit:effort` when present, which Phase 2.5
  confirmed is the one URDF attribute whose spelling still reaches Isaac
  (fixture (a): `maxForce = 87.0` came through on the stock import). Absent
  effort → leave `maxForce` unauthored rather than inventing a torque limit.
- **`targetPosition`:** set to the joint's current position, i.e. zero, so the
  drive holds the asset's authored pose rather than dragging it somewhere new.
- **Confidence:** `medium`. The model is a second-order system at the default
  pose; the gains are not tuned for any particular task.
- **Choice of `f_n = 10 Hz`, stated as a choice:** 10 Hz is roughly a sixth of a
  60 Hz control rate, which leaves margin for the discrete-time phase lag that
  makes stiff PD loops explode, and it is stiff enough to hold a robot arm
  against gravity. `ζ = 1.0` is critical damping — no overshoot, the safe default
  for an asset that will be loaded by someone who has not tuned anything. Both
  are `--target-frequency` / `--damping-ratio`. **Neither is measured yet**; §10
  is the Phase 4 experiment that has to justify or move them.

#### `drives.no-drive-for-fixed`

Fixed, spherical and generic joints get no drive and no armature. Stated so the
report can show them as deliberately skipped rather than missed.

---

### 4.3 Armature (G3)

One rule, writing `newton:armature` to `Stability.usda` and the per-backend
spellings to their own layers.

- **Trigger:** a revolute or prismatic joint has no armature authored in any
  namespace.
- **Repair:**

  ```
  armature = max( α · I_eq ,  β · I_eq_max )
  ```

  where `I_eq_max` is the largest equivalent inertia anywhere in the same
  articulation. Defaults: **`α = 0.01`**, **`β = 1e-4`**.

  Two terms, two jobs. `α · I_eq` is the scale-relative rotor term: 1% of what
  the DOF already carries, small enough not to change the robot's gross
  behaviour. `β · I_eq_max` is a **conditioning floor**: it bounds the ratio
  between the largest and smallest diagonal entry of the joint-space mass matrix
  at roughly `1/β`, which is what stops an iterative solver from stalling on the
  light end of a chain. Worked through:

  | DOF | `I_eq` | armature | as a multiple of `I_eq` |
  |---|---|---|---|
  | arm link, `I_eq_max = 0.05` | 0.05 | 5.0e-4 | 0.01 — the rotor term wins |
  | gripper fingertip, same arm | 1e-6 | 5.0e-6 | 5 — the floor wins, by 500× |

  The second row is the case that matters: a fingertip's own 1% would be
  1e-8 kg·m², which is no regularisation at all.

  **An earlier draft of this rule derived the floor from the control rate**, as
  the extra inertia needed to keep the drive's natural frequency under
  `control_rate/4`. That is circular and I dropped it: `drives.derive-gains`
  computes `K` *from* `I_total`, so ω_n comes out at `2π·f_n` by construction no
  matter what armature is. The control rate still appears, but as a **check**
  rather than an input — `fix` emits a `warning` record when the requested
  `f_n` exceeds `control_rate / 4`, because that is a gain choice the integrator
  will not survive, and no amount of armature changes it.
- **Authored:** `newton:armature`, `mjc:armature`, `physxJoint:armature` — all
  three take the **same** number, unscaled, per §3.
- **Confidence:** `medium`.
- **Why a rule at all, and why this one.** MuJoCo's own schema documentation says
  it plainly: *"positive armature significantly improves simulation stability,
  even for small values, and is a recommended possible fix when encountering
  stability issues"* (`omni.usd.schema.mujoco` `generatedSchema.usda:513`). The
  URDF path authors none anywhere — `convert_physx_to_mjc` will copy
  `physxJoint:armature` to `mjc:armature`, but nothing in the import ever sets
  it, so it is always absent.
- **The honest part:** both terms are conventions, not derivations. `α = 0.01` is
  chosen at the low end of what MuJoCo Menagerie models carry hand-tuned;
  `β = 1e-4` bounds the mass-matrix diagonal ratio at about 1e4, which is the
  order at which iterative solvers start failing to converge in a default
  iteration budget (`docs/ANALYSIS.md` G2). Neither is measured. What *is*
  principled is the shape — scale-relative with a floor relative to the whole
  articulation, rather than one absolute constant — because an absolute armature
  is wrong for any robot that is not the size it was tuned on. The report records
  which of the two terms won for every joint, so Phase 4 can move either of them
  on evidence.
- **What we do not do:** `<transmission>` / `mechanicalReduction` would give the
  physically correct `n²·I_rotor`, but `urdf-usd-converter` drops `<transmission>`
  entirely (`concept_mapping.md`), so it is not in the USD for us to read.
  Recovering it means re-reading the source URDF alongside the asset. That is a
  good feature and it is **not in this slice**; the report emits an
  `information` record naming the joints where it would have helped.

---

### 4.4 Joint limits (G7)

The converter turns a revolute joint with no `<limit>` into a joint with
`lower = upper = 0` — welded shut, silently (`link.py:392-393`). Phase 2.5
confirmed this at runtime on fixture (c), on both 0.3.2 and 0.3.3.

The hard part is not detecting `[0,0]`; it is deciding whether `[0,0]` was
*meant*. A URDF may legitimately say `lower="0" upper="0"`. We can tell the two
cases apart from the asset alone, using evidence Phase 2.5 confirmed is present:

| Evidence on a `[0,0]` revolute joint | Reading | Rule |
|---|---|---|
| no `urdf:limit:effort`, no `newton:velocityLimit` | `<limit>` was **absent entirely** — URDF requires `effort` and `velocity` on it, so neither could be recorded | `limits.restore-missing` |
| `urdf:limit:effort` and/or `newton:velocityLimit` present | `<limit>` existed; `lower`/`upper` were omitted or genuinely zero | `limits.report-ambiguous` |

This is exactly what fixture (c) shows: `no_limit_joint` has no effort and no
velocity limit, `partial_limit_joint` has `effort = 30.0` and
`velocityLimit = 85.94`, and both came out `[0,0]`.

#### `limits.restore-missing`

- **Trigger:** revolute or prismatic, `lower == upper == 0`, and no
  `urdf:limit:effort` and no `newton:velocityLimit`.
- **Repair:** author the joint as **unlimited** — remove the limit by writing
  `lower = -inf`, `upper = +inf`, which is how the converter already represents a
  `continuous` joint (fixture (c)'s `continuous_joint` reads exactly that).
- **Confidence:** `medium`. A revolute joint with no `<limit>` is malformed URDF;
  the nearest well-defined URDF concept is `continuous`, and an unlimited joint is
  strictly closer to any plausible intent than a welded one.
- **Prismatic caveat:** an unlimited prismatic joint is a sliding rail with no
  stops, which can be worse than a welded one. For prismatic joints this rule is
  **report-only by default**, opt-in via `--enable limits.restore-missing-prismatic`.

#### `limits.report-ambiguous`

- **Trigger:** `[0,0]` with limit evidence present.
- **Repair:** none. Emits a `warning` record naming the joint, the evidence, and
  the two possible readings.
- **Confidence:** n/a — this rule exists to refuse to guess.

#### `limits.compliance`

- **Trigger:** joint has finite limits and no `newton:limitStiffness` /
  `newton:limitDamping`.
- **Repair:** none in this slice; report only.
- **Why not:** `mujoco-usd-converter` leaves these unauthored deliberately and
  documents why in `joint.py:99-104` — MJCF describes limits through
  `solreflimit` in MuJoCo's normalised constraint space, and converting that to
  Newton's effort-per-unit-penetration needs the compiled effective inertia at
  `qpos0`, which would bake a configuration-dependent number into a static
  attribute. They are right, and inventing a limit stiffness without a simulation
  to check it against would violate the "every rule justified by a measurement"
  rule. Revisit in Phase 4 with the limit-sweep suite.

---

## 5. The repair record

One record per rule evaluation, including the ones that did nothing. The brief
asks for prim path, attribute, old value, new value, reason, confidence; the rest
are there so the report is diffable and so a reader can reconstruct the
arithmetic without rerunning the tool.

```json
{
  "rule": "drives.derive-gains",
  "status": "applied",
  "prim": "/a_dynamics_damping/Physics/shoulder_joint",
  "attribute": "drive:angular:physics:stiffness",
  "old": null,
  "old_state": "absent",
  "new": 0.869898,
  "units": "N*m/deg",
  "backend": "physx",
  "layer": "Stability_physx.usda",
  "reason": "no drive stiffness authored; derived for f_n=10.0 Hz, zeta=1.0",
  "confidence": "medium",
  "evidence": {
    "I_eq": 0.0125,
    "armature": 0.000125,
    "I_total": 0.012625,
    "K_si": 49.8415,
    "formula": "K_si = I_total * (2*pi*f_n)**2; stored = K_si * pi/180"
  },
  "forced": false
}
```

- `status` ∈ `applied` | `skipped` | `reported` | `refused`.
- `old_state` ∈ `absent` | `fallback` | `authored` — the three-way distinction
  `inspect` already makes, carried through so "we overwrote an authored zero" and
  "we filled an absent value" never look the same in a diff.
- `severity` on report-only records ∈ `information` | `warning` | `error`.

The report also carries the input digest, the tool version, the rule-set version,
the full resolved option set, and the per-layout detection result. `--json` emits
it; the default renderer prints a table grouped by rule.

---

## 6. CLI

```
urdf-usd-bridge fix <usd> [--out DIR]
    [--backend physx|newton|mujoco|all]     default: all
    [--variant SET=SELECTION]               applied before reading
    [--enable RULE[,RULE...]] [--disable RULE[,RULE...]]
    [--target-frequency HZ]                 default: 10.0
    [--damping-ratio Z]                     default: 1.0
    [--armature-fraction A]                 default: 0.01   (alpha)
    [--armature-floor B]                    default: 1e-4   (beta)
    [--control-rate HZ]                     default: 60.0   (checked, not used in any formula)
    [--force]                               overwrite authored valid values
    [--dry-run]                             report only, write nothing
    [--json] [-o PATH]
```

Exit codes: `0` clean or all repairs applied; `1` an `error`-severity finding
remains after repair (e.g. a body with no mass and no density); `2` usage or a
missing dependency, matching the existing `inspect` behaviour.

`--dry-run` runs every rule and produces the identical report, minus the writes.
It is the same code path, with the layer targets swapped for an in-memory
anonymous layer, so a dry run cannot diverge from a real one.

### `convert`, extended (deliverable 4)

`convert <urdf> <out>` gains `--fix` / `--no-fix` (**default `--fix`**) plus the
tuning flags above, so the common case is:

```bash
urdf-usd-bridge convert robot.urdf out/          # convert + stabilize, one command
urdf-usd-bridge convert robot.urdf out/ --no-fix # Phase 2 behaviour
```

It stays a thin front-end: it calls `urdf_usd_converter.Converter` exactly as
today and then runs the same `fix` entry point on the result. No new conversion
logic.

---

## 7. Determinism

The brief asks for byte-identical output for identical input. What we can
honestly promise, and how:

- **Traversal order** is sorted by prim path, never `Stage.Traverse()` order.
- **Rule order** is fixed: inertia → limits → armature → drives. Written down,
  not emergent.
- **No wall-clock time, no absolute paths, no hostname** in any authored layer or
  in the report. Provenance is the input SHA-256, the tool version, and the
  rule-set version. `--stamp` opts into a timestamp for people who want one, and
  is off by default precisely because it would break this property.
- **Float formatting** is USD's own, and we write `float` vs `double` to match
  each attribute's declared type rather than letting Python's default pick.
- **Eigendecomposition is canonicalised.** `numpy.linalg.eigh` can return
  eigenvectors with arbitrary sign and, for degenerate eigenvalues, arbitrary
  basis rotation — across BLAS builds. We sort eigenvalues descending, fix each
  eigenvector's sign so its largest-magnitude component is positive, and snap
  near-degenerate cases to the identity when the tensor is isotropic within
  tolerance.
- **Scope of the promise:** byte-identical for the same input, the same tool
  version, and the same platform and dependency set. Tested by running `fix`
  twice and comparing bytes, and by a golden-layer snapshot test. I am
  deliberately *not* claiming byte-identical output across BLAS
  implementations — canonicalisation makes that very likely but I have not
  measured it, and claiming it without evidence is the kind of thing this project
  exists to complain about.

---

## 8. Tests (deliverable 5 and 6)

| Tier | What | Where |
|---|---|---|
| Unit | `I_eq` against hand-worked examples (point mass on a rod, two-link chain); PSD projection; triangle-inequality repair; unit round-trips asserting the §3 table in both directions | `tests/unit/` |
| Rule | Each rule fired and not-fired, on hand-authored stages, asserting the record fields as well as the authored value | `tests/unit/` |
| Golden | `Stability*.usda` text snapshots for each fixture × backend | `tests/golden/` |
| Round-trip | `fix` then `inspect`, asserting the gaps are closed: `joints_with_drive_gains == joints_actuatable`, `joints_drive_applied_without_gains == 0`, `bodies_invalid_principal_axes == 0`, `joints_locked_by_equal_limits == 0`, `armature_authored_by_namespace` non-empty | `tests/converter/` |
| Determinism | `fix` twice, compare bytes of every authored layer | `tests/unit/` |
| Non-destructive | SHA-256 of every input file before and after | `tests/unit/` |

### New fixture (d) — closing the convexHull question

Phase 2.5 left "mesh colliders are always `convexHull`" as the one §1.4 claim
still source-only, because fixtures (a)–(c) use `<box>` collision geometry and
the converter authors no `MeshCollisionAPI` for primitives.

`tests/fixtures/d_mesh_collision.urdf` adds a link whose `<collision>` is a
`<mesh>` pointing at `tests/fixtures/meshes/l_bracket.obj` — a deliberately
**concave** L-shape, hand-written by us, so the hull differs visibly from the
mesh. Controls: a second link with a `<box>` collision, and a visual-only link.
Assertions: `MeshCollisionAPI` is applied with `physics:approximation ==
"convexHull"` on both 0.3.2 and 0.3.3, and the box link carries no approximation.

This closes the claim behaviourally. It does **not** turn into a repair — G4 is
explicitly out of scope — but the fixture is what a G4 slice will need.

### Isaac-pinned column in the converter matrix

`scripts/run_converter_matrix.py` gains a third column that pins the versions
Isaac Sim 6.1.0 actually ships, so the matrix stops silently testing a newer
stack than the one users have:

| Column | `urdf-usd-converter` | `usd-exchange` | `newton-usd-schemas` |
|---|---|---|---|
| `0.3.2` | 0.3.2 | resolved (3.0.0 today) | resolved (0.5.0 today) |
| `0.3.3` | 0.3.3 | resolved | resolved |
| **`0.3.2-isaac`** | **0.3.2** | **2.3.0** | **0.4.1** |

I checked the pin resolves on this machine before proposing it:
`uv pip install --dry-run 'urdf-usd-converter==0.3.2' 'usd-exchange==2.3.0'
'newton-usd-schemas==0.4.1'` → *Resolved 11 packages*, no conflict.

---

## 9. Decisions I need from you

**D1 — per-backend layer wiring.** Flat root sublayers (simple, `--backend all`
knowingly wrong inside an Isaac `mujoco` variant) versus variant-scoped when a
`Physics` variant set exists (correct on both layouts, a little more code).
*Recommendation: variant-scoped.* §2.3.

**D2 — `--backend` default.** `all` makes the cross-backend claim testable and is
the point of the project. `physx` would be the conservative choice.
*Recommendation: `all`*, contingent on D1-b.

**D3 — `f_n = 10 Hz`, `ζ = 1.0`, `α = 0.01`, `β = 1e-4`.** These are
defensible but unmeasured; Phase 4 is what justifies them. Are you happy to ship
them as documented defaults now and move them on evidence later, or would you
rather `drives.derive-gains` be opt-in until it has been simulated?
*Recommendation: ship as defaults, clearly marked unmeasured in the report
header.*

**D4 — `limits.restore-missing` on revolute joints.** Repairing `[0,0]` to
unlimited changes the robot from welded to free. It is defensible (the input is
malformed URDF) but it is the single largest behavioural change in this phase.
Default on, or default report-only?
*Recommendation: on for revolute, report-only for prismatic*, as written in §4.4.

Two smaller ones I have taken myself and will flag in the report rather than ask
about: `targetPosition = 0` (hold the authored pose) and `maxForce` left
unauthored when `<limit effort>` is absent.

---

## 10. What Phase 4 has to measure

Every `medium`-confidence default in this document is a hypothesis. The hold-pose
and drop suites from `docs/ANALYSIS.md` §6.4 are what turn them into facts:

1. Does `f_n = 10 Hz`, `ζ = 1.0` hold a fixed-base arm against gravity, in all
   three backends, with drift below a stated threshold?
2. Does the armature floor actually buy stability margin, measured as the largest
   `dt` that survives hold-pose, compared with `armature = 0`?
3. Do the three backends now *agree*? Same final base height, same settle time,
   within a stated tolerance. This is the product claim and nothing before Phase 4
   tests it.
4. Does `inertia.derive-from-geometry` produce a robot that behaves like the one
   PhysX would have auto-computed, or meaningfully differently?

Until then, `docs/PHASE3_REPORT.md` will say for each rule whether it is verified
statically (the attribute is authored with the value we computed), behaviourally
(a converter or Isaac Sim round-trip confirms it survives), or **not at all**.

---

## 11. Licensing

Two adaptations are planned, both from Apache-2.0 NVIDIA files, both already
pre-registered in `THIRD_PARTY.md` in Phase 2:

| Upstream | Tag | What we adapt |
|---|---|---|
| `isaacsim.robot_setup.gain_tuner/.../gain_tuner_drive_math.py` | `v6.1.0` | the `(f_n, ζ, m_eq) → (K, D)` relations and the stored-gain scale, §4.2 |
| `isaacsim/asset/importer/utils/.../urdf_to_mjc_physx_conversion_utils.py` | `v6.1.0` | the MJC `gainPrm`/`biasPrm` position-control encoding, §4.2 — **with the unit bug fixed**, which is the modification to record |

Each adapted file keeps its original `SPDX-FileCopyrightText` header, gains a
`Modified by urdf-usd-bridge contributors` line, and gets a `THIRD_PARTY.md` row
naming the upstream path, tag `v6.1.0`, commit
`7c206f75bdadd9e05fc457f19863ca4c3f0cb693`, and the change. Nothing is copied
from a file carrying `LicenseRef-NvidiaProprietary`; neither of these does.
`references/` stays read-only.
