# How it works

What each repair does, and why it is the repair rather than a different one.
No formulas you have to take on faith: every constant here either has a
measurement behind it ([`BENCHMARK.md`](BENCHMARK.md)) or is labelled as not
having one.

For the CLI and the Python API, see [`API.md`](API.md). For the original
derivations, [`history/PHASE3_DESIGN.md`](history/PHASE3_DESIGN.md).

---

## The problem

A URDF describes a robot's *kinematics* completely and its *dynamics* barely.
It has link masses and inertias, and it has `<limit>` and `<dynamics>` elements
that are optional and routinely omitted. It has nothing at all about actuation:
no gains, no rotor inertia, no controller.

That is fine for a URDF, because in ROS the controller lives elsewhere. It is
not fine once the file becomes a physics asset, because a physics engine cannot
decline to have an opinion. Each one fills the holes differently:

- **Zero inertia tensor.** PhysX quietly computes one from the colliders.
  MuJoCo errors or substitutes its own. Newton takes the zeros and builds a
  singular spatial inertia.
- **No drive gains.** Nothing holds the pose. The arm falls over, which looks
  like a modelling error rather than a missing controller.
- **A joint with no `<limit>`.** The converter writes `[0, 0]`, which every
  backend reads as *welded shut*.

So the same file is three different robots. `urdf-usd-bridge` writes the
missing dynamics down explicitly, once, so that stops being true.

## The mechanism: layers, not edits

Every repair is authored as an `over` prim in a **new USD layer** that sublayers
your original. The input file is opened read-only and never written.

```
out/
  robot_stabilized.usda     subLayers = [Stability*.usda ..., <your original>]
  Stability.usda            backend-neutral: inertia, limits, passive dynamics
  Stability_physx.usda      PhysX drive gains and armature
  Stability_mujoco.usda     MjcActuator gains, mjc:* damping and armature
  Stability_newton.usda     what Newton actually reads
```

Earlier sublayers win. This is not a stylistic choice — it buys three things
that an in-place edit cannot:

1. **You can diff it.** The stability layer *is* the list of assumptions added
   to your robot, in a text file.
2. **You can mute it.** Mute the stability layers and the original composes
   back exactly, which is also how the benchmark measures "before".
3. **The backends can disagree.** They have to: the same physical gain is
   spelled differently and in different units for each, so one flat set of
   attributes cannot serve all three.

There is a fourth reason, specific to Isaac Sim. Its `mujoco.usda` variant
*deliberately deletes* `PhysicsDriveAPI`. An opinion authored in a flat root
sublayer would override that deletion and re-add a drive Isaac meant to remove,
so when the asset has a `Physics` variant set, per-backend opinions are
authored **inside the matching variant**.

## The unit table the whole thing turns on

One physical gain, three spellings, two angle conventions:

| Backend | Attribute | Convention |
|---|---|---|
| PhysX | `drive:angular:physics:stiffness` | per **degree** |
| MuJoCo | `mjc:gainPrm[0]` | per **radian** |
| Newton | `UsdPhysics.DriveAPI` (converts internally) | per **degree** |

Get it wrong and you are out by `180/π = 57.29578`. That is not hypothetical:
it is a live bug in shipping software
([`UPSTREAM_ISSUES.md`](UPSTREAM_ISSUES.md) issue 2, reproduced on Isaac Sim
6.1.0).

Everything is computed **once in SI**, then converted **once per backend**, in
`model/units.py`. Quantities that have no angle in them — armature, Coulomb
friction, `maxForce` — are *not* scaled, and `tests/unit/test_repair_units.py`
asserts both halves: the `180/π` ratio where it belongs, and exact equality
where it does not. Both directions of the mistake fail the suite.

## What one DOF has to move

Most rules need to know how much inertia a joint is actually working against.
The model is the equivalent inertia at the default pose: for each body in the
subtree below the joint, its own inertia about the joint axis plus a
parallel-axis term for how far its centre of mass sits from that axis.

```
I_eq = Σ over the subtree ( a·(Rᵢ Iᵢ Rᵢᵀ)·a  +  mᵢ dᵢ² )
```

`a` is the joint axis in world space and `dᵢ` the perpendicular distance from
it to body *i*'s centre of mass. Prismatic joints use the plain mass sum.

This deliberately **ignores articulated-body coupling**, so it under-estimates.
That is the conservative direction: a smaller `I_eq` yields softer gains, and
soft gains are recoverable where stiff ones are not.

---

## The rules

Twelve rules. Nine change something; three exist to report rather than guess.
Every one produces a record — including when it decides to do nothing — with
the old value, the new value, the reason and a confidence.

### Inertia

**`inertia.principal-axes-identity`** *(on, confidence: high)*
A zero quaternion `(0,0,0,0)` has no meaning as a rotation. It appears because
`urdf-usd-converter` 0.3.2 authors it for links whose URDF had no `<inertial>`
— a defect Isaac Sim 6.1.0 inherits by shipping that version. The only reading
of "no rotation information" is identity, so identity is what gets written.
0.3.3 leaves the attribute unauthored instead, in which case this rule does not
fire and the next one supplies a real orientation.

**`inertia.derive-from-geometry`** *(on, confidence: medium — low without geometry)*
When a body has mass but no usable inertia tensor, compute one from its
collision geometry at uniform density: the analytic tensor for `Cube`,
`Sphere`, `Cylinder`, `Capsule` and `Cone`, and the oriented bounding box for a
`Mesh`. Scale it so the total matches the authored mass.

Worth doing rather than leaving to the engine precisely *because* PhysX would
handle it silently — and differently from the other two. Writing one tensor
into the asset is what makes the three agree.

With no collision geometry at all, it falls back to a solid sphere of 1% of the
articulation's bounding-box diagonal. That is a guess, so it is `low`
confidence and report-only.

**`inertia.make-physical`** *(on, confidence: medium)*
A real inertia tensor satisfies the triangle inequality `I₁ + I₂ ≥ I₃` and has
no negative entries. When one does not, negatives are clamped to zero and the
two smaller entries are raised by the least amount that satisfies the
inequality.

**Always by raising, never by lowering the largest.** More inertia is the
stability-increasing direction; a robot that is slightly too heavy to spin is
recoverable, one that is too easy to spin is not.

**`inertia.mass-floor`** *(off by default, report-only)*
A body with no mass and no density is reported, not repaired. A fabricated mass
propagates into every gain, every armature and every contact response in the
robot. Reporting it makes you fix the URDF, which is where the problem is.
`--enable inertia.mass-floor` opts into a 1000 kg/m³ estimate if you want it.

Mass *ratio* is diagnosed and reported, never repaired: the fix for a bad one
is solver iterations or model surgery, neither of which belongs in an asset.

### Joint limits

The converter writes `[0, 0]` for a joint whose URDF had no `<limit>`, and
every backend reads that as welded. The hard part is not spotting `[0, 0]` —
it is knowing whether it was *meant*, because a URDF may legitimately say
`lower="0" upper="0"`.

It is decidable from the asset. URDF requires `effort` and `velocity` on a
`<limit>` element, so if the element existed at all, traces of it survive:

| Evidence on a `[0,0]` joint | Reading |
|---|---|
| no `urdf:limit:effort`, no `newton:velocityLimit` | the `<limit>` was absent entirely |
| either one present | the `<limit>` existed; the range is genuinely ambiguous |

**`limits.restore-missing`** *(on, confidence: medium)*
First case: author the joint unlimited (`±inf`), which is how the converter
already represents a `continuous` joint. A revolute joint with no `<limit>` is
malformed URDF, and `continuous` is the nearest well-defined concept — and
certainly closer to any plausible intent than welded.

**`limits.restore-missing-prismatic`** *(off by default)*
Same logic, except an unlimited prismatic joint is a rail with no stops, which
can be worse than a welded one. Opt in deliberately.

**`limits.report-ambiguous`** *(on, `error` severity)*
Second case: refuse, loudly. The command exits non-zero, and the message says
what this costs you — **MuJoCo will not compile an asset containing a `[0,0]`
joint** ("range[0] should be smaller than range[1]") — and names the way out.
`--force-unlock` unlocks them anyway, marking every record `forced: true` at
`low` confidence with the reason saying the range is a guess.

**`limits.compliance`** *(on, report-only)*
Reports that a joint has finite limits and no limit compliance, so PhysX treats
the stop as rigid while MuJoCo derives a soft one from `solreflimit`.

Still not repaired, and this is the most-measured refusal in the project.
Phase 4 measured the case *for* acting — a driven joint overshoots its stop
harder than the undriven baseline in 6 of 20 cells, worst 0.046 → 0.328 rad —
and it is still not derivable: `newton:limitStiffness` is an effort per unit
penetration, and converting MuJoCo's `solreflimit` into it needs the effective
inertia at `qpos0`, a pose-dependent quantity. Baking one number in makes the
asset wrong at every other pose. `mujoco-usd-converter` declines it for the
same reason. What it needs is a calibration pass, not a rule
([`ROADMAP.md`](ROADMAP.md) §2).

### Armature

**`armature.default`** *(on)*

```
armature = max( α·I_eq , β·I_eq_max )        α = 0.01, β = 1e-4
```

Two terms doing two jobs. The first is a rotor term proportional to what the
joint moves. The second is a conditioning floor relative to the largest
equivalent inertia in the same articulation, for DOFs whose own inertia is
negligible — on a test arm with a fingertip, `I_eq = 5.01e-6` makes 1% equal
5e-8 kg·m², which regularises nothing, and the floor gives 16× `I_eq` instead.

Authored in all three namespaces (`physxJoint:armature`, `mjc:armature`,
`newton:armature`) with the **same** value, because armature has no angle in it.

Two honest notes. `α` was swept over a hundredfold range and had **no
measurable effect** on the stability margin; it ships at its original value with
that recorded as its provenance rather than being quietly dropped. And `β` is
the one constant in the project with no measurement behind it — the low-inertia
case it exists for is not represented in the corpus, and every report says so.

### Drives

Two halves that are easy to confuse, and keeping them apart is most of the
design.

**`drives.mirror-passive`** *(on, confidence: high)* — **translation, not invention.**
The URDF said `<dynamics damping="1.5" friction="0.3">`. The converter recorded
it as `newton:damping` / `newton:friction`. Isaac Sim reads a different
spelling (`urdf:dynamics:damping`), so PhysX and MuJoCo see nothing at all —
that is [upstream issue 1](UPSTREAM_ISSUES.md). This rule writes the value the
URDF already stated into each backend's spelling, with the unit conversion each
one needs. Nothing is invented; a number that was already in your file is made
legible.

**`drives.derive-gains`** *(on, confidence: medium)* — supplying what the URDF never had.

```
I_total = I_eq + armature
K       = I_total · (2π·f_n)²
D       = 2ζ · √(I_total · K)
```

A critically damped second-order position servo, sized to the inertia the joint
actually carries. `targetPosition` is the joint's current value, so applying
the fix does not make the robot move; `maxForce` comes from the URDF's
`<limit effort>` when it has one, so the drive cannot exceed a limit the robot's
author already declared.

`f_n` is **derived, not fixed**:

| Backend selection | `f_n` | Why |
|---|---|---|
| `physx` alone | `control_rate / 6` | measured stable |
| `mujoco` alone | `control_rate / 6` | measured stable |
| `newton` alone | `control_rate / 12` | Newton's Featherstone solver **diverges** at `/6` |
| any multi-backend asset | `control_rate / 12` | an asset can only be as stiff as its least tolerant consumer |

`ζ = 1.0` was confirmed by measurement: it minimises step settle time (0.175 s,
against 0.263 s at 0.7 and 0.200 s at 1.4) with zero overshoot, where 0.7
overshoots by 5.1%.

The MuJoCo actuator gains are authored **by this project**, never by Isaac
Sim's `create_mjc_actuator_from_physics`, because that function writes the
per-degree value into the per-radian slot. See
[`UPSTREAM_ISSUES.md`](UPSTREAM_ISSUES.md) issue 2.

For Newton, the driving mechanism is `UsdPhysics.DriveAPI`, because that is
what Newton 1.5.0 and 1.6.0 actually read. `NewtonActuator` is authored only
behind `--newton-actuator`, off by default, because Newton reads nothing from
it today and a release that started to would drive the joint twice.

**`drives.no-drive-for-fixed`** *(on, report-only)*
Records that a fixed joint was deliberately skipped, so "no drive here" is a
decision in the report rather than an absence you have to notice.

---

## What it will not do

The refusals are as much the design as the repairs:

- **Never overwrite a value you authored** unless you pass `--force`. One
  function decides this, so no rule can disagree with the policy by accident.
- **Never silently change dynamics.** Every evaluation produces a record, and
  the ones that did nothing are in there too.
- **Never guess an ambiguous limit.** Fail loudly and name the flag.
- **Never invent a mass, a friction coefficient, or a limit stiffness** that
  cannot be derived or measured.

## What the asset remembers

The output root layer's `customLayerData` carries every tuning value used, the
basis string for each, the previous defaults and the date they changed, a
SHA-256 of the input, the rule-set version, and `tuning_status` — which says
`partly measured` and names `armature_floor`, because one constant still has no
measurement behind it.

An asset built with an assumption should carry that assumption.
