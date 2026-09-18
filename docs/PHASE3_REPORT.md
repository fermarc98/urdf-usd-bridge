# Phase 3 report — the stability layer, first slice

**Date:** 2026-09-18
**Host:** Ubuntu 22.04.5 x86-64, RTX 4090, Python 3.10.12, `usd-core` 26.8
**Scope:** G1 (drives), G3 (armature), G2 (inertia), G7 (joint limits) — exactly
the four the brief named. No collision filtering, no physics materials, no scene
defaults.
**Design:** `docs/PHASE3_DESIGN.md`, approved with decisions D1–D4.

---

## 1. Headline

`urdf-usd-bridge fix` authors drive gains, armature, repaired inertia and
repaired joint limits as `over` prims in new layers above an untouched input.
**196 tests pass, 0 skipped** (Phase 2.5 ended at 91). The four gaps close on
real converter output, measured by running `inspect` over the result rather than
by trusting the repair records.

Nothing here has been simulated. Every tuning constant is an engineering choice
carried in the report header, in each record's evidence, and in the output
layer's `customLayerData` with the word `unmeasured` attached. Phase 4 is what
turns them into measurements — §7.

---

## 2. What each repair does

Rules run in a fixed order — **inertia → limits → armature → drives** — because
each depends on the one before. Full formulas and their derivations are in
`docs/PHASE3_DESIGN.md` §4; this is the summary.

| Rule | Trigger | What it authors | Confidence |
|---|---|---|---|
| `inertia.derive-from-geometry` | mass > 0 and no usable tensor | `physics:diagonalInertia`, `physics:principalAxes`, `newton:inertia` from the collision geometry at uniform density | medium (low without geometry) |
| `inertia.principal-axes-identity` | authored `principalAxes` with zero norm | identity quaternion | **high** |
| `inertia.make-physical` | negative moment, or `I1 + I2 < I3` | the smallest change that is physical, always by *raising* inertia | medium |
| `inertia.mass-floor` | no mass and no density | nothing — reports at `error` severity | low, off by default |
| `limits.restore-missing` | `[0,0]` with no `<limit>` evidence | `lowerLimit = -inf`, `upperLimit = +inf` | medium |
| `limits.report-ambiguous` | `[0,0]` **with** limit evidence | nothing — names both readings | n/a |
| `limits.compliance` | finite limits, no compliance | nothing — reports | n/a |
| `armature.default` | no armature in any namespace | `newton:armature`, `physxJoint:armature`, `mjc:armature` | medium |
| `drives.mirror-passive` | `newton:damping`/`friction` authored | `mjc:damping`, `mjc:frictionloss`, `physxJoint:jointFriction` | **high** |
| `drives.derive-gains` | no drive stiffness authored | `DriveAPI` gains, `MjcActuator`, `NewtonActuator` | medium |
| `drives.no-drive-for-fixed` | fixed/spherical/generic joint | nothing — records the deliberate skip | n/a |

### 2.1 The formulas

**Equivalent inertia** — what one DOF has to move, at the default pose:

```
I_eq = sum over the subtree below the joint of ( a . R_i I_i R_i^T . a  +  m_i d_i^2 )
```

`a` is the joint axis in world space, `d_i` the perpendicular distance from that
axis to body *i*'s centre of mass. Prismatic joints use the plain mass sum. This
ignores articulated-body coupling, so it is a lower bound — the conservative
direction, since it yields softer gains.

**Armature:**

```
armature = max( alpha * I_eq ,  beta * I_eq_max )        alpha = 0.01, beta = 1e-4
```

Two terms, two jobs: a scale-relative rotor term, and a conditioning floor
relative to the largest equivalent inertia in the same articulation. Measured on
a test arm with a fingertip attached:

| DOF | `I_eq` | armature | as a multiple of `I_eq` | term used |
|---|---|---|---|---|
| shoulder | 0.800605 | 8.006e-3 | 0.01 | rotor |
| fingertip | 5.01e-6 | 8.006e-5 | **16** | floor |

The fingertip is the case the floor exists for: its own 1% would be 5e-8 kg*m^2,
which is no regularisation at all.

**Drive gains:**

```
I_total = I_eq + armature
K_si    = I_total * (2 pi f_n)^2                          f_n = 10 Hz
D_si    = 2 zeta sqrt(I_total K_si)                       zeta = 1.0
```

Worked end to end on fixture (a)'s `shoulder_joint`, `I_eq = 0.075125`:

```
armature = 0.00075125          I_total = 0.0758763
K_si     = 299.55 N*m/rad   -> DriveAPI stiffness 5.228089 N*m/deg
D_drive  =   9.53 N*m*s/rad    (+ 1.5 passive from the URDF)
                            -> DriveAPI damping   0.192595 N*m*s/deg
```

### 2.2 Units: one gain, three conventions

The reason the backends get separate layers. All three carry the *same* physical
gain:

| Backend | Attribute | Value on `shoulder_joint` | Convention |
|---|---|---|---|
| PhysX | `drive:angular:physics:stiffness` | `5.228089` | per **degree** |
| MuJoCo | `mjc:gainPrm[0]` | `299.5474` | per **radian** |
| Newton | `newton:kp` | `299.5474` | per **radian** |

`299.5474 / 5.228089 = 57.29578 = 180/pi` exactly.
`tests/unit/test_repair_units.py` asserts that ratio, asserts the two per-radian
backends agree with each other, and asserts the per-degree one is *not* equal to
them — so a naive copy fails it. Quantities with no angle unit (friction,
armature, `maxForce`) are asserted **identical** across namespaces, which catches
the mirror-image bug.

**We author `MjcActuator` gains ourselves and never let Isaac Sim's
`create_mjc_actuator_from_physics` derive them**, because that function has
exactly this bug — confirmed at runtime, §5.

### 2.3 Defaults, and that they are defaults

| Constant | Default | Flag |
|---|---|---|
| target natural frequency `f_n` | 10 Hz | `--target-frequency` |
| damping ratio `zeta` | 1.0 | `--damping-ratio` |
| armature fraction `alpha` | 0.01 | `--armature-fraction` |
| armature floor `beta` | 1e-4 | `--armature-floor` |
| assumed control rate | 60 Hz | `--control-rate` |

The control rate is **checked, never used in a formula**: `fix` warns when `f_n`
exceeds a quarter of it, because that is a gain choice the integrator will not
survive and no armature value changes it. (An earlier draft of the design
derived the armature floor from the control rate; that was circular, since `K`
is computed *from* `I_total`, and was dropped before implementation.)

All five are written into the output root layer's `customLayerData` alongside
`tuning_status: "unmeasured..."`, so an asset carries the assumptions it was
built with.

---

## 3. Output shape

```
<out>/
  <name>_stabilized.usda     subLayers = [Stability*.usda ..., <original root>]
  Stability.usda             inertia, limits, newton:* passive dynamics
  Stability_physx.usda       DriveAPI gains, physxJoint:armature/jointFriction
  Stability_mujoco.usda      MjcActuator, mjc:damping/frictionloss/armature
  Stability_newton.usda      NewtonActuator + NewtonPDControlAPI
```

Earlier sublayers are stronger, so our `over` opinions win; the original is
referenced by relative path and never opened for write.

**D1 — variant-scoped when the asset has a `Physics` variant set.** Each
backend's opinions are authored inside its matching variant (`physx`, `mujoco`,
and `physics` for Newton, which has no variant of its own). Verified on a real
Isaac Sim 6.1.0 import:

| Selection | `DriveAPI` stiffness | `MjcActuator` gains | `NewtonActuator` kp |
|---|---|---|---|
| `physx` | 5.228 | — | — |
| `mujoco` | — | 299.547 | — |
| `physics` | fallback 0 | — | 299.547 |
| `none` | — | — | — |

The `mujoco` column is the point: Isaac's `mujoco.usda` deliberately deletes
`PhysicsDriveAPI`, and a flat root sublayer is stronger than that deletion. The
first implementation leaked all three backends into whichever variant was
selected last — because `SetVariantSelection` authors into the *current edit
target*, so selections have to be written to the root layer, not to whichever
stability layer is open. `tests/unit/test_repair_units.py` and
`test_repair_layer.py` both pin the corrected behaviour.

When the asset has no variant set, the layers are flat root sublayers and the
report says so at `warning` severity.

**D2 — `--backend all` is refused on a flat layout**, with an actionable
message, because three conventions in one composed stage would put a per-degree
`DriveAPI` gain and a per-radian `MjcActuator` gain on the same joint.

**Root-layer metadata is copied explicitly.** `defaultPrim`, `upAxis`,
`metersPerUnit` and `kilogramsPerUnit` are root-layer metadata and are *not*
composed from sublayers; a new root that omits them opens with no default prim,
which silently disables every variant selection and makes an Isaac package
compose as a geometry-only asset. `test_default_prim_survives` fails if that
regresses.

---

## 4. Verified statically, behaviourally, and not at all

### 4.1 Behavioural — the gaps close on real converter output

`tests/converter/test_fix_roundtrip.py` runs `fix` over assets a real
`urdf-usd-converter` produced, then runs `inspect` on the result. Measured on
the `0.3.2` column (identical on `0.3.3` and `0.3.2-isaac`):

| Fixture | Counter | Before | After |
|---|---|---|---|
| (a) | `joints_with_drive_gains` / `joints_actuatable` | 0 / 2 | **2 / 2** |
| (a) | `joints_damping_stranded_outside_drive` | 2 | **0** |
| (a) | `joints_without_armature` | 2 | **0** |
| (b) | `bodies_invalid_principal_axes` | 2 | **0** |
| (b) | `bodies_mass_without_authored_inertia` | 2 | **0** |
| (b) | `joints_with_drive_gains` | 0 | **3** |
| (c) | `joints_locked_by_equal_limits` | 2 | **1** |
| (d) | `joints_with_drive_gains` | 0 | **2** |

Fixture (c) going 2 → **1**, not 2 → 0, is the design working: `no_limit_joint`
is unlocked, `partial_limit_joint` carries `urdf:limit:effort` and is therefore
*reported* rather than guessed at.

Also verified behaviourally: the repaired asset inspects as the same robot
(identical body and joint paths, same stage metrics, same `defaultPrim`); the
input files are byte-identical afterwards; two runs produce byte-identical
output; muting the stability layers restores the input exactly.

### 4.2 Statically — the arithmetic

Checked against closed forms rather than recorded output, so a regression cannot
be "fixed" by updating an expected value: the parallel-axis sum on hand-worked
articulations, box/sphere/cylinder inertia against their textbook formulas,
`I = R diag R^T` reconstruction, `K = I (2 pi f)^2` scaling with `f^2`, and the
`180/pi` ratio between backends.

### 4.3 Not verified at all — everything about whether it *helps*

No simulation was run in this phase, as the brief required. Specifically **not**
shown: that `f_n = 10 Hz, zeta = 1.0` holds a robot against gravity; that the
armature floor buys stability margin; that the three backends now agree; that a
derived inertia behaves like the one PhysX would have auto-computed. Those are
§7.

---

## 5. Three upstream findings, and one confirmed at runtime

`docs/UPSTREAM_ISSUES.md` carries both reports ready to file.

**Issue 1 (G1)** — Isaac Sim 6.1.0 drops `<dynamics damping/friction>`.
Confirmed at runtime in Phase 2.5 on 6.1.0-rc.26, with Isaac's own warning as
evidence. Unchanged here.

**Issue 2a — new, and confirmed at runtime in this phase.**
`create_mjc_actuator_from_physics` copies the per-degree `UsdPhysics` drive
stiffness straight into the per-radian MJCF `gainPrm` slot. Observed on a real
Isaac import with `override_joint_stiffness=800`:

```
physx   drive:angular:physics:stiffness = 13.962634   (= 800 * pi/180, correct)
mujoco  mjc:gainPrm                     = [13.962634, 0, 0, ...]
ratio gain / stored = 1.000000            (expected 57.295780)
```

The user asked for 800 N·m/rad; MuJoCo is told 13.96 N·m/rad. This matters more
than it first looks: `override_joint_stiffness` is the *documented workaround*
for Issue 1, so the workaround produces a correct PhysX drive and a 57.3×-too-soft
MuJoCo actuator from the same import.

**Issue 2b — a source reading, explicitly not reproduced.**
`convert_physx_to_mjc` writes `mjc:ref` (radians) from `targetPosition`
(degrees). The write is guarded by `if target_position:` and the default target
is `0.0`, which is falsy, so the line never executed in any run performed here.
`docs/UPSTREAM_ISSUES.md` labels it as such in its heading, its severity line
and an evidence table, so it cannot be mistaken for an observation when filed.

---

## 6. Deliverables against the brief

| # | Item | Status |
|---|---|---|
| 1 | `fix <usd> [--out] [--backend ...]` with drives, armature, inertia, limits | **Done** |
| 2 | Structured record: prim, attribute, old, new, reason, confidence | **Done** — plus `old_state`, `units`, `backend`, `layer`, `severity`, `evidence`, `forced` |
| 3 | `--dry-run`, `--json` | **Done** — dry run shares the analysis path, so it cannot diverge |
| 4 | `convert <urdf>` = converter + fix | **Done**, `--no-fix` opts out |
| 5 | Tests per repair, fixture (d), Isaac-pinned matrix column | **Done** — 196 tests, 4 fixtures, 3 columns |
| 6 | `inspect` after fixing, asserting the gaps are closed | **Done** — §4.1 |
| 7 | This report | **Done** |
| + | Author `MjcActuator` gains ourselves; 57.3× regression test | **Done** — §2.2 |
| + | `docs/UPSTREAM_ISSUES.md` | **Done** — §5 |
| + | Root metadata preserved, `defaultPrim` test | **Done** — §3 |

### What shipped

| Area | Files | Lines |
|---|---|---|
| `src/urdf_usd_bridge/repair/` | 9 | 2,946 |
| `src/urdf_usd_bridge/model/articulation.py` | 1 | 379 |
| Tests added | 7 | ~1,800 |
| Docs | `PHASE3_DESIGN.md`, `UPSTREAM_ISSUES.md`, this report | — |

**Matrix:** three columns now — `0.3.2`, `0.3.3`, and **`0.3.2-isaac`**, which
pins `usd-exchange==2.3.0` and `newton-usd-schemas==0.4.1`, the versions Isaac
Sim 6.1.0 ships, so "what the converter does" and "what an Isaac user gets" stop
being the same assumption. 12 conversions, all green.
`test_the_isaac_pinned_column_agrees_with_the_plain_one` asserts every repaired
value matches between the plain and pinned columns, so a version-sensitive
repair cannot pass unnoticed.

**Fixture (d)** closes the last source-only claim from Phase 2.5. A concave
L-shaped `<mesh>` collider (hand-written, not derived from `references/`) plus a
`<box>` control and a visual-only link. On all three columns the mesh gets
`MeshCollisionAPI` with `physics:approximation = "convexHull"` and the boxes get
none — "mesh colliders are always convexHull" is now behavioural. The fixture
also pins that Phase 3 authors **nothing** about collision: the approximation
survives untouched and the repair report contains no collision records.

### Licensing

No upstream code was copied. `THIRD_PARTY.md` records two *derivation
references* — the second-order gain relations and the MJCF position-servo slot
layout — with what was taken and what differs. Neither carries an upstream
copyright header, because neither reproduces upstream expression; adding one
would misattribute original work. The parallel-axis helper listed as a planned
adaptation in Phase 2 was not used. `references/` shows 0 modified files.

---

## 7. What Phase 4 has to measure

Every `medium`-confidence default is a hypothesis:

1. Does `f_n = 10 Hz`, `zeta = 1.0` hold a fixed-base arm against gravity in all
   three backends, with drift below a stated threshold?
2. Does the armature floor buy stability margin — the largest `dt` that survives
   hold-pose, against `armature = 0`?
3. **Do the three backends agree?** Same final base height, same settle time,
   within a stated tolerance. This is the product claim and nothing before
   Phase 4 tests it.
4. Does `inertia.derive-from-geometry` produce a robot that behaves like the one
   PhysX would have auto-computed, or meaningfully differently?
5. Is `limits.restore-missing` (welded → unlimited) an improvement in practice,
   or does it need a bounded default?

---

## 8. Known rough edges

Small, honest, and none of them silent.

- ~~`convert` with default `--backend all` fails on its own output.~~
  **Resolved after review.** `convert` now writes one stabilized root per
  backend (`<name>_stabilized_physx.usda`, `_mujoco`, `_newton`), each composing
  the shared neutral layer, its own backend layer and the original. The gain
  conventions never meet, and the caller picks a file rather than a flag. `fix`
  keeps its refusal, because the asset handed to it may be an Isaac package
  where variant scoping is the better answer.
- **The neutral layer applies under the `none` variant too.** Inertia and limit
  repairs are flat root sublayers so they hold under every physics variant;
  under `none` that leaves a few empty `over` prims in an asset that is supposed
  to be visual-only. Harmless, but untidy.
- **Newton actuators are opt-in for a reason.** The `NewtonActuator` schema
  family documents itself as EXPERIMENTAL — "attribute names, defaults, and
  composition rules may change without notice". `--backend newton` works and is
  tested; it is not in a default single-backend run.
- **`inertia.derive-from-geometry` approximates meshes by their oriented
  bounding box**, and cylinders/capsules/cones by their elliptic generalisations
  under non-uniform scale. Exact for boxes, ellipsoids and mesh bounds. Every
  record says which it used.
- **The mass-ratio diagnosis is reported, never repaired.** The fix is solver
  iteration counts or model surgery, both out of scope.
