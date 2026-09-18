# urdf-usd-bridge — Phase 1 Analysis

> **Development record.** Written during the phase it describes and kept for
> provenance, not maintained since. Where it disagrees with the current
> documentation, the current documentation is right — start at
> [`docs/history/README.md`](README.md).

**Date:** 2026-09-17
**Status:** analysis only, no library code written.
**Method:** read directly from the pinned reference checkouts in `references/`.
Every claim below is either cited to a file:line, to an upstream CHANGELOG entry,
or explicitly marked as *unverified — needs a runtime check*.

---

## 0. Reference state

| Repo | Ref | Commit | Commit date |
|---|---|---|---|
| `references/urdf-usd-converter` | `main` = `v0.3.3` | `b6364698371346a8e01bdbbb9bae312b78c67975` | 2026-08-06 |
| `references/mujoco-usd-converter` | `main` = `v0.5.0` | `657158f1cbf83702ab7017a8555b1977da6ef677` | 2026-08-05 |
| `references/IsaacSim` | detached `v6.1.0` (shallow) | `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` | 2026-09-10 |
| `references/IsaacSim` | tag `v6.0.1` (fetched `--depth 1`) | `045ca8b59622b99a408092124377c66346e8d9c2` | 2026-06-22 |
| `references/IsaacSim` | tag `v5.1.0` (fetched `--depth 1`) | `47d886f2858d1ceed556b21c88927aa67bc81c12` | 2025-10-21 |

All three working trees are clean; nothing under `references/` is tracked by our repo
(`.gitignore:2`).

### Version matrix that Isaac Sim 6.1.0 actually ships

From `references/IsaacSim/python_packages.toml`, `deps/pip_urdf_usd.toml`, and
`source/extensions/isaacsim.asset.importer.urdf/pyproject.toml`:

| Package | Isaac Sim 6.1.0 | Isaac Sim 6.0.1 |
|---|---|---|
| `urdf-usd-converter` | **0.3.2** | 0.1.3 |
| `usd-exchange` | 2.3.0 | — |
| `newton[sim]` | 1.5.0 | — |
| `newton-usd-schemas` | 0.4.1 | 0.2.0 |
| `mujoco` / `mujoco-warp` | 3.11.0 / 3.11.0 | 3.8.0 / 3.8.0.3 |
| `mujoco-usd-converter` | 0.5.0 | 0.2.0 |
| `numpy` | 2.3.1 | 2.3.1 |

Note the version skew we inherit if we build on the Isaac path: Isaac pins
**0.3.2**, but upstream `main` is already **0.3.3**, and 0.3.3 fixes two inertia
bugs that are therefore *live in Isaac Sim 6.1.0* (see §4, G2).

### Verification of the three stated facts

1. **"Isaac Sim 6.1.0's URDF importer uses urdf-usd-converter 0.3.2, then
   post-processes and splits physics into `physx.usda` / `mujoco.usda` layers
   behind a 'Physics' variant."** — **Confirmed, with refinements.** The pin is
   `urdf-usd-converter==0.3.2` (`deps/pip_urdf_usd.toml:19`,
   `isaacsim.asset.importer.urdf/pyproject.toml:23`, `python_packages.toml:338`).
   The split is into **three** layers — `physics.usda`, `physx.usda`,
   `mujoco.usda` — not two, and the two backend layers each *sublayer* the neutral
   one. In 6.1.0 the `Physics` variant set has **no default selection authored**
   (it did in 6.0.1). Details in §2.
2. **"Neither project appears to detect or repair physically unstable input."** —
   **Substantially confirmed, with two exceptions worth naming**: Isaac Sim ships
   a SimReady validator `NonAdjacentCollisionMeshesDoNotClash`
   (`isaacsim.asset.validation/collision_validation.py:221`) that *detects*
   resting interpenetration, and an interactive Gain Tuner
   (`isaacsim.robot_setup.gain_tuner`) with ω_n/ζ math, divergence detection and a
   dt sweep. Neither is part of the import pipeline; both are out-of-band,
   PhysX-runtime-bound, and report-or-tune-by-hand rather than repair. Details in §4.
3. **Implied third fact — the converters are Apache-2.0 and reusable.** —
   Confirmed for the Python sources; see §7 for the two files that are not.

---

## 1. Newton `urdf-usd-converter` architecture

Apache-2.0, `newton-physics/urdf-usd-converter`. ~4.7k lines of Python across
`urdf_usd_converter/_impl/`. Entry point is one class:

```python
Converter(layer_structure=True, scene=True, comment="", ros_packages=[])
  .convert(input_file, output_dir) -> Sdf.AssetPath
```

### 1.1 Pipeline

`_impl/convert.py:38-164`, in order:

1. **Parse** — a hand-written URDF parser (`_impl/urdf_parser/`, ~1.6k lines) with
   line-number tracking, a reserved-name table, and capture of *undefined*
   (non-spec) elements/attributes so they survive as `urdf:`-namespaced custom
   attributes rather than being dropped.
2. **Resolve `package://`** — `_impl/ros_package.py` walks up from the URDF to
   auto-discover package roots; CLI/API overrides merge on top.
3. **Build `ConversionData`** (`_impl/data.py:29`) — a single mutable bag holding
   the parser, per-token stages, a `usdex.core.NameCache`, the `LinkHierarchy`,
   and mesh/material caches.
4. **`LinkHierarchy`** (`_impl/link_hierarchy.py`) — derives the kinematic tree
   from `<joint>` parent/child, finds the root link, raises
   `"Closed loop articulations are not supported."` (`link_hierarchy.py:187`), and
   runs the **ghost-link** analysis (below).
5. **Meshes → `GeometryLibrary`** (`_impl/mesh.py`, `mesh_cache.py`) via
   numpy-stl / tinyobjloader / pycollada. Deduplicated; referenced, not copied.
6. **Materials → `MaterialsLibrary`** (`_impl/material.py`, 579 lines) —
   `UsdPreviewSurface`; embedded DAE/OBJ materials take priority over URDF
   materials, matching `rviz`.
7. **Content layers** — `Geometry.usda` and `Physics.usda` added via
   `usdex.core.addAssetContent`.
8. **Scene** — a `UsdPhysics.Scene` + `NewtonSceneAPI`, *only if* `scene=True`
   (`_impl/scene.py`).
9. **Links & joints** (`_impl/link.py`, 579 lines) — the core; see 1.3.
10. **Undefined elements** → `custom` Scopes / `urdf:` attributes
    (`_impl/undefined.py`).
11. **Asset interface** — `usdex.core.addAssetInterface`; optional
    flatten (`_impl/_flatten.py`) when `layer_structure=False`.

### 1.2 Output asset structure (`layer_structure=True`)

An **Atomic Component** per NVIDIA's *Principles of Scalable Asset Structure*
(`docs/concept_mapping.md`, Appendix D):

```
<robot>.usda                 # asset interface: metadata, Kind, payload
  Payload/
    Contents.usda            # sublayers the content layers
    Geometry.usda            # link Xform hierarchy + Gprims, NO physics, NO materials
    Materials.usda           # material bindings only
    Physics.usda             # ALL UsdPhysics + Newton schemas (as `over` prims)
    GeometryLibrary.usdc     # class-scoped reusable meshes
    MaterialsLibrary.usda
    Textures/
```

Prim hierarchy on the composed stage:

```
/<Robot>                     (Xform, defaultPrim)
  /Geometry   (Scope)  /root_link/child_link/grandchild_link ...   ← NESTED links
  /Physics    (Scope)  /root_joint, /<joint_name>, ...
  /Materials  (Scope)
```

Stage metrics are set explicitly: `upAxis = Z`, `metersPerUnit = 1.0`,
`kilogramsPerUnit = 1` (`convert.py:106-114`).

Key structural decision: **links are nested**, mirroring the kinematic tree. The
README is explicit that this is a compatibility risk — *"The rigid bodies are
structured hierarchically, which maximal coordinate solvers often do not support."*

### 1.3 Physics mapping (`_impl/link.py`)

| URDF | USD authored | Where |
|---|---|---|
| `<link>` | `UsdGeom.Xform` + `UsdPhysics.RigidBodyAPI` (as `over` in Physics layer) | `link.py:78-82` |
| root link | + `UsdPhysics.ArticulationRootAPI` + `NewtonArticulationRootAPI` | `link.py:86-90` |
| (root, non-ghost) | synthesized `root_joint` fixed joint to the world | `link.py:390-397` |
| `<inertial><mass>` | `UsdPhysics.MassAPI` `physics:mass` | `link.py:184` |
| `<inertial><origin xyz>` | `physics:centerOfMass` | `link.py:180` |
| `<inertial><inertia>` | `newton:inertia` (double[6], body frame) **and** `physics:diagonalInertia` + `physics:principalAxes` via eigendecomposition | `link.py:155-171` |
| `<collision>` | `UsdPhysics.CollisionAPI` + `NewtonCollisionAPI`; meshes get `MeshCollisionAPI` with **`approximation = convexHull`, always** | `geometry.py:67-84` |
| `<visual>` | Gprim, `purpose=default`; collisions get `purpose=guide` | `geometry.py:50` |
| `fixed`/`revolute`/`continuous`/`prismatic` | `usdex.core.definePhysics*Joint`, radians→degrees | `link.py:432-447` |
| `planar` | generic `UsdPhysics.Joint` + `LimitAPI` locks (`low=+inf, high=-inf`) | `planar_joint.py` |
| `floating` | **nothing** — explicitly a no-op | `link.py:441-445` |
| `<mimic>` | `NewtonMimicAPI` (`newton:mimicJoint/Coef0/Coef1`) | `link.py:558-580` |
| `<dynamics damping/friction>` | `newton:damping` / `newton:friction` (**NewtonJointAPI only**) | `link.py:485-491` |
| `<limit velocity>` | `newton:velocityLimit` | `link.py:493` |
| `<limit effort>` | `urdf:limit:effort` **custom attribute** — declared a *GAP* | `link.py:530` |
| `<calibration>`, `<safety_controller>` | `urdf:*` custom attributes | `link.py:532-555` |
| `<transmission>` | **nothing** — "not well specified… cannot map to USD" | `concept_mapping.md:152` |
| `<gazebo>` | captured as undefined/custom | `concept_mapping.md:958` |

**Ghost links.** A link with no inertial *and* no visual *and* no collision is a
"ghost link" (`link_hierarchy.py:105`). If a ghost chain terminates the tree and is
reached by fixed joints, the rigid bodies are dropped and the joints skipped; joints
whose `body0` resolves to a body-less link are re-targeted at the nearest
rigid-body ancestor, with the dropped fixed-joint origins accumulated into
`physics:localPos0` / `localRot0` (0.3.3 CHANGELOG). This is genuinely good
engineering and we should preserve the behaviour rather than re-derive it.

### 1.4 What it deliberately does **not** author

Grepped across the whole package — zero occurrences of any of these:

- `UsdPhysics.DriveAPI` — **no actuation at all**
- armature / rotor inertia
- `physics:filteredPairs`, collision groups, self-collision control
- contact offset, rest offset, CCD, sleep/stabilization thresholds
- max linear/angular velocity clamps
- `UsdPhysics.MaterialAPI` (friction, restitution)
- solver iteration counts
- any mesh approximation other than `convexHull`
- any validation of mass, inertia, or geometry

This is a defensible scope decision — it is a *data converter*, faithful to URDF,
and URDF simply does not carry most of this. But it means the output is
**kinematically correct and dynamically unusable** without a second pass.

### 1.5 Benchmarks

`benchmarks.md` (2026-04-17): 408 public models, 394 converted (96.6%). The
corpora are named in `tools/*_annotations.yaml`: **SO-ARM100** (24),
**example-robot-data** (616), **newton-assets** (176), **urdf_files_dataset**
(2445). Crucially the benchmark measures *conversion success, time, and file
size* — **it never simulates the result.** That is the hole we fill.

---

## 2. What Isaac Sim 6.1.0 adds, and the exact structure it emits

Extension `isaacsim.asset.importer.urdf` v3.11.10. `URDFImporter.import_urdf()`
(`python/impl/converter.py:87-259`) is a wrapper, not a converter.

### 2.1 The pipeline

```python
self.converter = urdf_usd_converter.Converter(layer_structure=False, scene=False)
```
(`converter.py:155`) — note it **discards the Newton layer structure entirely**
(flatten) and **suppresses the PhysicsScene**, then rebuilds its own structure.

Order of operations (`converter.py:135-250`):

| # | Step | Default | Notes |
|---|---|---|---|
| 1 | `merge_fixed_joints()` on the **URDF XML** | off | Correct parallel-axis inertia composition (`urdf_utils.py:474-520`) |
| 2 | `urdf_usd_converter.Converter(...).convert()` | — | flattened `.usdc` in a temp dir |
| 3 | `reconstruct_source_geometry` / `_joints` | auto | round-trip only: parses `isaac:source_geometry` / `isaac:source_joint` XML comments left by Isaac's *own* URDF **exporter**, to restore Capsule/Cone and Spherical/D6 joints. No effect on third-party URDFs. |
| 4 | `remove_custom_scopes()` | on | deletes `/<Robot>/Geometry/custom` |
| 5 | **`add_joint_schemas()`** | on | applies `PhysxJointAPI`, `UsdPhysics.DriveAPI:<angular\|linear>`, `PhysxJointStateAPI` to every revolute/prismatic joint — **with no values set** (`importer_utils.py:391-411`) |
| 6 | `reconstruct_source_drives` | auto | round-trip only, as #3 |
| 7 | `apply_fix_base` / `apply_floating_base` | `None` = untouched | adds/removes the world fixed joint, relocates `ArticulationRootAPI` |
| 8 | `apply_link_density(d)` | `None` | sets `physics:density` **only** on bodies whose mass is missing or 0 (`asset_utils.py:442`) |
| 9 | `apply_joint_drives(...)` | `None` | the only way to get real gains; correctly converts Nm/rad → Nm/deg (`asset_utils.py:577-624`) |
| 10 | mesh merge / cleanup | off | `merge_mesh_utils` |
| 11 | `collision_from_visuals(type)` | off | drops `purpose=guide` colliders, re-derives colliders from visuals, and is the **only** path that sets `convexDecomposition` / `boundingSphere` / `boundingCube` (`importer_utils.py:181-256`) |
| 12 | `enable_self_collision(bool)` | `False` | writes `newton:selfCollisionEnabled` on articulation roots — global, not per-pair |
| 13 | `create_robot_schema(robot_type)` | on | `IsaacRobotAPI` / `IsaacLinkAPI` / `IsaacJointAPI` / `IsaacSiteAPI` |
| 14 | **`convert_joints_attributes()`** | on | URDF → PhysX → MJC translation; see 2.2 |
| 15 | `run_asset_transformer_profile(DEFAULT_PROFILE_PATH)` | on | restructures into the final package; see 2.3 |

### 2.2 The URDF → PhysX → MuJoCo joint translation

`isaacsim/asset/importer/utils/python/impl/urdf_to_mjc_physx_conversion_utils.py`.
For every `RevoluteJoint`/`PrismaticJoint` it applies `MjcJointAPI` then runs three
passes:

- `convert_urdf_to_physx()` — reads `urdf:limit:effort` → `DriveAPI.maxForce`;
  reads `urdf:dynamics:damping` → `DriveAPI.damping`; reads
  `urdf:dynamics:friction` → `PhysxJointAPI physxJoint:jointFriction`; reads
  `urdf:calibration:reference_position` → `DriveAPI.targetPosition`.
- `create_mjc_actuator_from_physics()` — creates `MjcActuator` prims under
  `/<Robot>/Physics`, `mjc:target` → the joint, `mjc:forceRange:min/max` from
  `maxForce`, and derives `mjc:gainPrm` / `mjc:biasPrm` / `gainType=fixed` /
  `biasType=affine` from the PhysX stiffness/damping (position-control form
  `[kp,0,…]` / `[0,-kp,-kd,…]`, velocity-control form `[kd,0,…]` / `[0,0,-kd,…]`).
- `convert_physx_to_mjc()` — `targetPosition` → `mjc:ref`,
  `physxJoint:jointFriction` → `mjc:frictionloss`, `physxJoint:armature` →
  `mjc:armature`.

**This is where Isaac Sim 6.1.0 is broken.** See §4 G1 — the attributes it reads
are not the attributes converter 0.3.2 writes.

### 2.3 The asset-transformer profile — the actual output structure

`isaacsim.asset.transformer.rules/data/isaacsim_structure.json`, 20 rules. Net effect:

```
<robot>/
  <robot>.usda                    # interface layer: defaultPrim + Reference to payloads/base
  payloads/
    base.usd(a)                   # flattened geometry/xforms; sublayers robot.usda
    robot.usda                    # Isaac* robot schemas + isaac:* properties
    geometries.usd                # deduplicated mesh library
    instances.usda                # instanced references into geometries.usd
    materials.usda                # visual materials
    Textures/
    Physics/
      physics.usda                # UsdPhysics.* + Newton.*  (backend-neutral)
      physx.usda                  # Physx.* schemas/prims/properties; SUBLAYERS physics.usda
      mujoco.usda                 # Mjc.*  schemas/prims/properties; SUBLAYERS physics.usda
                                  #   + deletions of PhysicsDriveAPI* / PhysicsJointStateAPI*
```

Routing is purely namespace-driven (`core/schemas.py`, `core/prims.py`,
`core/properties.py`):

| Layer | Matches | Excludes |
|---|---|---|
| `physics.usda` | `Physics.*`, `Newton.*` | `PhysicsCollisionAPI` (stays in base) |
| `physx.usda` | `Physx.*`, `physx.*` | — |
| `mujoco.usda` | `Mjc.*`, `mjc.*` | — |
| `robot.usda` | `IsaacRobotAPI`, `IsaacLinkAPI`, `IsaacJointAPI`, `IsaacSiteAPI`, `IsaacSurfaceGripper`, `IsaacAttachmentPointAPI`, `IsaacReferencePointAPI` | — |

Two rules matter for backend fidelity:

- **`Fix Physics Joint Poses`** (`isaac_sim/physics_joint_pose_fix.py`) —
  after the geometry rule re-parents and combines transforms, it recomputes each
  joint's world pose from both bodies against the *original* stage on disk and
  corrects `localPos0/1`, `localRot0/1` so the joint frame is preserved.
  Tolerance 1e-6.
- **`Delete Physics Drive and Joint State APIs`** — in `mujoco.usda` only, it
  authors removals of `PhysicsDriveAPI.*` and `PhysicsJointStateAPI.*`, because
  MuJoCo actuation lives on `MjcActuator` prims instead. This is the cleanest
  idea in the whole design and we should copy the pattern.

**The `Physics` variant set.** `InterfaceConnectionRule` scans `payloads/*/` and
turns each subfolder into a variant set whose variants are its USD filenames, each
adding a **payload**, plus a synthesized `None` variant
(`structure/interface.py:331-440`). So:

```
variantSet "Physics" = { None, mujoco, physics, physx }
```

Selecting `physx` payloads `physx.usda`, which sublayers `physics.usda` → neutral
UsdPhysics + PhysX overlay. Selecting `mujoco` payloads `mujoco.usda` → neutral
UsdPhysics + MJC overlay − PhysX drives. Selecting `physics` gives the
backend-neutral layer alone. Selecting `None` gives a pure visual asset.

In **6.1.0 no default selection is authored** (`"clear_default_variant_sets":
["Physics"]`). Consumers must select explicitly — Isaac's own tests do
`prim.GetVariantSet("Physics").SetVariantSelection("physx")`
(`python/tests/test_urdf.py:190`).

### 2.4 Final prim paths

From Isaac's own test assertions (`python/tests/test_urdf.py:188-210`):

```
/test_basic                                                     (defaultPrim, Physics variantSet)
/test_basic/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_1   ← still NESTED
/test_basic/Physics/root_to_base            (PhysicsFixedJoint)
/test_basic/Physics/wrist_joint             (PhysicsRevoluteJoint)
/test_basic/Physics/<joint>_actuator        (MjcActuator)
```

So Isaac **keeps** the Newton converter's `Geometry`/`Physics` scope names and its
nested-body layout; it changes the *layer* organisation, not the prim topology.

---

## 3. Comparison with Isaac Sim 6.0.1 and 5.1.0

### 3.1 Isaac Sim 5.1.0 (2025-10-21) — a different program

The importer was **native C++**: `plugins/isaacsim.asset.importer.urdf/UrdfImporter.cpp`,
`UrdfParser.cpp`, `MeshImporter.cpp`, `KinematicChain.cpp`, with pybind bindings and
`python/impl/commands.py` (Kit commands `URDFCreateImportConfig`,
`URDFParseAndImportFile`). There is **no** `urdf-usd-converter` dependency, **no**
asset transformer, **no** MJC schemas, **no** `Physics` variant, **no** `Newton*`
schemas. Output was a single USD (optionally instanceable) with PhysX schemas
applied inline, driven by a `UrdfImportConfig` struct (`fixBase`,
`defaultDriveType`, `defaultDriveStrength`, `importInertiaTensor`,
`selfCollision`, `distanceScale`, `convexDecomp`, …).

Practical consequence: **Isaac Sim 5.1.0 cannot consume the 6.x package layout**,
and the 6.x importer cannot reproduce 5.1.0's output. There is no "compatible with
both" structure here — only a compatible *interface layer*.

### 3.2 Isaac Sim 6.0.1 (2026-06-22) — same architecture, three deltas

Extension v3.11.2. Python importer, `urdf-usd-converter==0.1.3`, same
`isaacsim_structure.json` except for the tail. `diff v6.0.1 v6.1.0` on the profile
shows exactly three changes:

1. `default_variant_selections: {"Physics": "physx", "Gripper": "None",
   "Left_Hand": "None", "Right_Hand": "None"}` → **removed**, replaced by
   `clear_default_variant_sets: ["Physics"]`.
   → *6.0.1 assets open with PhysX physics live; 6.1.0 assets open with no physics
   until a variant is chosen.*
2. `Delete Physics Drive and Joint State APIs` gained
   `input_stage_path: payloads/Physics/physics.usda`.
3. A disabled `Delete Newton Redundant APIs` rule (`NewtonMimicAPI`,
   `NewtonArticulationRootAPI` out of `physx.usda`) was deleted outright.

Everything before line 278 of the profile — variant routing, base flattening,
joint-state APIs, robot-schema routing, geometry/material routing, joint pose fix,
PhysX/MuJoCo/Physics routing — is **byte-identical** between 6.0.1 and 6.1.0.

Other 6.0.1 → 6.1.0 deltas (from `docs/CHANGELOG.md` and the tree diff):

- `isaacsim.asset.importer.utils` moved from `source/extensions/` to
  `source/libraries/`, and gained `physx_asset_to_mjc.py` plus MJC schema
  resources (`schemas/generatedSchema.usda`, `plugInfo.json`).
- converter 0.1.3 → 0.3.0 → 0.3.2; `newton-usd-schemas` 0.2.0 → 0.4.1;
  `mujoco` 3.8 → 3.11; `mujoco-usd-converter` 0.2.0 → 0.5.0.
- 3.10.0: `PhysxMimicJointAPI` authoring removed in favour of `NewtonMimicAPI`;
  `PhysxArticulationAPI` no longer applied to articulation roots — self-collision
  moved to `newton:selfCollisionEnabled`.
- 3.11.0: `fix_base` became tri-state `bool | None`; `MassAPI` only applied when a
  non-default density is requested.
- 3.11.5: joint friction moved from `physxJoint:jointFriction` to `newton:friction`.
- 3.11.8: `package://` resolution after fixed-joint merging.

### 3.3 What structure should **we** emit?

**Recommendation: emit the Newton `urdf-usd-converter` layer structure as our
canonical output, and ship an Isaac-6.x package writer as a selectable target.**

Reasoning:

- The Newton structure is the upstream, spec-documented one
  (`concept_mapping.md` Appendix D), it is the one both Newton converters agree
  on, and Isaac itself *consumes* it (it just flattens it first). Anything we
  author into `Physics.usda` survives Isaac's namespace routing automatically:
  `Physics.*` and `Newton.*` land in `physics.usda`, `Physx.*` in `physx.usda`,
  `Mjc.*` in `mujoco.usda`. **Our layer content is forward-compatible with the
  Isaac profile without us reimplementing the profile.**
- Targeting the Isaac 6.x package directly is still worth offering, because that
  is what Isaac Lab and the Isaac asset browser expect, and because the
  `Physics` variant is a genuinely good UX for "which backend am I loading".
- **Do not target 5.1.0's layout.** It is a dead architecture and chasing it would
  constrain every design decision. Instead offer a `--compat flat-bodies` switch
  (see below), which is the *actual* compatibility problem, and which also buys
  us Unreal/Omniverse-maximal-coordinate compatibility.

**The real cross-version compatibility axis is not the layer structure — it is
nested vs. flat rigid bodies.** Nested bodies in `UsdPhysics` only stabilised
around USD 25.11 (`mujoco-usd-converter/README.md:31`), and both Newton READMEs
warn that maximal-coordinate solvers reject them. Concretely:

| Target | Nested bodies OK? |
|---|---|
| Newton ≥ 1.5 | yes |
| Isaac Sim 6.x (PhysX articulations) | yes |
| MuJoCo (USD-enabled build) | yes — matches MJCF's kinematic tree |
| Isaac Sim 5.1.0 | *unverified — needs a runtime check* |
| Omniverse / Unreal / generic maximal-coordinate UsdPhysics | no |

So our compatibility plan is: **nested by default** (matches 6.1, 6.0.1, Newton,
MuJoCo), with a `flat` body-layout mode that hoists every link to a sibling and
bakes world-space transforms, exposed as a `BodyLayout` variant set alongside
`Physics`. Emit stage metrics explicitly (`upAxis=Z`, `metersPerUnit=1`,
`kilogramsPerUnit=1`) in every layer, as both converters do — that alone prevents
a large class of "it exploded in the other app" reports.

---

## 4. Stability problems none of them handle

Ordered by how often they bite. For each: what the input looks like, what gets
authored, and what each backend does with it.

### G1 — No actuation, and in Isaac Sim 6.1.0 the damping is silently dropped

**Newton converter:** authors *no* `UsdPhysics.DriveAPI` anywhere. A converted
6-DoF arm has six unactuated joints.

**Isaac Sim 6.1.0:** `add_joint_schemas()` applies `DriveAPI` to every
revolute/prismatic joint but sets **no values** (`importer_utils.py:391-411`), so
stiffness = damping = 0 unless the caller passes `override_joint_stiffness` /
`override_joint_damping`.

**And there is a live regression.** `convert_urdf_to_physx()` reads:

```python
joint.GetAttribute("urdf:dynamics:damping")    # urdf_to_mjc_physx_conversion_utils.py:205
joint.GetAttribute("urdf:dynamics:friction")   # :214
```

Converter **0.1.3** (Isaac 6.0.1) authored exactly those attributes
(`link.py:408,411` at `v0.1.3`). Converter **0.3.0+** (Isaac 6.1.0 pins 0.3.2)
replaced them with `newton:damping` / `newton:friction` — commit `99ff034`,
*"Updated to newton schemas v0.4.0"*, which deletes both `CreateAttribute` calls.
Isaac's reader was never updated. Confirmed: `git grep 'urdf:dynamics' v0.3.2` in
the converter returns nothing.

Net effect in Isaac Sim 6.1.0, for any URDF with `<dynamics damping="...">`:

- `DriveAPI.damping` stays 0 → PhysX drives contribute nothing.
- `physxJoint:jointFriction` is never set.
- `create_mjc_actuator_from_physics` falls into its `else` branch and logs
  *"Stiffness and damping not available … actuator will be created without gain
  parameters"*, emitting an `MjcActuator` with no `gainPrm`/`biasPrm`.

The unit test passes because
`tests/test_urdf_to_mjc_physx_conversion.py:50` hand-authors
`urdf:dynamics:damping` on a synthetic stage — it never runs the real converter.

**Failure modes:**
- *PhysX:* a manipulator released at its home pose collapses under gravity and
  settles on its joint limits; the limits then act as the only stop, producing
  chatter at the limit. A fixed-base arm looks like it "melted".
- *MuJoCo:* an `MjcActuator` with default gain is a unit-gain force actuator, not a
  PD controller — position commands do nothing, and `ctrl=0` means zero torque.
  Same collapse, plus the user's `qpos` targets are silently ignored.
- *Newton:* `newton:damping` *is* present so joints are viscous but unactuated —
  the arm sags slowly instead of dropping. **Three backends, three different
  wrong answers from one asset.** That divergence is itself the bug.

**Also a latent unit bug:** the 6.0.1 path set `DriveAPI.damping` from a
per-radian URDF value straight into USD's per-degree convention, i.e. **57.3×
too stiff**. Isaac's own `apply_joint_drives` does the conversion correctly
(`asset_utils.py:601-624`), so the two code paths disagree. A 57× stiffness error
at 60 Hz is an immediate explosion.

### G2 — Zero, missing, and non-physical inertia

**Zero inertia is authored, not rejected.** Isaac's own test asserts it:

```python
# test_urdf.py:255 — link with mass 10 kg and no <inertia>
self.assertAlmostEqual(prim.GetAttribute("physics:diagonalInertia").Get()[0], 0.0)
self.assertAlmostEqual(prim.GetAttribute("physics:mass").Get(), 10.0)
```

A 10 kg body with a zero inertia tensor and (in that test) no collision geometry
to auto-compute from.

**Invalid quaternion in Isaac Sim 6.1.0's pinned converter.** At `v0.3.2`,
`apply_inertial()` does `mass_api.GetPrincipalAxesAttr().Set(orientation * axes)`
whenever `<inertial>` has an `origin`, *including when there is no `<inertia>`*.
0.3.3's CHANGELOG: *"Fixed an invalid zero `physics:principalAxes` authored when
`<inertial>` specifies `origin` and `mass` but no `<inertia>`."* Isaac 6.1.0 pins
0.3.2, so it ships the bug.

**Non-physical tensors pass straight through.** Nothing checks
positive-definiteness or the triangle inequality (I₁+I₂ ≥ I₃). `_extract_inertia`
runs `np.linalg.eigh` and writes whatever eigenvalues come out — including negative
ones. Hand-authored URDFs violate the triangle inequality constantly.

**Mass-ratio conditioning is never examined.** A 50 kg base rigidly chained to a
5 g fingertip gives an articulation mass matrix with a ~10⁴ condition number.

**Failure modes:**
- *PhysX:* near-zero inertia → angular acceleration ≈ τ/I blows up within a step;
  the articulation link spins to NaN. PhysX will auto-compute inertia from
  colliders when the tensor is zero, which silently **changes the robot's
  dynamics** relative to the URDF — and does nothing at all for visual-only or
  ghost links. Negative eigenvalues produce an indefinite mass matrix and the
  TGS solver diverges (energy gain per step instead of loss).
- *MuJoCo:* requires positive-definite body inertia; `mj_compile` errors out, or —
  with `inertiafromgeom` — silently substitutes a geometry-derived tensor, so
  MuJoCo and PhysX now simulate *different robots* from the same file.
- *Newton:* reads `newton:inertia`, which carries the same zeros; a zero 6-vector
  gives a singular spatial inertia and the featherstone pass produces NaN.
- *All three:* high mass ratios → the iterative solver cannot converge in the
  default iteration budget → visible high-frequency jitter at the light end of the
  chain, which users misdiagnose as a gain problem.

### G3 — No armature / rotor inertia anywhere on the URDF path

URDF has no armature element; MJCF does, and `mujoco-usd-converter` authors both
`mjc:armature` and `newton:armature` (`joint.py:82,96`). The URDF path authors
neither. Isaac's `convert_physx_to_mjc` copies `physxJoint:armature` →
`mjc:armature` (`urdf_to_mjc_physx_conversion_utils.py:345`) but **nothing in the
URDF import ever sets `physxJoint:armature`**, so it is always absent.

Armature adds a diagonal term to the joint-space mass matrix and is the standard
cure for stiff-PD instability on geared joints. Without it:

- *PhysX:* a gripper finger with I ≈ 1e-6 kg·m² under a position drive with
  k = 800 has ω_n ≈ 28 kHz. At 60 Hz the discrete PD loop is unconditionally
  unstable — the classic gripper "buzz" that becomes an explosion when something
  is grasped.
- *MuJoCo:* same, and worse for tendon-driven hands; the Menagerie models all
  carry hand-tuned `armature` for exactly this reason.
- *Newton:* `newton:armature` absent → same.

The right value is recoverable from the URDF when a `<transmission>` block with
`mechanicalReduction` exists (armature ≈ n²·I_rotor), and otherwise estimable as a
fraction of the joint's accumulated child inertia.

### G4 — Collision geometry: `convexHull` always, and no pair filtering

`apply_physics_collision()` unconditionally sets
`approximation = convexHull` (`geometry.py:80`). Isaac only changes this when
`collision_from_visuals=True`, and then it applies to *visual* geometry.

Nobody authors `physics:filteredPairs`, collision groups, or MuJoCo `<contact>`
exclusions. Isaac's `allow_self_collision` is a single global boolean
(`newton:selfCollisionEnabled`) on the articulation root.

**Failure modes:**
- A concave collision mesh (gripper jaw, C-bracket, shell) becomes its convex
  hull, which intersects the neighbouring link's hull **at the home pose**. On
  step 0, *PhysX* generates a deep-penetration contact and applies a depenetration
  impulse proportional to the overlap — the robot visibly pops apart on play.
  *MuJoCo* generates a large `solref`-driven restoring force and the assembly
  vibrates. *Newton* behaves like MuJoCo when using the MuJoCo solver and like
  PhysX otherwise.
- Adjacent links are only auto-excluded if the backend does so; PhysX excludes
  joint-connected articulation links, MuJoCo excludes parent-child bodies by
  default — but neither excludes the *grandparent* or the two fingers of a gripper.
  With `allow_self_collision=True` for RL, a humanoid immediately self-jams.
- Convex hulls also fatten thin plates, so a foot sinks through a floor it should
  stand on, or a wheel rides on a hull that is larger than the tyre.

Isaac Sim *can* detect this — `NonAdjacentCollisionMeshesDoNotClash` steps PhysX
once and reports contacting non-adjacent pairs — but it lives in
`isaacsim.asset.validation`, requires a live Kit + PhysX process, is PhysX-only,
runs nowhere in the import path, and **only reports**.

### G5 — No physics materials

URDF has no standard friction. Gazebo's `<gazebo><mu1>/<mu2>` extensions become
inert `urdf:`-namespaced custom attributes. No `UsdPhysics.MaterialAPI` is ever
authored, so every backend uses its own default:
PhysX ≈ 0.5 static / 0.5 dynamic, MuJoCo sliding friction = 1.0. A wheeled robot
that drives in one backend spins its wheels in the other, and a quadruped's feet
slip differently in each — which reads as "the controller is broken" rather than
"the asset is underspecified".

### G6 — No scene, and unit-convention drift across the boundary

Isaac calls the converter with `scene=False`, and nothing in the Isaac pipeline
authors a `UsdPhysics.Scene`. So gravity, timestep, solver iteration counts, and
the PhysX/MuJoCo solver selection all come from whatever host loads the asset.
The same file is stable in an app defaulting to 240 Hz and explodes in one
defaulting to 60 Hz, with no way to tell from the asset that this would happen.

Separately, **degrees vs radians is a recurring, already-demonstrated bug class**
at every URDF↔USD↔backend boundary:

- `urdf-usd-converter` 0.3.2 fixed `newton:mimicCoef0` to degrees, and notes
  *"Newton 1.4 and older fail to handle the degrees-to-radians conversion for
  `coef0`"* — so the asset's correctness depends on the reader's version.
- `mujoco-usd-converter` 0.4.1 fixed `newton:damping` to degrees for angular joints.
- `convert_urdf_to_physx` sets `DriveAPI.damping` with no conversion, while
  `apply_joint_drives` applies `× π/180`. Both are in Isaac Sim 6.1.0.

A converter that does not *assert* its unit conventions will keep regressing them.

### G7 — Joint limits: silently locked joints, and no limit compliance

```python
limit_lower = joint.limit.get_with_default("lower") if joint.limit is not None else 0.0
limit_upper = joint.limit.get_with_default("upper") if joint.limit is not None else 0.0
```
(`link.py:424-425`). A `revolute` joint whose `<limit>` is missing — or present but
without `lower`/`upper` — becomes a joint with limits **[0, 0]**: welded shut, with
no warning. URDF requires `<limit>` on revolute joints, and real-world files omit
it often enough that this will bite.

Limit *compliance* is also unauthored: `newton:limitStiffness` /
`newton:limitDamping` are left unset (the MuJoCo converter documents this
deliberately, `joint.py:99-104`). PhysX treats USD limits as hard constraints;
MuJoCo derives soft limits from `solreflimit`. A joint driven into its stop
therefore bounces in one backend and sticks in the other.

`<limit effort>` reaches `DriveAPI.maxForce` only on the Isaac path; on the pure
Newton path it stays an inert `urdf:limit:effort` custom attribute, so drives (if
any downstream tool adds them) are torque-unbounded.

### G8 — Nested rigid bodies break maximal-coordinate consumers

Both Newton converters emit nested bodies and both READMEs warn about it. Isaac
preserves the nesting. Loading such an asset into a maximal-coordinate UsdPhysics
runtime yields either a flat pile of free bodies (kinematics lost) or double-counted
transforms (links at the wrong place, joints immediately violated → explosion on
frame 1).

### G9 — Two different fixed-joint merge passes with different semantics

The converter's ghost-link removal (`link_hierarchy.py`) drops *massless* fixed-joint
chains; Isaac's `merge_fixed_joints` (off by default) merges *all* fixed joints at the
XML level with correct parallel-axis inertia composition (`urdf_utils.py:474-520`,
though it returns early when the child mass is 0). Enabling both means two passes
with different rules run on the same model. Isaac's own docs list *"Robot flies
apart → `merge_fixed_joints: false` + stiff PD"* as a known pitfall — i.e. the
number of bodies in the articulation is itself a stability parameter, and nothing
chooses it for you.

### G10 — Nobody ever simulates the output

The converter's 408-model benchmark records success/time/size. Isaac's importer
tests call `timeline.play()` for one second and assert *"nothing crashes"*
(`test_urdf.py:221-225`). Isaac's Gain Tuner does run sinusoidal/step/snap/stress
tests with divergence detection and a dt sweep — interactively, in Kit, on one
robot at a time, after import, for PhysX/Newton.

**There is no headless, cross-backend, CI-runnable "does this robot stand still"
regression suite anywhere in this ecosystem.** That is the single largest gap,
and it is the one that makes all the others measurable.

---

## 5. Our position

### Option A — standalone converter

Reimplement URDF→USD end to end.

*Pros:* total control of the mapping; no dependence on Isaac's pinned 0.3.2 or on
upstream's roadmap; we could emit flat bodies natively.

*Cons:* we would reimplement ~4.7k lines of well-tested parsing, mesh loading
(STL/OBJ/DAE), material translation, `package://` resolution, ghost-link analysis,
and inertia eigendecomposition; we would have to keep up with a 408-model
compatibility benchmark we did not write; and we would fork the ecosystem at
exactly the moment it is converging on one converter. We would also lose the
strongest thing we have going for us — that our output is *the same asset*, only
stable.

### Option B — a stability layer over the existing converters  ← **recommended**

A library + CLI that consumes either (a) a URDF, by invoking
`urdf-usd-converter` itself, or (b) an already-converted USD asset — Newton
layout *or* Isaac 6.x package — analyses it, and authors a **separate stability
layer** of `over` prims plus a machine-readable report.

*Pros:*
- Every gap in §4 is an *additive* authoring problem: drives, armature, inertia
  repair, collision approximation and filtering, physics materials, limit
  compliance, scene defaults. All of it composes cleanly as `over` prims in a
  dedicated layer, which is exactly what USD is for.
- Non-destructive and auditable: the stability edits are one sublayer the user can
  mute, diff, or hand-tune. Nothing we do is hidden inside a converted asset.
- Isaac's namespace routing gives us 6.x compatibility for free — anything we
  author as `Physics.*`/`Newton.*`, `Physx.*`, `Mjc.*` lands in the right layer of
  their package automatically.
- We inherit the 408-model conversion benchmark as our input corpus instead of
  competing with it.
- Honest positioning: *"the converters are correct; the physics is
  underdetermined; here is the missing half, and here is the evidence it works."*

*Cons:*
- We depend on upstream's output contract. Mitigated by pinning and by a
  structure-detection layer that reads the composed stage, not file paths.
- We cannot fix what the converter loses before we see it — e.g. `<transmission>`
  data. Mitigated by optionally re-reading the source URDF alongside the USD.
- The version skew is real: Isaac pins 0.3.2, upstream is 0.3.3. Our repair layer
  must *detect and fix* 0.3.2's zero-`principalAxes` bug rather than assume 0.3.3.
  (That is a feature — it is an immediate, demonstrable win.)

### Recommendation

**Option B, with a thin "convert" front-end.** `urdf-usd-bridge convert robot.urdf`
runs `urdf-usd-converter` and then our stability pass, so the common case is one
command; `urdf-usd-bridge stabilize robot.usda` works on anything already
converted, including Isaac 6.x packages. We are a *stability and validation* layer,
not a competing converter, and we say so in the README.

Two hard rules for the design:

1. **Never silently change dynamics.** Every repair is recorded in the report with
   the before value, the after value, the rule that fired, and its severity.
   `--check` mode reports and exits non-zero without writing.
2. **Every rule must be justified by a measurement.** If we cannot demonstrate a
   metric improving on a hold-pose or drop test, the rule does not ship.

---

## 6. Proposed architecture

### 6.1 Package layout

```
urdf-usd-bridge/
├── pyproject.toml
├── README.md                     # acknowledges newton-physics + Isaac Sim
├── LICENSE                       # Apache-2.0
├── THIRD_PARTY.md                # every adapted file, with upstream commit
├── CLAUDE.md
├── docs/
│   ├── ANALYSIS.md               # this file
│   ├── rules.md                  # one page per stability rule
│   └── compatibility.md          # tested version matrix
├── src/urdf_usd_bridge/
│   ├── __init__.py
│   ├── __main__.py
│   ├── cli.py                    # convert / stabilize / check / test
│   ├── convert.py                # thin wrapper over urdf_usd_converter
│   ├── model/
│   │   ├── stage.py              # structure detection: Newton vs Isaac 6.x vs flat
│   │   ├── articulation.py       # links, joints, tree, accumulated inertia
│   │   └── units.py              # rad/deg, kg, m — one place, asserted
│   ├── diagnose/                 # PURE: read-only, returns Findings
│   │   ├── base.py               # Finding(code, severity, prim, message, evidence)
│   │   ├── inertia.py            # G2: zero, negative, triangle inequality, mass ratio
│   │   ├── drives.py             # G1: missing/zero gains, unit drift, maxForce
│   │   ├── armature.py           # G3
│   │   ├── collision.py          # G4: hull overlap at rest, thin plates, filtering
│   │   ├── materials.py          # G5
│   │   ├── limits.py             # G7: [0,0] locked joints, missing compliance
│   │   └── structure.py          # G8/G9: nesting, body count, articulation roots
│   ├── repair/                   # authors `over` prims into the stability layer
│   │   ├── layer.py              # create/attach Stability.usda; provenance metadata
│   │   ├── inertia.py            # geometry-derived tensor, PSD projection, floors
│   │   ├── drives.py             # gains from accumulated inertia + target ω_n, ζ
│   │   ├── armature.py           # from <transmission> or an inertia fraction
│   │   ├── collision.py          # approximation choice, filteredPairs, offsets
│   │   ├── materials.py          # explicit MaterialAPI so backends agree
│   │   ├── limits.py             # limit stiffness/damping, effort → maxForce
│   │   └── scene.py              # PhysicsScene + per-backend solver defaults
│   ├── backends/                 # one adapter per target; author + simulate
│   │   ├── base.py               # Backend protocol: author(), simulate(), metrics()
│   │   ├── physx.py              # Physx.* schemas; sim via Isaac Sim (optional)
│   │   ├── mujoco.py             # Mjc.* schemas + MjcActuator; sim via mujoco
│   │   └── newton.py             # Newton.* schemas; sim via newton
│   ├── emit/
│   │   ├── newton_layout.py      # canonical: Physics.usda / Geometry.usda / ...
│   │   ├── isaac_package.py      # payloads/ + Physics variant (6.x compatible)
│   │   └── flat_bodies.py        # maximal-coordinate compatibility mode
│   └── report/
│       ├── model.py              # JSON schema for findings + metrics
│       └── render.py             # terminal, JSON, Markdown
└── tests/
    ├── unit/                     # per-rule, on synthetic URDF/USD
    ├── golden/                   # authored-layer snapshots (usda text)
    └── sim/                      # hold-pose / drop / limit / gain sweeps per backend
```

**Design constraint:** `diagnose/` is pure and has no backend imports, so
`urdf-usd-bridge check` runs anywhere with just `usd-core`. Backends and
simulators are optional extras.

### 6.2 The stability layer

Output adds one layer next to the existing physics content:

```
<robot>/Payload/Stability.usda        # (Newton layout)
<robot>/payloads/Physics/stability.usda   # (Isaac layout — sublayers physics.usda)
```

containing only `over` prims, plus a provenance dictionary on the default prim:
tool version, input digest, rule set version, and the full finding list. Muting the
layer restores the original asset exactly.

### 6.3 Dependencies

Runtime (core):
- `usd-core` — or `usd-exchange`, which vendors it, when we also want
  `usdex.core` authoring helpers. **Decision needed:** `usd-exchange` buys us
  name-validation and consistent authoring and matches both converters; it also
  drags a heavier wheel. *Leaning `usd-exchange`*, pinned to the 2.3.0 that Isaac
  6.1.0 ships, for exact interop.
- `numpy` (2.3.x, matching Isaac's 2.3.1)
- `typer` or `argparse` for the CLI (prefer stdlib `argparse` — one less pin)

Optional extras:
- `[convert]` → `urdf-usd-converter==0.3.2` (Isaac parity) with 0.3.3 supported
- `[mujoco]` → `mujoco>=3.11,<3.12`, `mujoco-usd-converter==0.5.0`
- `[newton]` → `newton[sim]==1.5.0`, `newton-usd-schemas==0.4.1`
- `[physx]` → nothing installable from PyPI; PhysX simulation requires Isaac Sim,
  so that backend is gated behind an `ISAAC_SIM_DIR` env var and skipped in CI
  unless a runner provides it.

Dev: `pytest`, `ruff`, `black`, `coverage`, `hypothesis` (property tests for the
inertia math), `pyyaml`.

Python: `>=3.10,<3.13` — the intersection of both converters' ranges. (Isaac's
standalone wheels want `>=3.12`; our core must not.)

### 6.4 Test plan

Four tiers. Tier 3 is the one that makes this project mean anything.

**Tier 0 — unit.** Pure-function tests on synthetic inputs: inertia PSD
projection, triangle-inequality repair, rad↔deg round-trips with asserted
conventions, accumulated-inertia computation against hand-worked examples,
gain derivation from (I, ω_n, ζ). Property-based tests (hypothesis) for "repaired
tensor is always PSD and satisfies the triangle inequality" and "unit conversions
round-trip to 1e-12".

**Tier 1 — golden layers.** Convert a small fixed set of URDFs, run the stability
pass, and snapshot the authored `Stability.usda` as text. Catches unintended
authoring changes and makes every rule's effect reviewable in a diff.

**Tier 2 — corpus conversion.** Run `check` (diagnose-only, no simulation) across
the public corpora the Newton benchmark already uses — **SO-ARM100**,
**example-robot-data**, **newton-assets**, **urdf_files_dataset** — and publish a
findings table: how many models have zero inertia, locked `[0,0]` joints,
overlapping hulls at rest, no drives. This doubles as the project's evidence base
and costs no GPU.

**Tier 3 — simulation regression, per backend.** A fixed, named scene: gravity
−9.81 m/s² on Z, ground plane at z=0, fixed timestep, default solver settings
recorded in the report. For each robot × backend × {baseline, stabilized}:

| Test | Setup | Metrics |
|---|---|---|
| **hold-pose** | fixed base, joints at home pose, position targets = home, 5 s | max joint-position drift (rad), RMS joint velocity, peak joint velocity, peak contact impulse, energy drift, NaN/divergence flag |
| **drop** | floating base, dropped 10 cm, 5 s | settle time to \|v\|<1e-3, final base height error, bodies-penetrating count at rest, max penetration depth, NaN flag |
| **limit sweep** | each joint commanded past both limits, one at a time | overshoot past limit, chatter frequency at the stop, residual velocity |
| **gain sweep** | ω_n ∈ {2, 5, 10, 20} Hz × ζ ∈ {0.7, 1.0} | step-response settle time, overshoot, divergence threshold |
| **dt sweep** | dt ∈ {1/60, 1/120, 1/240, 1/500} | largest dt that survives hold-pose — the asset's *stability margin* |

**Pass criteria** are relative, not absolute: the stabilized asset must be
no worse than baseline on every metric, and must fix at least one previously
diverging case per robot class. Cross-backend agreement gets its own metric —
final base height and settle time should agree within a stated tolerance across
PhysX / MuJoCo / Newton — because "the same asset behaves the same everywhere" is
the actual product claim.

**Runner:** `urdf-usd-bridge test --backend mujoco,newton --suite hold-pose,drop
--out report.json`, with a Markdown renderer for the repo, mirroring the style of
the converter's own `benchmarks.md`. MuJoCo and Newton run headless on CPU/GPU in
CI; PhysX is opt-in and skipped when Isaac Sim is absent.

---

## 7. Licensing constraints and reuse plan

| Source | License | Reuse |
|---|---|---|
| `urdf-usd-converter/**` | Apache-2.0 (`LICENSE.md`; SPDX headers on every `.py`) | OK to adapt with attribution |
| `mujoco-usd-converter/**` | Apache-2.0 | OK to adapt with attribution |
| `IsaacSim/**/*.py` (importer, utils, transformer rules) | Apache-2.0 SPDX headers throughout — **0 Python files** carry a proprietary marker | OK to adapt with attribution |
| `isaacsim.asset.importer.urdf/config/extension.toml` | **`LicenseRef-NvidiaProprietary`** | **Do not copy** |
| `isaacsim.asset.importer.urdf.ui/config/extension.toml` | **`LicenseRef-NvidiaProprietary`** | **Do not copy** |
| `IsaacSim/LICENSE` (repo root) | Apache-2.0 **plus** a notice that building/using requires components under NVIDIA's *additional* license (Kit SDK, 3D models, textures) | Never vendor Kit, models, or textures |
| `IsaacSim/**/data/tests/**/*.usd*` | Git-LFS test assets; covered by the additional-materials notice | **Do not copy** |
| `isaacsim_structure.json` and the `*_conversion.json` profiles | No header; in an Apache-2.0 repo | Treat as Apache-2.0; if we ship a derived profile, attribute it in `THIRD_PARTY.md` |

Concrete plan: we expect to **adapt** (not copy wholesale) the unit-conversion
helpers and the MJC-actuator gain/bias derivation from
`urdf_to_mjc_physx_conversion_utils.py`, and the ω_n/ζ ↔ (k, d) math from
`gain_tuner_drive_math.py`. Both are Apache-2.0 NVIDIA files; each adapted file
keeps its original `SPDX-FileCopyrightText` header, gains a
`Modified by urdf-usd-bridge contributors` line, and gets a `THIRD_PARTY.md` row
recording the upstream path, the tag (`v6.1.0`), the commit
(`7c206f75bdadd9e05fc457f19863ca4c3f0cb693`), and what changed.

The README will acknowledge `newton-physics/urdf-usd-converter` and Isaac Sim as
prior art and interop targets, and will state plainly that the project is not
endorsed by or affiliated with Newton, NVIDIA, Google DeepMind, or Disney.

---

## 8. Open questions to settle before writing code

1. **`usd-exchange` or bare `usd-core` as the core dependency?** Interop and
   authoring quality argue for `usd-exchange==2.3.0`; install weight argues for
   `usd-core`. Needs a decision — it shapes `emit/`.
2. **Does Isaac Sim 5.1.0 load a 6.x-layout package at all?** Unverified; needs a
   5.1.0 runtime. If not, the `flat-bodies` mode is our only 5.1.0 story and we
   should say so rather than imply compatibility.
3. **How does `geometries.usd` / `instances.usda` sublayer into `base.usd`?** The
   custom connections in the profile only wire physx/mujoco→physics and
   base→robot; the geometry and material wiring is inside
   `perf/geometries.py` (3.8k lines) and `perf/materials.py`. Confirm empirically
   by running the importer once, rather than by further reading.
4. **Confirm the G1 regression on a live Isaac Sim 6.1.0.** The static evidence is
   conclusive (the attribute names do not intersect), but a one-line reproduction —
   import a URDF with `<dynamics damping="1.0">` and read back
   `drive:angular:physics:damping` — is worth having before we publish the claim,
   and is the basis for an upstream issue.
5. **Scope of the first release.** Proposal: G1 (drives), G2 (inertia), G7
   (limits) plus the hold-pose and drop suites on MuJoCo and Newton. G3 (armature)
   and G4 (collision) land second; PhysX simulation lands when a runner has
   Isaac Sim.
