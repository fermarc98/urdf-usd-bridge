# Upstream issues, ready to file

Three reports: two against **Isaac Sim** and one against **Newton**, all found
while building this project and all reproducible.

## Filing status

| # | Target | Title | Status | URL |
|---|---|---|---|---|
| 1 | `isaac-sim/IsaacSim` | URDF `<dynamics damping>` and `<dynamics friction>` are silently dropped | **not filed** | — |
| 2 | `isaac-sim/IsaacSim` | degree/radian errors converting PhysX drives to MuJoCo actuators | **not filed** | — |
| 3 | `newton-physics/newton` | `SolverFeatherstone` diverges on serial arms and humanoids | **not filed** | — |

None of these has been filed yet: the machine this was developed on has no
`gh` and no GitHub credentials, and filing posts to a public tracker under a
real account, which is not something to do on someone's behalf without their
say-so. Everything needed is ready:

```bash
python scripts/file_upstream_issues.py          # write the bodies, print the gh commands
python scripts/file_upstream_issues.py --file   # file them (needs gh, and gh auth login)
```

Paste each URL into the table above when filed.
``tests/unit/test_upstream_issues.py`` checks the table lists every issue in
this document, so the two cannot drift apart. Neither has been filed yet; each section below is meant to be
pasted into an issue tracker with only the placeholders removed.

Both concern `isaacsim.asset.importer.urdf` and its helper library
`isaacsim.asset.importer.utils`. Both are Apache-2.0 source. Nothing in this
document reproduces code beyond the short excerpts needed to identify the
defect.

**Tested build:** Isaac Sim **6.1.0-rc.26**, `release.49347.2d230af4.gl`,
installed from the standalone Linux package. It is a release candidate, not the
final 6.1.0 tag (`v6.1.0` = `7c206f75bdadd9e05fc457f19863ca4c3f0cb693`); the
source lines cited below are identical in the tag and in the installed build's
`pip_prebundle`, but anyone reproducing this should say which build they used.

---

## Issue 1 — URDF `<dynamics damping>` and `<dynamics friction>` are silently dropped

**Component:** `isaacsim.asset.importer.urdf` (extension 3.11.10) /
`isaacsim.asset.importer.utils`
**Affected:** Isaac Sim 6.1.0 (present in 6.1.0-rc.26). Isaac Sim 6.0.1 is
believed unaffected; see *Not yet verified* below.
**Severity:** silent data loss — no error, no warning naming the cause, and the
resulting robot simulates differently in all three supported backends.

### Summary

Importing a URDF whose joints carry `<dynamics damping="..." friction="...">`
produces an asset in which the PhysX drive has no damping, `physxJoint:jointFriction`
is never set, and every `MjcActuator` is created without gain parameters. The
values are not lost by the converter — they are present in the asset under a
different attribute name than the Isaac-side reader looks for.

### Root cause

`urdf_to_mjc_physx_conversion_utils.convert_urdf_to_physx()` reads:

```python
joint.GetAttribute("urdf:dynamics:damping")     # line 205
joint.GetAttribute("urdf:dynamics:friction")    # line 214
```

`urdf-usd-converter` **0.1.3** authored exactly those attributes. Commit
`99ff034` ("Updated to newton schemas v0.4.0", first released in **0.3.0**)
removed both `CreateAttribute` calls and replaced them with `newton:damping` and
`newton:friction`.

Isaac Sim 6.1.0 pins `urdf-usd-converter==0.3.2`
(`deps/pip_urdf_usd.toml:19`, `isaacsim.asset.importer.urdf/pyproject.toml:23`,
`python_packages.toml:338`; confirmed in the installed build at
`exts/isaacsim.asset.importer.urdf/pip_prebundle/urdf_usd_converter-0.3.2.dist-info`).

So the set of attributes the converter writes and the set the importer reads do
not intersect. `git grep 'urdf:dynamics' v0.3.2` in the converter returns
nothing.

The extension's own unit test passes because
`isaacsim/asset/transformer/rules/tests/test_urdf_to_mjc_physx_conversion.py:50`
hand-authors `urdf:dynamics:damping` onto a synthetic stage; it never runs the
real converter.

### Reproduce

Minimal URDF — one revolute joint with damping and friction:

```xml
<robot name="damping_demo">
  <link name="base_link">
    <inertial><mass value="5.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/></inertial>
    <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision>
  </link>
  <link name="arm_link">
    <inertial><mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/></inertial>
    <visual><geometry><box size="0.05 0.05 0.3"/></geometry></visual>
    <collision><geometry><box size="0.05 0.05 0.3"/></geometry></collision>
  </link>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="87.0" velocity="2.0"/>
    <dynamics damping="1.5" friction="0.3"/>
  </joint>
</robot>
```

Import it with a stock `URDFImporterConfig` and read the result back. Note that
the importer cannot be imported from a bare `python.sh`: `isaacsim` is a regular
package with a fixed `__path__`, and only Kit's extension manager joins the
per-extension trees, so a headless `SimulationApp` has to start first.

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.kit.app
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.asset.importer.urdf", True
)
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from pxr import Usd, UsdPhysics

asset = URDFImporter(
    URDFImporterConfig(urdf_path="damping_demo.urdf", usd_path="/tmp/out")
).import_urdf()

stage = Usd.Stage.Open(asset, Usd.Stage.LoadAll)
# 6.1.0 authors no default Physics selection, so one must be chosen.
stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
joint = stage.GetPrimAtPath("/damping_demo/Physics/shoulder_joint")
drive = UsdPhysics.DriveAPI(joint, "angular")
print("drive damping  :", drive.GetDampingAttr().Get())
print("drive stiffness:", drive.GetStiffnessAttr().Get())
print("drive maxForce :", drive.GetMaxForceAttr().Get())
print("newton:damping :", joint.GetAttribute("newton:damping").Get())
print("physxJoint:jointFriction:", joint.GetAttribute("physxJoint:jointFriction").Get())
app.close()
```

### Expected

`<dynamics damping="1.5">` reaches the PhysX drive (`drive:angular:physics:damping`,
converted to USD's per-degree convention, i.e. `1.5 * pi/180 = 0.0261799`), and
`<dynamics friction="0.3">` reaches `physxJoint:jointFriction`.

### Actual (observed on 6.1.0-rc.26)

| Reading | Value |
|---|---|
| `drive:angular:physics:damping` | **not authored** |
| `drive:angular:physics:stiffness` | **not authored** |
| `drive:angular:physics:maxForce` | `87.0` |
| `newton:damping` | `0.026179939508` — the value *is* in the asset |
| `newton:friction` | `0.3` |
| `urdf:dynamics:damping` | absent — the attribute the importer reads |
| `physxJoint:jointFriction` | not authored |
| `mjc:gainPrm` / `mjc:biasPrm` on the actuator | not authored |

`maxForce = 87.0` arriving while stiffness and damping do not is the clearest
single symptom: `urdf:limit:effort` is the one URDF attribute whose spelling
still connects.

Isaac Sim reports the consequence itself, at default settings:

```
[Warning] [isaacsim.asset.importer.utils.impl.urdf_to_mjc_physx_conversion_utils]
Stiffness and damping not available joint /damping_demo/Physics/shoulder_joint,
actuator will be created without gain parameters
```

### Control

Re-importing with `override_joint_stiffness=800`, `override_joint_damping=40`
**does** populate the drive (`drive:angular:physics:damping = 0.698132`,
i.e. `40 * pi/180`). So the drive is writable and readable, and the default path
is genuinely dropping the URDF's value rather than the inspection being wrong.

### Impact per backend

* **PhysX** — the drive contributes nothing. A fixed-base arm released at its
  home pose collapses under gravity onto its joint limits.
* **MuJoCo** — the `MjcActuator` keeps its schema default `gainPrm = [1, 0, ...]`
  with `biasType = "none"`, which is a unit-gain force actuator, not a PD
  controller. Position targets do nothing and `ctrl = 0` means zero torque.
* **Newton** — `newton:damping` *is* present, so the joint is viscous but
  unactuated: it sags slowly instead of dropping.

One asset, three different wrong behaviours. The divergence is arguably worse
than the loss.

### Suggested fix

Either update `convert_urdf_to_physx()` to read `newton:damping` /
`newton:friction` (with the per-degree to per-degree identity for damping, and
no conversion for friction, which is an effort), or have the extension's own
unit test exercise the pinned converter rather than a hand-authored stage — the
latter would have caught this at the version bump.

### Not yet verified

Isaac Sim **6.0.1** pinned `urdf-usd-converter==0.1.3`, whose attribute names do
intersect with what `convert_urdf_to_physx()` reads, so 6.0.1 is expected to be
unaffected. That half has **not** been reproduced at runtime here: only 6.1.0 is
installed on the machine used. The evidence for 6.0.1 is source-level
(`deps/pip_urdf_usd.toml` at tag `v6.0.1` = `045ca8b59622b99a408092124377c66346e8d9c2`).

---

## Issue 2 — degree/radian errors converting PhysX drives to MuJoCo actuators

**Component:** `isaacsim.asset.importer.utils`,
`urdf_to_mjc_physx_conversion_utils.py`
**Affected:** Isaac Sim 6.1.0 (present in 6.1.0-rc.26).
**Severity:** 2a is **reachable today** via `override_joint_stiffness` /
`override_joint_damping`, which is the documented workaround for Issue 1 — so
the workaround produces a correct PhysX drive and a 57.3x-too-soft MuJoCo
actuator from the same import. It is reproduced below. 2b is latent **and is a
source reading that was not reproduced** — see its own status note. Both become
live for the default path as soon as Issue 1 is fixed.

### Summary

Two values are copied from `UsdPhysics` attributes into MJC attributes without
the unit conversion the two conventions require. Each is wrong by a factor of
`180/pi = 57.29577951`.

### 2a — actuator gains

`create_mjc_actuator_from_physics()`, lines 279–287:

```python
stiffness = drive_api.GetStiffnessAttr().Get() ...
damping   = drive_api.GetDampingAttr().Get() ...

if stiffness > 0 and damping > 0:
    gain_prm = [stiffness, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    bias_prm = [0, -stiffness, -damping, 0, 0, 0, 0, 0, 0, 0]
```

`UsdPhysics` declares a rotational drive's stiffness as
`mass*DIST*DIST/degrees/second/second` and its damping as
`mass*DIST*DIST/second/degrees` — both **per degree**. Isaac's own
`asset_utils._set_stiffness_on_joints` applies `value * math.pi / 180.0` when
writing them, and `gain_tuner_drive_math.stored_gain_scale` documents the same
scale.

`mjc:gainPrm` and `mjc:biasPrm` mirror MJCF `gainprm`/`biasprm`, which are in
MuJoCo's native units — **per radian** for a hinge joint. `mujoco-usd-converter`
makes the distinction explicit in the opposite direction: it writes MuJoCo's
value unscaled to `mjc:damping` (`joint.py:83`) and multiplies by `pi/180` for
`newton:damping` (`joint.py:97,108`).

So a drive tuned correctly for PhysX becomes **57.3x too soft** in MuJoCo.

### 2b — actuator reference position (**source reading only — NOT reproduced**)

> This sub-issue is a reading of the source. It was **not** observed on a
> running Isaac Sim, for the reason given below. File it as such, or split it
> out and drop it if the maintainers would rather see only reproduced defects.


`convert_physx_to_mjc()`, line 337:

```python
joint.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(target_position)
```

`drive:*:physics:targetPosition` is in **degrees** for a rotational drive;
`mjc:ref` is MJCF's `ref`, in **radians**. A 90 degree target becomes 90 radians.

Note also that `mjc:ref` and `mjc:forceRange:*` are created here as `Float`,
while the MJC schema declares them `uniform double`
(`omni.usd.schema.mujoco` `generatedSchema.usda`). That is cosmetic next to the
unit error, but it is the same line.

### Reproduce

Import the URDF from Issue 1 with explicit gains, then compare the two
representations of the same gain:

```python
asset = URDFImporter(URDFImporterConfig(
    urdf_path="damping_demo.urdf", usd_path="/tmp/out",
    joint_drive_type="force", joint_target_type="position",
    override_joint_stiffness=800.0, override_joint_damping=40.0,
)).import_urdf()

stage = Usd.Stage.Open(asset, Usd.Stage.LoadAll)
vset = stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics")

vset.SetVariantSelection("physx")
drive = UsdPhysics.DriveAPI(
    stage.GetPrimAtPath("/damping_demo/Physics/shoulder_joint"), "angular")
stored = drive.GetStiffnessAttr().Get()          # per degree

vset.SetVariantSelection("mujoco")
actuator = stage.GetPrimAtPath("/damping_demo/Physics/shoulder_joint_actuator")
gain = actuator.GetAttribute("mjc:gainPrm").Get()[0]   # per radian

print(stored, gain, gain / stored)
```

### Expected

`gain / stored == 180/pi ≈ 57.2958`, because the same physical gain is being
expressed in two conventions.

### Actual — confirmed at runtime on 6.1.0-rc.26

```
physx   drive:angular:physics:stiffness = 13.962634    (= 800 * pi/180, correct)
physx   drive:angular:physics:damping   =  0.698132    (=  40 * pi/180, correct)
mujoco  mjc:gainPrm                     = [13.962634, 0, 0, ...]
mujoco  mjc:biasPrm                     = [0, -13.962634, -0.698132, ...]

ratio gain / stored = 1.000000     (expected 57.295780)
```

The user asked for `stiffness = 800 N*m/rad`. `apply_joint_drives` converts that
to USD's per-degree convention correctly. `create_mjc_actuator_from_physics`
then copies the per-degree number into the per-radian MJCF slot, so MuJoCo is
told the gain is **13.96 N*m/rad** — 57.3x softer than requested.

### 2b status: source reading, not reproduced

The `mjc:ref` defect was **not** reproduced in the run above.
`convert_physx_to_mjc` guards the write with `if target_position:`, and the
default target is `0.0`, which is falsy — so the line never executes in the
runs performed here. A non-zero `urdf:calibration:reference_position`, or any
caller setting a non-zero drive target before the MJC pass, would exercise it.

To be explicit about the evidence for each part of this report:

| Claim | Status |
|---|---|
| 2a — actuator gains copied per-degree into a per-radian slot | **Reproduced** on 6.1.0-rc.26; measured ratio 1.000000 where 57.295780 was expected |
| 2b — `mjc:ref` written in degrees into a radian attribute | **Source reading only.** Not executed, not observed |

### Suggested fix

In `create_mjc_actuator_from_physics`, scale by `180/pi` for rotational joints
before writing `gain_prm`/`bias_prm`, and leave prismatic joints unscaled. In
`convert_physx_to_mjc`, apply `math.radians()` to `target_position` before
writing `mjc:ref`. A regression test asserting the `180/pi` ratio between the
two representations would pin both, and would not pass with either defect
present.

---

## Issue 3 — `SolverFeatherstone` diverges on serial arms and humanoids that `SolverMuJoCo` simulates fine

**Project:** `newton-physics/newton`
**Component:** `newton.solvers.SolverFeatherstone`
**Version:** Newton 1.5.0, Warp 1.16.0 (as bundled in Isaac Sim 6.1.0-rc.26)
**Severity:** the solver produces NaN within 0.1 s of simulated time on a
6-DoF arm converted from a public URDF, with no contacts and no actuation.

### Summary

Across a 14-robot corpus converted from public URDFs, `SolverFeatherstone`
diverges within 0.03–0.27 s on **every serial arm and humanoid tried**, while
`SolverMuJoCo` — given the identical `newton.Model` from the same
`ModelBuilder.add_usd` call — is stable on all of them.

The split is by morphology, not by size:

| Morphology | Robots | Featherstone | MuJoCo |
|---|---|---|---|
| Quadruped (12 DoF) | a1, aliengo, b2, go1, go2, laikago | **all 6 stable** | all stable |
| Serial arm (6 DoF) | so100, so101, z1 | **all 3 diverge** | all stable |
| Humanoid (19, 23 DoF) | h1, g1 | **both diverge** | both stable |
| Small fixture (2–3 DoF) | 3 fixtures | all stable | all stable |

A 12-DoF quadruped converges while a 6-DoF arm does not, so this is not a
DoF-count or problem-size effect. All of these are with drive gains authored;
the same robots also diverge as the converter emits them, with no gains at
all.

Reducing the timestep sixteenfold does not help, which is what distinguishes
this from ordinary explicit-integration instability.

### Reproduce

```python
import newton, numpy as np

builder = newton.ModelBuilder()
builder.add_usd("so101_new_calib.usda", collapse_fixed_joints=False,
                enable_self_collisions=False)
model = builder.finalize()

solver = newton.solvers.SolverFeatherstone(model)   # swap for SolverMuJoCo: stable
state_0, state_1 = model.state(), model.state()
control = model.control()

for step in range(480):
    state_0.clear_forces()
    solver.step(state_0, state_1, control, None, 1 / 240)
    state_0, state_1 = state_1, state_0
    q = state_0.joint_q.numpy()
    if not np.all(np.isfinite(q)) or np.abs(q).max() > 1e3:
        print("diverged at t =", step / 240); break
```

No ground plane, no contacts, no drive gains needed — `control` is left at its
defaults.

### What was ruled out

Each row is a measurement, not a guess.

| Hypothesis | Test | Result |
|---|---|---|
| Our drive gains cause it | zero `joint_target_ke`/`kd` | **still diverges** (t=0.267 s) |
| Timestep / explicit integration | dt 1/240 → 1/1000 → 1/4000 | **still diverges** (0.0875 → 0.172 → 0.177 s) |
| The 1e-9 kg marker link | raise its mass to 1e-6 … 1e-2 kg | **no change** at 1e-6…1e-3; 1e-2 delays by 6 steps |
| Mass-ratio conditioning | `collapse_fixed_joints=True` drops the ratio from 1.47e8 to 8.67 | **diverges at the identical time**, 0.0875 s |
| It is specific to that robot | SO-100, mass ratio 49, no marker link | **also diverges**, t=0.0333 s |
| Gravity | gravity disabled | **stable** |
| Joint limits | limits widened to ±1e6 | delayed 0.086 s → 0.863 s |
| Joint-space regularisation | `joint_armature = 1e-1` for every DOF | **stable** |

The model's own body inertias span 1e-6 to 1.6e-4 kg·m², so the armature that
suppresses the divergence is roughly **1000× the inertia of the bodies it is
regularising** — enough to change the robot's dynamics completely. It is
evidence about conditioning, not a usable workaround.

### Expected

`SolverFeatherstone` either simulates the model, or rejects it with a
diagnosable error. A silent NaN 21 steps in, on a model the MuJoCo backend
handles, is hard to act on.

### Actual

| Robot | bodies | mass ratio | Featherstone | MuJoCo |
|---|---|---|---|---|
| SO-101 (repaired) | 8 | 1.47e8 | NaN at 0.0875 s | stable |
| SO-101 (converter output) | 8 | 1.47e8 | NaN at 0.2625 s | stable |
| SO-100 (repaired) | 7 | 49 | NaN at 0.0958 s | stable |
| SO-100 (converter output) | 7 | 49 | NaN at 0.0333 s | stable |
| Unitree Z1 (6-DoF arm) | — | — | NaN | stable |
| Unitree H1 (19-DoF humanoid) | — | — | NaN | stable |
| Unitree G1 (23-DoF humanoid) | — | — | NaN | stable |
| Unitree Go2 / A1 / B2 / Go1 / AlienGo / Laikago | — | — | **stable** | stable |

`ModelBuilder.finalize()` also emits `Inertia validation corrected 1 bodies` for
SO-101, so the solver is already aware the model needs repair; it just does not
survive it.

### What we cannot say

We have not identified the mechanism inside the solver. Everything above is
black-box: what changes the time-to-divergence and what does not. The
combination "smaller dt does not help", "the MuJoCo backend handles the same
`Model`", and "only a physically implausible armature suppresses it" points at
the reduced-coordinate mass-matrix or limit-constraint handling rather than at
integration, but that is an inference from outside, not a diagnosis.

The morphology split is the strongest clue we have and we cannot explain it:
quadrupeds are several short chains from one base, while the failing robots are
long serial chains, which is the case a reduced-coordinate articulated-body
algorithm should handle best.

It is also possible the models are simply outside what Featherstone is expected
to handle. If so, an explicit error at `finalize()` or `SolverFeatherstone()`
construction would be far more useful than a NaN, and this issue becomes a
feature request for that check.

### Assets

Both URDFs are public and Apache-2.0:
`https://github.com/TheRobotStudio/SO-ARM100` at `eecbe3e0a9eb`,
`Simulation/SO101/so101_new_calib.urdf` and `Simulation/SO100/so100.urdf`,
converted with `urdf-usd-converter==0.3.2`.

---

## How these were found

`urdf-usd-bridge` is a stability layer that authors drive gains, armature,
inertia and joint-limit repairs over converted URDF assets. It has to write the
same physical gain into three backends with three different angle conventions,
so it carries a unit table
(`docs/PHASE3_DESIGN.md` section 3) and a regression test that asserts the
`180/pi` relationship between them
(`tests/unit/test_repair_units.py`). Issue 2a fell straight out of writing that
table and was then confirmed against a real import; Issue 1 was confirmed at runtime by
`scripts/verify_isaac_regression.py`, which returns `REGRESSION_CONFIRMED`
together with the JSON evidence attached above.

Issue 3 came out of Phase 4's cross-backend matrix: the same repaired asset
held its pose in PhysX and MuJoCo and produced NaN in Newton, and the
isolation table above is the Phase 5 follow-up.

None of these is a criticism of the converter, which is doing what it
documents. Issue 1 is a version-skew problem between two components that were
updated independently, and the pattern that would have caught it — testing
against the pinned dependency rather than a synthetic stage — is cheap.
