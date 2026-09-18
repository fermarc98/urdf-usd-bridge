# API reference

Three entry points, in the order you would use them: **inspect** what an asset
carries, **fix** what it is missing, and **measure** whether that helped.

Everything here codes against `pxr` (OpenUSD) only. `inspect` and `fix` need no
GPU; `sim` needs one.

---

## CLI

### `urdf-usd-bridge inspect <asset>`

Read-only. Reports what physics data an asset actually carries, per namespace,
never collapsing two spellings into one number.

| Option | Meaning |
|---|---|
| `--variant SET=SELECTION` | applied before reading; Isaac Sim 6.1.0 packages author no default `Physics` selection, so an asset opened as shipped has no physics at all |
| `--json` | the raw report, `schema_version: 1` |
| `-o PATH` | write instead of printing |
| `-v` | per-namespace joint table |

Exit codes: `0` read it, `1` could not open it, `2` usage or missing OpenUSD.

### `urdf-usd-bridge fix <asset> --out DIR`

Authors the repairs as `over` prims in new layers above an untouched input.

| Option | Default | Meaning |
|---|---|---|
| `--backend` | `all` | `physx`, `mujoco`, `newton`, a comma-separated subset, or `all`. `all` needs a `Physics` variant set to keep the per-backend gain conventions apart, and is refused without one |
| `--out DIR` | — | required unless `--dry-run` |
| `--dry-run` | off | run every rule, write nothing. Same code path, so it cannot disagree with a real run |
| `--json`, `-o`, `-v` | | as for `inspect` |
| `--enable RULE`, `--disable RULE` | | repeatable; an unknown rule name is an error, never ignored |
| `--force` | off | overwrite values the input authored *and* that validate |
| `--force-unlock` | off | unlock `[0,0]` joints even where a `<limit>` demonstrably existed. Off because the range is then a guess; on, because MuJoCo refuses to compile an asset containing one |
| `--newton-actuator` | off | also author `NewtonActuator`. Newton 1.5.0 does not read it, and a release that starts to would drive the joint twice |
| `--target-frequency HZ` | derived | see [tuning](#tuning) |
| `--damping-ratio`, `--armature-fraction`, `--armature-floor`, `--control-rate` | measured | see [tuning](#tuning) |

Exit codes: `0` clean, `1` an `error`-severity finding remains (a massless body,
an ambiguous locked joint), `2` usage.

### `urdf-usd-bridge convert <urdf> <out_dir>`

`urdf-usd-converter` followed by `fix`. Takes every `fix` option, plus
`--no-fix` to stop after conversion and `--out` for where the stability layers
go (default `<out_dir>/stability`).

Converter output never has a `Physics` variant set, so `--backend all` here
writes **one stabilized root per backend** rather than refusing.

### `scripts/run_sim_matrix.py`

The cross-backend harness. Needs a GPU and Isaac Sim.

```bash
<isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts --json results.json
```

`--backend`, `--suite`, `--asset` (repeatable), `--dt`, `--no-public`,
`--exploratory`, `--converter-python`, `--no-simulation-app`; `--sweep` with
`--frequencies`, `--zetas`, `--alphas`, `--dts` runs the tuning sweep instead.

---

## Tuning

Five constants govern the derived gains. Four are measured; one is not, and
says so in every report.

| Constant | Default | Provenance |
|---|---|---|
| `target_frequency` | **derived** | `control_rate / 6` for `physx` or `mujoco` alone, `control_rate / 12` for `newton` or any multi-backend asset |
| `damping_ratio` | 1.0 | measured: minimises step settle time with zero overshoot |
| `armature_fraction` | 0.01 | measured to have **no effect** on the stability margin; retained |
| `armature_floor` | 1e-4 | **unmeasured** — the low-inertia case it exists for was not exercised |
| `control_rate` | 60 Hz | the assumption the frequency is derived from; checked, never used in a formula |

The divisors come from a dt sweep: PhysX and MuJoCo were stable at
`control_rate/6` in every cell, Newton's Featherstone solver diverged there and
needed `/12`. A cross-backend asset can only be as stiff as its least tolerant
consumer. `docs/history/PHASE4_REPORT.md` §5.

Every value, its basis, the cross-backend value for comparison, and the
previous defaults with the date they changed are written into the output root
layer's `customLayerData`.

---

## Python API

### Inspecting

```python
from urdf_usd_bridge.inspection import inspect_stage

report = inspect_stage("robot.usda", {"Physics": "physx"})
report["summary"]["joints_with_drive_gains"]      # 0 on converter output
report["joints"][0]["damping_by_namespace"]       # every spelling, separately
```

Readings are three-state — `None` (absent), `{"authored": False}` (schema
fallback), `{"authored": True}` (authored) — and the distinction is the point:
an authored zero and an unauthored one mean different things.

### Repairing

```python
from urdf_usd_bridge.repair import RepairOptions, fix_asset

report = fix_asset("robot.usda", "out/", RepairOptions(backends_requested="physx"))
report["summary"]["applied_by_rule"]
report["output"]["roots"]        # one, or one per backend
```

`analyse()` is the same pass without the writes, if you want the plan.

#### The repair record

Every rule evaluation produces one, including the ones that did nothing:

```json
{
  "rule": "drives.derive-gains", "status": "applied",
  "prim": "/robot/Physics/shoulder", "attribute": "drive:angular:physics:stiffness",
  "old": null, "old_state": "absent", "new": 0.869898, "units": "N*m/deg",
  "backend": "physx", "layer": "Stability_physx.usda",
  "reason": "...", "confidence": "medium", "forced": false,
  "evidence": {"I_eq": 0.0125, "armature": 0.000125, "K_si": 49.8415, "formula": "..."}
}
```

`status` ∈ `applied` | `skipped` | `reported` | `refused`.
`confidence` ∈ `high` | `medium` | `low`.
`severity` on report-only records ∈ `information` | `warning` | `error`.

### Simulating

```python
from urdf_usd_bridge.sim import metrics
from urdf_usd_bridge.sim.backends import get_backend, RunRequest
from urdf_usd_bridge.sim.scene import SceneSpec, check_gravity_loaded

backend = get_backend("newton")
usable, why = backend.available()          # a reason either way
traj = backend.run(RunRequest(asset="robot.usda", scene=SceneSpec()))
metrics.pose_drift(traj.q, targets)
```

`metrics` is pure — arrays in, numbers out — so it is testable without a GPU
and identical across backends. A difference between backends can never come
from a difference in how a metric was computed.

---

## The rules

| Rule | Default | What it does |
|---|---|---|
| `inertia.derive-from-geometry` | on | tensor from collision geometry when mass exists and inertia does not |
| `inertia.principal-axes-identity` | on | zero quaternion → identity (the converter 0.3.2 defect) |
| `inertia.make-physical` | on | clamp negatives, satisfy `I1 + I2 ≥ I3`, always by raising |
| `inertia.mass-floor` | **off** | reports a massless body rather than inventing a mass |
| `limits.restore-missing` | on | `[0,0]` with no `<limit>` evidence → unlimited |
| `limits.restore-missing-prismatic` | **off** | an unlimited rail can be worse than a welded one |
| `limits.report-ambiguous` | on | `[0,0]` *with* evidence → refuse, at `error` severity |
| `limits.compliance` | on | report only; still not derivable, see below |
| `armature.default` | on | `max(α·I_eq, β·I_eq_max)` in all three namespaces |
| `drives.mirror-passive` | on | `newton:damping`/`friction` → each backend's spelling, converted |
| `drives.derive-gains` | on | PD gains from `I_total` and a target frequency |
| `drives.no-drive-for-fixed` | on | records the deliberate skip |

`limits.compliance` stays report-only on purpose. Phase 4 measured the case
*for* writing it — driven joints overshoot their stops harder than undriven
ones in 6 of 20 cells — but the value still cannot be derived from the asset:
`newton:limitStiffness` is an effort per unit penetration, and converting
MuJoCo's `solreflimit` to it needs the effective inertia at `qpos0`, a
pose-dependent quantity that must not be baked into a static attribute.
`mujoco-usd-converter` declines it for the same reason.

---

## Output layout

```
out/
  <name>_stabilized.usda        subLayers = [Stability*.usda ..., <original>]
  Stability.usda                inertia, limits, newton:* passive dynamics
  Stability_physx.usda          DriveAPI gains, physxJoint:armature/jointFriction
  Stability_mujoco.usda         MjcActuator, mjc:damping/frictionloss/armature
  Stability_newton.usda         DriveAPI (what Newton 1.5.0 reads)
```

Earlier sublayers are stronger. The original is referenced by relative path and
never opened for write; muting the stability layers restores it exactly.

When the asset has a `Physics` variant set, per-backend opinions are authored
**inside the matching variant**, because Isaac Sim's `mujoco.usda` deliberately
deletes `PhysicsDriveAPI` and a flat root sublayer would override that deletion.

`defaultPrim`, `upAxis`, `metersPerUnit` and `kilogramsPerUnit` are root-layer
metadata and are **not** composed from sublayers, so `fix` copies all four. A
root that omits `defaultPrim` opens with no default prim, which silently
disables every variant selection.

---

## Units

One table, applied once, on the way out. Getting this wrong is a 57.3× error
that no smoke test catches, and it is a live bug in shipping software
(`docs/UPSTREAM_ISSUES.md` issue 2).

| Quantity | Attribute | Angular convention |
|---|---|---|
| passive damping | `newton:damping` | per **degree** |
| passive damping | `mjc:damping` | per **radian** |
| drive gains | `UsdPhysics.DriveAPI` | per **degree** |
| MuJoCo actuator gains | `mjc:gainPrm`, `mjc:biasPrm` | per **radian** |
| armature | `newton:`/`mjc:`/`physxJoint:armature` | **not** angle-scaled, kg·m² |
| Coulomb friction | `newton:friction` and friends | **not** angle-scaled, N·m |

`tests/unit/test_repair_units.py` asserts the `180/π` ratio between the
conventions and that quantities without an angle unit are **identical** across
namespaces — so both directions of the mistake fail the suite.
