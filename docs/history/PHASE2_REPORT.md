# Phase 2 report

> **Development record.** Written during the phase it describes and kept for
> provenance, not maintained since. Where it disagrees with the current
> documentation, the current documentation is right — start at
> [`docs/history/README.md`](README.md).

**Date:** 2026-09-17
**Host:** macOS 13.7.8, x86-64, Python 3.10.18 / 3.12.14, `usd-core` 26.8
**Scope:** scaffold plus reproducible evidence. No stability repairs implemented.

> **Superseded in part by Phase 2.5 (2026-09-17, Linux + RTX 4090).** Everything
> below §5 describes the macOS host and stands as the record of what was known
> then. The tiers it lists as blocked have since run: **91 tests pass, 0 skipped**,
> and the Isaac Sim 6.1.0 check returned `REGRESSION_CONFIRMED`. Read **§5** for
> the current per-claim status and `docs/VERIFY.md` for the commands and raw
> readings. §4.2's "the importer does not need Kit" is the one claim Phase 2.5
> found to be wrong; see §5.1.

---

## 1. Headline

Everything that can run on macOS runs and passes: **51 tests green, 0 failures,
ruff and black clean.** The G1 regression from `docs/history/ANALYSIS.md` is now backed
by **ten automated source-level assertions** that execute here, against the
pinned upstream tags.

The two things that cannot run here are both blocked by one packaging fact:
**`usd-exchange` publishes no macOS wheels**, for any version. Everything
downstream of it — `urdf-usd-converter`, the converter matrix, the Isaac Sim
importer — is therefore unreachable on this machine. That work is written,
committed, and marked, and runs unchanged on Linux.

---

## 2. What shipped

| Area | Files | Lines |
|---|---|---|
| Library (`src/urdf_usd_bridge/`) | 11 | 1,545 |
| Tests (`tests/`) | 8 + 3 fixtures | 1,490 |
| Scripts (`scripts/`) | 2 | 454 |
| Docs | `VERIFY.md`, this report | — |
| Packaging | `pyproject.toml` (uv), `LICENSE`, `README.md`, `THIRD_PARTY.md` | — |

### Design decisions taken

- **No hard USD dependency.** The core codes against `pxr` only. Extras `[core]`
  (`usd-core`, all platforms) and `[exchange]` (`usd-exchange`, Linux/Windows)
  let the consumer choose; `[convert]`, `[mujoco]`, `[newton]`, `[schemas]`
  gate the rest. `src/urdf_usd_bridge/usd.py` turns a missing `pxr` into an
  install hint rather than a traceback.
- **Inspection reads attributes by name, never through typed schema APIs.** An
  asset therefore inspects correctly with no `physx`, `mjc` or `newton` schema
  plugin registered — which matters, because that is the normal state outside
  Isaac Sim.
- **Absent / fallback / authored are three different states**, never collapsed.
  This is the distinction the whole G2 finding turns on, and the renderer shows
  it directly: `-` absent, `(0)` schema fallback, `0` authored zero.
- **Per-producer spellings are reported separately.** `newton:damping`,
  `mjc:damping`, `urdf:dynamics:damping` and `drive:*:physics:damping` each get
  their own column, because mismatched spellings between producer and consumer
  are themselves the bug class.

### `inspect` covers

Drives (per DriveAPI instance: type, stiffness, damping, maxForce, targets) ·
damping, friction and armature in every namespace · mass, density, centre of
mass, `physics:diagonalInertia`, `physics:principalAxes`, `newton:inertia` ·
inertia validity (authored-zero, undefined, negative, triangle inequality,
positive-semidefiniteness, Newton/USD disagreement, zero quaternion) · mass
ratio · joint limits including `LimitAPI` instances and locked `[0,0]` joints ·
limit compliance and velocity limits · mimic joints · collider approximations,
`filteredPairs`, collision groups, self-collision · physics materials and their
bindings · `MjcActuator` prims and whether they carry gains · articulation roots
· `PhysicsScene` · stage metrics · variant sets and selections · asset layout
detection. Output: aligned text table, or `--json` with `schema_version: 1`.

---

## 3. What ran on macOS, and passed

```console
$ .venv/bin/python -m pytest tests -q
51 passed, 40 skipped in 3.88s
$ .venv/bin/ruff check .   → All checks passed!
$ .venv/bin/black --check . → 25 files would be left unchanged
```

### 3.1 Static source evidence — 10 tests, all passing

`tests/unit/test_reference_sources.py` reads the pinned reference trees with
`git show <tag>:<path>` and asserts on upstream source. This is the part of the
G1 claim that does **not** need a GPU, and it is now automated:

| Test | Asserts |
|---|---|
| `test_modern_converter_authors_newton_namespace_not_urdf_dynamics` | at `v0.3.2` **and** `v0.3.3`, `link.py` writes `newton:damping`/`newton:friction` and does **not** write `urdf:dynamics:*` |
| `test_old_converter_did_author_urdf_dynamics` | at `v0.1.3` it **did** write them |
| `test_converter_authors_no_drive_api` | `git grep DriveAPI` over the converter package is empty at both tags |
| `test_isaac_61_reads_the_spelling_its_pinned_converter_no_longer_writes` | Isaac 6.1.0 pins `0.3.2`; Isaac reads `urdf:dynamics:damping`/`friction`; the intersection with what 0.3.2 writes is **empty**; `urdf:limit:effort` is the one attribute that still connects |
| `test_isaac_60_pinned_a_converter_whose_spelling_matched` | Isaac 6.0.1 pins `0.1.3`, where both names **do** intersect — which is what makes 6.1.0 a regression |
| `test_isaac_61_applies_drive_api_without_authoring_gains` | `add_joint_schemas()` calls `DriveAPI.Apply` and never `CreateStiffnessAttr`/`CreateDampingAttr` |
| `test_converter_032_can_author_principal_axes_without_an_inertia_tensor` | in `v0.3.2` `apply_inertial()` sets `principalAxes` from inside the `origin` branch; in `v0.3.3` it does not, and only the inertia-guarded branch sets it |
| `test_converter_authors_only_convex_hull_for_mesh_colliders` | `geometry.py` mentions `convexHull` and none of `convexDecomposition`/`boundingCube`/`boundingSphere`/`sdf` |

### 3.2 A newly verified detail

`physics:principalAxes`'s **schema fallback is `(0, 0, 0, 0)`** — an invalid zero
quaternion — confirmed against `usd-core` 26.8:

```console
$ python -c "from pxr import Usd,UsdPhysics; s=Usd.Stage.CreateInMemory(); \
    print(UsdPhysics.MassAPI.Apply(s.DefinePrim('/a','Xform')).GetPrincipalAxesAttr().Get())"
(0, 0, 0, 0)
```

This is the mechanism behind the 0.3.2 defect: `apply_inertial()` does
`Get()` on an unauthored `principalAxes` and multiplies, and quaternion
multiplication by zero is zero. Every `inspect --json` report now carries a
`schema_fallbacks` section recording this, so a reader never has to guess
whether a zero was authored or inherited.

### 3.3 Inspector tests — 41 tests

Against hand-authored stages in `tests/unit/builders.py`, shaped like known
producer output. These test **our tool**, and are labelled as such; they are not
evidence about upstream. Coverage includes every flag in §2, the CLI (JSON
round-trip, `-o`, bad arguments, missing asset, console script), the text
renderer, and the unit-conversion helpers (including that 1.5 N·m·s/rad is
0.0261799… N·m·s/deg — the 57.3× error that would be an instant explosion).

### 3.4 Fixtures

Three hand-written URDFs, `tests/fixtures/`, each with a negative control so a
test cannot pass for the wrong reason:

| Fixture | Targets | Control |
|---|---|---|
| `a_dynamics_damping.urdf` | G1 — `<dynamics damping="1.5" friction="0.3">` on a revolute joint | prismatic joint, linear units, no angular rescale |
| `b_inertial_origin_mass_no_inertia.urdf` | G2 — origin + mass, no `<inertia>`; plus a link with no collision to fall back on | `control_link`, fully specified |
| `c_revolute_no_limit.urdf` | G7 — revolute joint with no `<limit>`, and one with a partial `<limit>` | `continuous_joint` (legitimately unbounded), `limited_joint` |

All three parse as valid XML/URDF. None is derived from anything in
`references/`.

---

## 4. What could not run, and exactly why

### 4.1 `urdf-usd-converter` 0.3.2 / 0.3.3 — blocked on macOS

`usd-exchange` publishes wheels for `manylinux_2_35_x86_64`,
`manylinux_2_35_aarch64` and `win_amd64` only. There is **no macOS wheel for any
version** (192 wheels on PyPI, zero `macosx`). `urdf-usd-converter` requires
`usd-exchange>=2.2.2`, so it cannot be installed here:

```console
$ pip install 'urdf-usd-converter==0.3.2'
Collecting urdf-usd-converter==0.3.2
  Downloading urdf_usd_converter-0.3.2-py3-none-any.whl.metadata (9.4 kB)
...
ERROR: Could not find a version that satisfies the requirement usd-exchange>=2.2.2
       (from urdf-usd-converter) (from versions: none)
ERROR: No matching distribution found for usd-exchange>=2.2.2
```

`scripts/run_converter_matrix.py` was run anyway and **recorded the failure**
rather than skipping silently. `tests/_artifacts/matrix.json` now holds the
host, both attempted versions, and uv's resolver explanation, which surfaces
verbatim in the pytest skip reason:

```
SKIPPED [20] converter 0.3.2 unavailable: cause: Because usd-exchange>=2.2.2 has
no wheels with a matching platform tag (e.g., `macosx_13_0_x86_64`) ...
```

**40 converter tests are written and skipping.** They assert exact values —
`newton:damping == 1.5·π/180`, prismatic damping unchanged at 4.0, velocity
limit converted for angular DOFs only, `urdf:limit:effort` 87.0/50.0, zero
drives, zero armature, zero physics materials, zero filtered pairs; a zero
`principalAxes` quaternion on 0.3.2 and an unauthored one on 0.3.3; two joints
locked at `[0,0]` with `continuous_joint` unaffected; limits in degrees. On
Linux this is one command:

```bash
python scripts/run_converter_matrix.py && pytest tests/converter -q
```

The tests **fail** (not skip) if a run is recorded in `matrix.json` but did not
succeed, so a broken conversion cannot pass quietly.

### 4.2 Isaac Sim URDF importer as a standalone package — blocked, and not only by macOS

Attempted as instructed, in place, with no writes to `references/`
(`git status` on all three reference repos: **0 modified, before and after**).

Five independent blockers. **Four are not platform-specific.**

| # | Blocker | Evidence | macOS-only? |
|---|---|---|---|
| 1 | `usd-exchange==2.3.0` has no macOS wheel | `uv pip install 'usd-exchange==2.3.0'` on Python 3.12 → *"no wheels with a matching platform tag (e.g. `macosx_13_0_x86_64`)"* | yes |
| 2 | `isaacsim-asset-transformer-rules` and `isaacsim-asset-transformer` are unpublished | `pypi.org/simple/...` → **HTTP 404** for both; only `isaacsim` and `isaacsim-robot-schema` (6.1.0.0, `py3-none-any`) exist | no |
| 3 | The importer pins `isaacsim-asset-transformer-rules==1.0.0`, but that package's own `pyproject.toml` in this very checkout declares `version = "1.7.18"` | both files read directly; `isaacsim.asset.importer.mjcf` carries the same pin | no |
| 4 | A plain `pip install .` of either extension produces an **empty wheel** | `setuptools.find_packages(where=<ext>, include=["isaacsim.asset.importer.urdf", ...])` → `[]`; there is no `isaacsim/` directory, only `python/`. The import path is created by `[tool.standalone-wheel] symlinks`, materialised by `./repo.sh build_standalone_wheels` | no |
| 5 | `isaacsim.asset.transformer` (the rule engine) and `isaacsim.asset.importer.utils` (`asset_utils`, `importer_utils`, `urdf_to_mjc_physx_conversion_utils`) have **no `pyproject.toml` or `setup.py` anywhere** in the repo, yet `converter.py` imports both at module scope. They are built by `module-carrier.toml` / premake under Kit | no |

Only five extensions in the whole repository ship a standalone `pyproject.toml`:
`isaacsim.asset.exporter.urdf`, `isaacsim.asset.importer.mjcf`,
`isaacsim.asset.importer.urdf`, `isaacsim.asset.transformer.rules`,
`isaacsim.robot_setup.sysid`.

**Conclusion:** standalone wheels are not obtainable without running NVIDIA's own
`./repo.sh build_standalone_wheels` inside `references/IsaacSim`, which writes
into that tree and is forbidden by `CLAUDE.md`. The practical path is an
**installed Isaac Sim and its `python.sh`**, which is what
`scripts/verify_isaac_regression.py` and `docs/VERIFY.md` now target.

One useful discovery while probing: the importer does **not** need Kit.
`isaacsim.asset.importer.utils.stage_utils` uses plain `pxr.Usd`, and the
extension ships `standalone_tests/` that exercise import without a running app.
So the verification script needs `python.sh`, not a `SimulationApp` — no
rendering, and in principle no GPU for the import path itself.

> **Wrong — corrected in Phase 2.5.** On a real install `python.sh` alone gives
> `ModuleNotFoundError: No module named 'isaacsim.asset'`, because `isaacsim` is
> a regular package whose `__path__` only Kit's extension manager extends. The
> script now boots a headless `SimulationApp` first. See §5.1 and
> `docs/VERIFY.md` §T2.

### 4.3 No real converted asset exists locally

All 145 `.usd*` files under `references/` are Git-LFS pointers except six schema
definition files. There is no real robot asset to inspect here, so the sample
`inspect` output below was produced from a **hand-authored** stage and is
labelled accordingly.

---

## 5. Confirmed vs. still unverified

> **Updated 2026-09-17 (Phase 2.5), on `isr-lab`** — Ubuntu 22.04.5 x86-64,
> RTX 4090, driver 580.178.04, Isaac Sim 6.1.0-rc.26. T0, T1 and T2 all ran.
> `docs/VERIFY.md` holds the commands, the full version matrix and the raw
> readings; this table is the verdict per claim.

**Headline change: 91 tests pass, 0 skipped.** On macOS it was 51 passed and 40
skipped. The 40 converter assertions now execute and pass, and the Isaac Sim
regression returned `REGRESSION_CONFIRMED`. Nothing observed contradicted
`docs/history/ANALYSIS.md`; one claim in `docs/VERIFY.md` was wrong and is corrected
below.

| Claim (from `docs/history/ANALYSIS.md`) | Status after Phase 2 | Status after Phase 2.5 | How |
|---|---|---|---|
| Isaac Sim 6.1.0 pins `urdf-usd-converter==0.3.2` | Confirmed, automated | **Confirmed on the binary install too** | `pip_prebundle/urdf_usd_converter-0.3.2.dist-info` in `~/isaacsim` |
| Converter ≥ 0.3.0 writes `newton:damping`, not `urdf:dynamics:damping` | Confirmed at source | **Confirmed behaviourally** | `newton:damping = 0.026179939508`, `urdf:dynamics:*` absent, both versions |
| Converter 0.1.3 wrote `urdf:dynamics:damping` | Confirmed, automated | unchanged (source only) | 0.1.3 is not in the matrix; it predates the fixtures' schema set |
| Isaac Sim 6.1.0 reads `urdf:dynamics:damping` | Confirmed at source | **Confirmed at runtime** | Isaac's own warning names the consequence |
| The two sets do not intersect → G1 | Confirmed at source level | **Confirmed at runtime** | see the G1 row below |
| **G1 observable in a running Isaac Sim 6.1.0** | *Unverified — needs T2* | **CONFIRMED** | `REGRESSION_CONFIRMED`; stock import leaves `drive:angular:physics:damping` unauthored while `newton:damping` carries 1.5 rad⁻¹ |
| **G1 absent in Isaac Sim 6.0.1** | *Unverified — needs T2* | **Still unverified** | 6.0.1 is not installed on this box; install steps in `docs/VERIFY.md` §T2 |
| Isaac applies `DriveAPI` with no gains | Confirmed at source | **Confirmed at runtime** | `drive_api_applied = True`, damping and stiffness unauthored, `maxForce = 87.0` |
| Converter authors no `DriveAPI` at all | Confirmed, automated (grep) | **Confirmed behaviourally** | `joints_with_drive_api = 0` on all six conversions |
| **Converter 0.3.2 authors a zero `principalAxes`** | Source level; behaviour unverified | **CONFIRMED** | both no-inertia links: `principalAxes = (0,0,0,0)`, `authored: true` |
| **0.3.3 fixed it** | Source level; behaviour unverified | **CONFIRMED** | same links on 0.3.3: `authored: false` |
| **Revolute joint without `<limit>` → `[0,0]`** | *Not yet verified* | **CONFIRMED** | `no_limit_joint` and `partial_limit_joint` both `[0,0]`; `continuous_joint` stays ±inf |
| Mesh colliders are always `convexHull` | Confirmed, automated (source) | **Still source-only** | all three fixtures use `<box>`; `collider_approximations = {"(unauthored)": 3}`. Needs a mesh fixture |
| **No physics materials / filtered pairs authored** | Phase 1 grep; assertions ready | **CONFIRMED** | `physics_materials_total = 0`, `collision_groups_total = 0`, `colliders_with_filtered_pairs = 0`, no armature in any namespace |
| Converter authors no `Physics` variant set | implied | **Confirmed** | `variant_sets = []` from the converter; the set is Isaac's post-processing |
| Isaac 6.1.0 splits physics into three layers behind a `Physics` variant with no default selection | Confirmed at source (§2) | **Confirmed from real output** | `payloads/Physics/{physics,physx,mujoco}.usda`; variants `mujoco, none, physics, physx`, `has_authored_selection: false` |
| `usd-exchange` has no macOS wheels | Confirmed | unchanged | PyPI index + two install attempts |
| Isaac importer installable standalone | Disproved (5 blockers) | **Disproved again, differently** | see §5.1 |

### 5.1 One Phase 2 claim was wrong: "the importer does not need Kit"

§4.2 of this report and `docs/VERIFY.md` both asserted that `python.sh` alone
would do, because `isaacsim.asset.importer.utils.stage_utils` uses plain
`pxr.Usd`. On a real install that is false:

```console
$ ~/isaacsim/python.sh scripts/verify_isaac_regression.py
ModuleNotFoundError: No module named 'isaacsim.asset'
```

`isaacsim` is a *regular* package with a fixed `__path__`; each extension's real
modules live in its own `exts/<ext>/pip_prebundle/isaacsim/...`; and
`isaacsim.asset.transformer` (one extension, a regular package) and
`isaacsim.asset.transformer.rules` (another extension) cannot be merged by
patching `__path__` by hand. Kit's extension manager is what joins them.
`scripts/verify_isaac_regression.py` now boots a headless `SimulationApp` and
enables `isaacsim.asset.importer.urdf` before importing it; `--no-simulation-app`
keeps the old path available. `docs/VERIFY.md` §T2 carries the full trace.

A related correction to §4.2 blocker 5: those two modules *do* exist as wheels —
in the binary install's `pip_prebundle` directories. They are simply absent from
the source repo. The blocker stands; the reason was stated too strongly.

### 5.2 Version skew worth naming

The matrix resolved `usd-exchange` **3.0.0** and `newton-usd-schemas` **0.5.0**,
while Isaac Sim 6.1.0 ships 2.3.0 and 0.4.1. Converter behaviour matched the
prediction regardless, but per `CLAUDE.md`'s pinning rule the matrix should gain
an Isaac-pinned column before anyone reasons from these runs about Isaac's exact
stack. `matrix.json` now records the resolved versions (`env.frozen`), which it
did not before — `pip freeze` silently returned nothing in `uv`-created venvs,
because they have no `pip`.

### 5.3 The honest summary, updated

Phase 2 ended with: *the regression is proven at source level and automated; it
is not yet proven at runtime.* Phase 2.5 closes that for 6.1.0. The stock import
drops `<dynamics damping="1.5">`, the override control proves the drive is
writable and readable, and Isaac Sim emits its own warning saying the actuator
is created without gain parameters. What remains is the 6.0.1 half of the
before/after pair, which needs a second install, and a mesh-collider fixture.

## 6. Sample `inspect` output

> **Synthetic.** Hand-authored to the shape of an Isaac Sim 6.1.0 import of
> fixture (a), because no converter can run on this host. It demonstrates the
> tool's rendering, not upstream behaviour.
>
> **Phase 2.5:** the real thing has since been produced on `isr-lab` and the
> shape held, including `newton:damping` authored with no drive damping. One
> detail was wrong here and is fixed below: the variant is spelled `none`,
> lowercase, and the variant order is `mujoco, none, physics, physx`.

```
Variant sets
============
  set      variants                      selection
  -------  ----------------------------  ---------------
  Physics  mujoco, none, physics, physx  (none authored)
  note: no 'Physics' selection is authored, so no physics layer composes
        until a consumer selects one (Isaac Sim 6.1.0 behaviour).

Joints (2, 2 actuatable)
========================
  joint           type           limits   drive                 damping@  friction@  armature@  notes
  --------------  -------------  -------  --------------------  --------  ---------  ---------  ------------------------------------
  shoulder_joint  RevoluteJoint  -90..90  angular:k=-,d=-,F=87  newton    newton     -          no-gains damping-stranded no-armature

MuJoCo actuators (1)
====================
  actuator                 target                                     gainType  gainPrm  biasPrm  notes
  -----------------------  -----------------------------------------  --------  -------  -------  --------
  shoulder_joint_actuator  /a_dynamics_damping/Physics/shoulder_joint  -         -        -        NO-GAINS
```

`damping@ newton` with an empty drive column, next to `NO-GAINS` on the
actuator, is the whole G1 finding in one line.

---

## 7. Against the Phase 2 brief

| # | Item | Status |
|---|---|---|
| 1 | Package skeleton: uv `pyproject.toml`, src layout, `urdf-usd-bridge` entry point, Apache-2.0 `LICENSE`, `README` with acknowledgements, `THIRD_PARTY.md`, `tests/` | **Done** |
| 2 | Pure-`pxr` `inspect` covering drives, damping/friction across all namespaces, armature, mass/inertia validity, `principalAxes`, limits, collision filtering, physics materials, Physics variant; text + `--json` | **Done**, 41 tests |
| 3 | Fixtures (a)(b)(c) written by us; tests asserting what 0.3.2 and 0.3.3 author | **Written; cannot execute on macOS.** 40 assertions ready; runner builds one venv per version | 
| 4 | `scripts/verify_isaac_regression.py` + `docs/VERIFY.md`, not run here | **Done**, not run |
| 5 | Install the Isaac importer standalone from `references/` | **Attempted, blocked.** Five documented blockers, four platform-independent. `references/` untouched |
| 6 | Run everything that works on macOS; report | **Done** — this file |

`CLAUDE.md` compliance: `references/` shows **0 modified files** in all three
repos; nothing was copied into `src/`; `THIRD_PARTY.md` records the only
borrowed artifact (the canonical Apache-2.0 licence text) and pre-registers the
three adaptations planned for Phase 3; `README.md` carries the acknowledgements
and an explicit non-endorsement statement; the tested-version matrix is in
`README.md` and `docs/VERIFY.md`.

---

## 8. Next actions

> **Phase 2.5 status:** the first two blocks below are **done** — see §5. What
> is left from this section is the 6.0.1 comparison and the Phase 3 questions.

**On a Linux x86-64 box (no GPU needed):** ✅ done, 40/40 passing.

```bash
uv pip install -e '.[core]' --group dev
python scripts/run_converter_matrix.py
pytest tests/converter -q
```

Expect 40 assertions to go from skipped to passing or failing. Either outcome is
informative; a failure means `docs/history/ANALYSIS.md` needs correcting. *(All 40
passed; `docs/history/ANALYSIS.md` needed no correction.)*

**On a Linux + NVIDIA GPU box with Isaac Sim 6.1.0 and 6.0.1:**

```bash
"$ISAAC_SIM_DIR_61/python.sh" scripts/verify_isaac_regression.py --json isaac_6.1.0.json
"$ISAAC_SIM_DIR_60/python.sh" scripts/verify_isaac_regression.py --json isaac_6.0.1.json
```

Expect `REGRESSION_CONFIRMED` on 6.1.0 and `REGRESSION_ABSENT` on 6.0.1. With
both JSON files, the upstream issue writes itself; `docs/VERIFY.md` lists the
four facts to attach.

*(Phase 2.5: 6.1.0 ran and returned `REGRESSION_CONFIRMED`, plus Isaac Sim's own
"actuator will be created without gain parameters" warning. 6.0.1 is not
installed on that box; `docs/VERIFY.md` §T2 has the exact install command.)*

**Open questions for Phase 3**, unchanged from `docs/history/ANALYSIS.md` §8 except the
first, which Phase 2 settled:

1. ~~`usd-exchange` or `usd-core`?~~ **Settled:** neither is a hard dependency;
   code against `pxr`, offer both as extras.
2. Whether Isaac Sim 5.1.0 can load a 6.x package at all — still needs a 5.1.0 runtime.
3. ~~How `geometries.usd` / `instances.usda` sublayer into `base.usd`~~ —
   **settled in Phase 2.5** by running the importer: the asset is
   `<robot>.usda` + `payloads/{base.usda, robot.usda}` +
   `payloads/Physics/{physics,physx,mujoco}.usda`, behind a `Physics` variant
   set with no authored selection.
4. First repair scope. Proposal stands: G1 (drives), G2 (inertia), G7 (limits),
   then hold-pose and drop suites on MuJoCo and Newton.
