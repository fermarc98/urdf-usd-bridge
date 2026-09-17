# Verification runbook

What can be verified where, and how to run the parts that need hardware the
development machine does not have.

## Platform tiers

| Tier | Requires | Runs |
|---|---|---|
| **T0 — any platform** | Python 3.10–3.12, `usd-core` | unit tests, `inspect`, static source evidence |
| **T1 — Linux / Windows x86-64** | + `usd-exchange` (no macOS wheels) | `convert`, the converter matrix |
| **T2 — Linux + NVIDIA GPU** | + an Isaac Sim 6.x install | `scripts/verify_isaac_regression.py` |
| **T3 — Linux + NVIDIA GPU** | + `newton`, `mujoco` | simulation suites (Phase 3+) |

### Hosts used so far

| Host | Tiers reached | When |
|---|---|---|
| macOS 13.7.8 x86-64 (Phase 2 dev box) | T0 | 2026-09-17 |
| Ubuntu 22.04.5 x86-64, RTX 4090, Isaac Sim 6.1.0 (`isr-lab`) | **T0, T1, T2** | 2026-09-17 (Phase 2.5) |

As of Phase 2.5 nothing below T3 is unexecuted: T0 and T1 pass on Linux and T2
returned a verdict. `docs/PHASE2_REPORT.md` §5 records claim-by-claim status.

### Why macOS stops at T0

`usd-exchange` publishes wheels for `manylinux_2_35_x86_64`,
`manylinux_2_35_aarch64` and `win_amd64` only — there is no macOS wheel for any
version. `urdf-usd-converter` depends on `usd-exchange>=2.2.2`, so it cannot be
installed on macOS either. Reproduce:

```console
$ pip install 'urdf-usd-converter==0.3.2'
ERROR: Could not find a version that satisfies the requirement usd-exchange>=2.2.2
       (from urdf-usd-converter) (from versions: none)
```

This is a packaging fact, not a bug in this project, and it is why `inspect` is
built against `pxr` alone.

---

## T0 — everything, on any platform

```bash
uv venv --python 3.10 .venv
uv pip install --python .venv/bin/python -e '.[core,schemas]' --group dev
.venv/bin/python -m pytest tests/unit -q
```

`tests/unit/test_reference_sources.py` additionally needs the `references/`
checkouts with tags `v0.3.2`, `v0.3.3`, `v0.1.3` (converter) and `v6.1.0`,
`v6.0.1` (Isaac Sim). It reads them with `git show` and skips if absent.
`references/` is not tracked by git, so those ten tests skip on a fresh clone
and in CI; `tests/conftest.py::have_git_references` is the gate. A shallow
checkout is enough to add a missing tag:

```bash
git -C references/IsaacSim fetch --depth 1 origin tag v6.0.1
```

### Two host traps, both found on Linux

1. **`uv` is required**, by this runbook and by `scripts/run_converter_matrix.py`
   (which falls back to `venv` + `pip` but then cannot record the resolved
   versions as cleanly). Install it however you like; nothing else needs it.
2. **A sourced ROS 2 environment breaks pytest.** ROS puts
   `/opt/ros/<distro>/lib/python3.10/site-packages` on `PYTHONPATH`, the venv
   inherits it, and pytest autoloads ROS's `launch_testing` plugin, which dies
   with `ModuleNotFoundError: No module named 'yaml'` before collection. It is
   not a failure of this project. Clear the variable for the run:

   ```bash
   PYTHONPATH= .venv/bin/python -m pytest tests/unit -q
   ```

**Result on `isr-lab`, 2026-09-17:** 51 passed, 0 failed, 0 skipped — the ten
reference-source tests run here because `references/` is present. On a clean
checkout without `references/`: 41 passed, 10 skipped, which is what CI sees.

---

## T1 — the converter matrix

Proves what each converter version actually authors for each fixture, which is
the behavioural counterpart to the static source evidence in T0.

```bash
python scripts/run_converter_matrix.py            # builds one venv per version
pytest tests/converter -q
```

Artifacts land in `tests/_artifacts/`:

```
matrix.json                     environment + per-run status
0.3.2/a_dynamics_damping.json   the inspect report
0.3.2/a_dynamics_damping/       the converted USD asset
```

`tests/converter/` skips when `matrix.json` is missing and **fails** when a run
recorded in it did not succeed, so a broken conversion cannot pass silently.

### Result on `isr-lab`, 2026-09-17 — all three predictions held

6 conversions, 6 successes, **40 of 40 assertions passing**, nothing skipped.
Every row below was predicted by `docs/ANALYSIS.md` from source alone and is now
an observed value; nothing contradicted the analysis.

| Fixture | 0.3.2 | 0.3.3 | Observed |
|---|---|---|---|
| (a) | `newton:damping = 1.5·π/180`, no `urdf:dynamics:*`, no `DriveAPI` | identical | `newton:damping = 0.026179939508`, `newton:friction = 0.3`, `urdf:dynamics:*` absent, `joints_with_drive_api = 0` on both |
| (b) | `physics:principalAxes` authored as `(0,0,0,0)` | left unauthored | on 0.3.2 both no-inertia links carry `principalAxes = (0,0,0,0)` with `authored: true`; on 0.3.3 the same attribute is `authored: false` |
| (c) | `lowerLimit == upperLimit == 0` on two joints | identical | `no_limit_joint` and `partial_limit_joint` lock to `[0,0]` on both versions; `continuous_joint` stays `±inf` |

Also observed on every fixture and both versions, confirming `ANALYSIS.md` §1.4
behaviourally rather than by grep: zero `DriveAPI`, zero armature in any
namespace, zero physics materials, zero collision groups, zero filtered pairs,
and no `Physics` variant set at all — the variant set is Isaac Sim's doing, not
the converter's.

**One §1.4 claim is still source-only:** "mesh colliders are always
`convexHull`". All three fixtures use `<box>` collision geometry, so no
`MeshCollisionAPI` is authored and `collider_approximations` reads
`{"(unauthored)": 3}`. A mesh fixture is needed to close this behaviourally.

**Versions actually resolved** (recorded in `matrix.json`, `env.frozen`):

| | 0.3.2 run | 0.3.3 run |
|---|---|---|
| `urdf-usd-converter` | 0.3.2 | 0.3.3 |
| `usd-exchange` | 3.0.0 | 3.0.0 |
| `newton-usd-schemas` | 0.5.0 | 0.5.0 |
| `numpy` | 2.2.6 | 2.2.6 |

Note the skew against what Isaac Sim 6.1.0 ships (`usd-exchange` 2.3.0,
`newton-usd-schemas` 0.4.1). The converter behaviour matched the prediction
anyway, but per `CLAUDE.md` the matrix should grow a pinned-to-Isaac column
before any claim is made about Isaac's exact stack from these runs.

---

## T2 — the Isaac Sim regression check

This is the one that turns `docs/ANALYSIS.md` G1 from a source-level claim into
a reproducible bug report.

### Requirements

- Linux x86-64 with an NVIDIA GPU and a current driver.
- Isaac Sim **6.1.0** installed, and ideally **6.0.1** as well for the
  before/after comparison.
- This repository checked out on the same machine. Nothing needs to be
  pip-installed into Isaac Sim's interpreter — the script puts `src/` on
  `sys.path` itself.
- Two installs in **separate directories** if you want the comparison; the
  standalone package always unzips to a fixed layout, so a second version
  installed over the first replaces it.

### Run

```bash
# Isaac Sim 6.1.0
"$ISAAC_SIM_DIR_61/python.sh" scripts/verify_isaac_regression.py \
    --out /tmp/isaac61 --json isaac_6.1.0.json

# Isaac Sim 6.0.1, for the comparison
"$ISAAC_SIM_DIR_60/python.sh" scripts/verify_isaac_regression.py \
    --out /tmp/isaac60 --json isaac_6.0.1.json
```

### Correction: the importer *does* need Kit

Phase 2 claimed, from reading `references/IsaacSim`, that `python.sh` alone was
enough because `isaacsim.asset.importer.utils.stage_utils` uses plain `pxr.Usd`.
**That is wrong on a real install**, and Phase 2.5 corrected the script. What
actually happens on Isaac Sim 6.1.0:

```console
$ ~/isaacsim/python.sh scripts/verify_isaac_regression.py
ModuleNotFoundError: No module named 'isaacsim.asset'
```

The mechanism, traced on the installed build:

1. `isaacsim` is a **regular** package — `python_packages/isaacsim/__init__.py` —
   so its `__path__` is one fixed directory and no amount of `sys.path` or
   `PYTHONPATH` can graft `isaacsim.asset` onto it.
2. Each extension ships its own tree at `exts/<ext>/isaacsim/...`, and the real
   implementation modules live one level deeper again, in
   `exts/<ext>/pip_prebundle/isaacsim/...`. In the binary build
   `exts/isaacsim.asset.importer.utils/isaacsim/asset/importer/utils/` contains
   only `tests/`; `asset_utils.py` and
   `urdf_to_mjc_physx_conversion_utils.py` are in the prebundle.
3. `isaacsim.asset.transformer` is itself a *regular* package in one extension's
   prebundle, while `isaacsim.asset.transformer.rules` lives in a different
   extension — so even hand-patching `isaacsim.__path__` gets as far as
   `ModuleNotFoundError: No module named 'isaacsim.asset.transformer.rules'`.

Stitching those trees together is precisely what Kit's extension manager does.
The script therefore boots a headless `SimulationApp` and calls
`set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)` before
importing anything from `isaacsim.asset`. No rendering is performed and no
window opens, but a GPU is still required. `--no-simulation-app` opts out, and
is kept only so the standalone path can be re-tested if upstream changes.

Startup cost: ~157 s on the first run (Warp kernel and shader caches cold),
~10 s afterwards.

### What it does

1. Imports `tests/fixtures/a_dynamics_damping.urdf` with a **stock**
   `URDFImporterConfig`, i.e. what a user gets by default.
2. Inspects the output with `--variant Physics=physx`, because 6.1.0 authors no
   default selection for that variant set.
3. Imports it **again** with `override_joint_stiffness=800`,
   `override_joint_damping=40` as a control. If the override run also shows no
   drive damping, the inspection path is wrong rather than the importer, and the
   script reports `INCONCLUSIVE` instead of claiming a bug.

### Reading the verdict

| Verdict | Meaning |
|---|---|
| `REGRESSION_CONFIRMED` | `<dynamics damping="1.5">` did not reach `drive:angular:physics:damping`, `newton:damping` did carry it, and the override control worked. |
| `REGRESSION_ABSENT` | The drive was populated; `docs/ANALYSIS.md` G1 is wrong for this build and must be corrected. |
| `INCONCLUSIVE` | The control failed too; fix the harness before claiming anything. |

Expected: `REGRESSION_CONFIRMED` on 6.1.0 and `REGRESSION_ABSENT` on 6.0.1. A
`REGRESSION_ABSENT` on 6.1.0 is a real result — record it and correct the
analysis.

### Result on `isr-lab`, 2026-09-17: `REGRESSION_CONFIRMED`

Isaac Sim `6.1.0-rc.26` (`release.49347.2d230af4.gl`), which prebundles
`urdf_usd_converter-0.3.2.dist-info` — the pin, confirmed on the binary install
rather than only in the source tree.

Observed on the stock import, `Physics=physx`:

| Reading | Value |
|---|---|
| `drive_api_applied` | `True` |
| `drive:angular:physics:damping` | **not authored** |
| `drive:angular:physics:stiffness` | **not authored** |
| `drive:angular:physics:maxForce` | `87.0` |
| `newton:damping` | `0.026179939508` (= 1.5 rad⁻¹ in degrees) |
| `urdf:dynamics:damping` | absent |
| `physxJoint:jointFriction` | not authored |
| `newton:friction` | `0.3` |
| `urdf:limit:effort` | `87.0` |
| armature, any namespace | none |

The control run with `override_joint_stiffness=800`,
`override_joint_damping=40` **did** populate the drive
(`drive_damping = 0.698131680` = 40·π/180), so drives are readable and the
default path is genuinely dropping the URDF value. Verdict is not
`INCONCLUSIVE`.

`drive:angular:physics:maxForce = 87.0` next to an empty damping and stiffness
is the whole finding in one line: `urdf:limit:effort` is the one attribute whose
spelling still connects, exactly as the source analysis predicted.

**Isaac Sim says it itself.** The import logs, unprompted:

```
[Warning] [isaacsim.asset.importer.utils.impl.urdf_to_mjc_physx_conversion_utils]
Stiffness and damping not available joint /a_dynamics_damping/Physics/shoulder_joint,
actuator will be created without gain parameters
```

That warning fires for both actuatable joints. It is upstream's own diagnosis of
the bug, emitted at default settings, and it is the single best thing to put at
the top of the issue.

Two structural claims from `docs/ANALYSIS.md` §2 were confirmed in passing, from
the real output rather than from reading:

- The physics split is into **three** layers — `payloads/Physics/physics.usda`,
  `physx.usda`, `mujoco.usda` — alongside `payloads/base.usda` and
  `payloads/robot.usda`.
- The `Physics` variant set offers `mujoco, none, physics, physx` and carries
  **no authored selection**, so no physics layer composes until a consumer picks
  one. (The Phase 2 synthetic sample spelled that variant `None`; it is
  lowercase `none`.)

### Not yet run: the 6.0.1 comparison

Only 6.1.0 is installed on `isr-lab` (`~/isaacsim`, 32 GB). The `REGRESSION_ABSENT`
half of the before/after pair is still outstanding, so what is proven is that
6.1.0 drops the value — not yet, at runtime, that 6.0.1 did not. The source-level
evidence for 6.0.1 (it pinned converter 0.1.3, whose spelling intersects) stands
and is automated in `tests/unit/test_reference_sources.py`.

To close it, install 6.0.1 **into a separate directory** and run the same script:

```bash
# ~46 GB free needed: ~14 GB zip + ~32 GB installed
python3 ~/isaacsim/skills/isaac-sim-installation/scripts/install_binary.py \
    --release-source fallback --platform linux-x86_64 \
    --download --verify-md5 \
    --install-dir ~/isaacsim-6.0.1 --execute

PYTHONPATH= ~/isaacsim-6.0.1/python.sh scripts/verify_isaac_regression.py \
    --out /tmp/isaac60 --json isaac_6.0.1.json
```

The pinned fallback metadata in that installer's
`references/binary.md` is Linux x86-64 `isaac-sim-standalone-6.0.1-linux-x86_64.zip`,
MD5 `65e2c2e83e2461ce0f33b0732d0ee4a3`. Drop `--execute` first for a dry run.
Do not install over `~/isaacsim`; that is the working 6.1.0.

### If it is confirmed

Attach `isaac_6.1.0.json`, `isaac_6.0.1.json`, the fixture, and these facts to
an upstream issue:

- `isaacsim.asset.importer.urdf/pyproject.toml` pins `urdf-usd-converter==0.3.2`.
- `urdf-usd-converter` commit `99ff034` ("Updated to newton schemas v0.4.0",
  first released in 0.3.0) removed the `urdf:dynamics:damping` and
  `urdf:dynamics:friction` attributes.
- `urdf_to_mjc_physx_conversion_utils.convert_urdf_to_physx()` still reads both.
- The rule's unit test hand-authors those attributes on a synthetic stage
  (`isaacsim/asset/transformer/rules/tests/test_urdf_to_mjc_physx_conversion.py`),
  so it passes without exercising the real converter.

---

## Standalone Isaac Sim packaging (attempted, blocked)

Installing `isaacsim.asset.importer.urdf` from `references/IsaacSim` as an
ordinary Python package does not work, for five independent reasons. Four of
them are not platform-specific.

| # | Blocker | Platform-specific? |
|---|---|---|
| 1 | `usd-exchange==2.3.0` has no macOS wheel | yes |
| 2 | `isaacsim-asset-transformer-rules` and `isaacsim-asset-transformer` are not published on PyPI (HTTP 404); only `isaacsim` and `isaacsim-robot-schema` are | no |
| 3 | The importer pins `isaacsim-asset-transformer-rules==1.0.0`, but that package's own `pyproject.toml` in this checkout declares `version = "1.7.18"` — unsatisfiable as written, even from source | no |
| 4 | `setuptools.find_packages` returns `[]` for both extension directories: the `isaacsim/asset/...` import path exists only after the `[tool.standalone-wheel] symlinks` step, applied by `./repo.sh build_standalone_wheels`, so a plain `pip install .` produces an empty wheel | no |
| 5 | `isaacsim.asset.transformer` and `isaacsim.asset.importer.utils` — both imported at module level by `converter.py` — have no `pyproject.toml` or `setup.py` anywhere in the repo; they are built by `module-carrier.toml` / premake under Kit | no |

Phase 2.5 footnote: the *binary* install does ship those two as ordinary wheels,
under `exts/<ext>/pip_prebundle/`. That does not unblock a standalone install —
see the Kit correction under T2 for why the trees cannot be merged by hand — but
it does mean the modules exist in wheel form, contrary to what reading the
source repo alone suggests.

Only five extensions in the whole repo ship a standalone `pyproject.toml`:
`isaacsim.asset.exporter.urdf`, `isaacsim.asset.importer.mjcf`,
`isaacsim.asset.importer.urdf`, `isaacsim.asset.transformer.rules`,
`isaacsim.robot_setup.sysid`.

**Therefore: use an installed Isaac Sim and its `python.sh`.** Building the
standalone wheels from source would require running NVIDIA's own
`./repo.sh build_standalone_wheels` inside `references/IsaacSim`, which writes
into that tree and is forbidden by `CLAUDE.md`. If standalone wheels are ever
wanted, copy the checkout elsewhere first.

---

## Version matrix to record with any result

Paste this filled in alongside any verification run.

| Component | Version | Source |
|---|---|---|
| OS / arch | | |
| Python | | |
| `usd-core` or `usd-exchange` | | |
| `urdf-usd-converter` | | |
| `newton-usd-schemas` | | |
| Isaac Sim | | |
| `newton` | | |
| `mujoco` / `mujoco-warp` | | |
| NVIDIA driver | | `nvidia-smi` |

### Filled in for the Phase 2.5 run (`isr-lab`, 2026-09-17)

| Component | Version | Source |
|---|---|---|
| OS / arch | Ubuntu 22.04.5 LTS, Linux 6.8.0-138-generic, x86-64 | `uname -a`, `/etc/os-release` |
| Python (T0/T1 driver) | 3.10.12 system; 3.11.15 also present; 3.12 fetched by `uv` for the CI rehearsal | `python3 -V` |
| Python (T2) | 3.12, Isaac Sim's own interpreter | `python.sh` |
| `uv` | 0.12.15 | not installed system-wide; bootstrapped for this run |
| `usd-core` (T0) | 26.8 | `uv pip install -e '.[core,schemas]'` |
| `usd-exchange` (T1) | 3.0.0 | resolved by `urdf-usd-converter` |
| `urdf-usd-converter` | 0.3.2 and 0.3.3, one venv each | `scripts/run_converter_matrix.py` |
| `newton-usd-schemas` | 0.5.0 | resolved from `>=0.4.0` |
| `numpy` | 2.2.6 | resolved |
| Isaac Sim | 6.1.0-rc.26, build `release.49347.2d230af4.gl`, at `~/isaacsim` | `cat ~/isaacsim/VERSION` |
| Isaac Sim's bundled converter | `urdf_usd_converter` 0.3.2 | `exts/isaacsim.asset.importer.urdf/pip_prebundle/` |
| `newton` / `mujoco` | not exercised (T3) | — |
| NVIDIA driver | 580.178.04, CUDA 13.0, GeForce RTX 4090 (24 GB) | `nvidia-smi` |
| Warp (inside Isaac Sim) | 1.16.0, CUDA Toolkit 12.9 | Isaac Sim startup log |

Reference checkouts used, all clean, none modified:

| Repo | Tags available | Commit of the tag under test |
|---|---|---|
| `references/urdf-usd-converter` | `v0.1.0`…`v0.3.3`, incl. `v0.1.3`, `v0.3.2`, `v0.3.3` | HEAD `b636469` = `v0.3.3` |
| `references/mujoco-usd-converter` | `v0.1.0`…`v0.5.0` | HEAD `657158f` = `v0.5.0` |
| `references/IsaacSim` | `v6.1.0`; `v6.0.1` fetched `--depth 1` in Phase 2.5 | `v6.1.0` = `7c206f7`, `v6.0.1` = `045ca8b` |

`v5.1.0` is **not** present in this checkout. No test needs it; `CLAUDE.md` lists
it as a pinned reference state, so fetch it the same way if Phase 3 needs the
5.1.0 comparison:

```bash
git -C references/IsaacSim fetch --depth 1 origin tag v5.1.0
```
