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

The development machine for this project is macOS x86-64, i.e. **T0 only**.
Everything below T0 is written, committed and unexecuted until it runs on a
suitable host; `docs/PHASE2_REPORT.md` records exactly which is which.

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

Expected outcome, if `docs/ANALYSIS.md` is right:

| Fixture | 0.3.2 | 0.3.3 |
|---|---|---|
| (a) | `newton:damping = 1.5·π/180`, no `urdf:dynamics:*`, no `DriveAPI` | identical |
| (b) | `physics:principalAxes` authored as `(0,0,0,0)` | left unauthored |
| (c) | `lowerLimit == upperLimit == 0` on two joints | identical |

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

### Run

```bash
# Isaac Sim 6.1.0
"$ISAAC_SIM_DIR_61/python.sh" scripts/verify_isaac_regression.py \
    --out /tmp/isaac61 --json isaac_6.1.0.json

# Isaac Sim 6.0.1, for the comparison
"$ISAAC_SIM_DIR_60/python.sh" scripts/verify_isaac_regression.py \
    --out /tmp/isaac60 --json isaac_6.0.1.json
```

The importer itself does not need Kit — `isaacsim.asset.importer.utils.stage_utils`
uses plain `pxr.Usd` — so `python.sh` is enough and no `SimulationApp` is started.

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
