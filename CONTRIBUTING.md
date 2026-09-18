# Contributing

Thanks for looking. This project has an unusual rule at its centre, so please
read the first section before opening a PR that adds a repair.

## The one rule: a repair must earn its place

Every repair rule has to be justified by a **measurement**, not by an argument.
`docs/PHASE4_REPORT.md` reports one of our own rules — `armature.default` — as
having **no measurable effect**, and that verdict is recorded in the code's own
provenance table rather than quietly dropped.

So a new rule needs:

1. **A gap it closes**, referenced to `docs/ANALYSIS.md` or to a reproduction.
2. **A fixture** that exhibits the gap, with a negative control so a test
   cannot pass for the wrong reason.
3. **A unit test** of the rule firing *and* not firing.
4. **A metric it should move**, and a simulation run showing whether it does.
   If it does not, say so — a rule with an honest "no measurable difference"
   verdict is welcome; a rule with an unbacked claim is not.

If you cannot run the simulation (it needs a GPU), open the PR with the first
three and say so. Someone with the hardware can run the fourth.

## Setup

```bash
uv venv --python 3.10 .venv
uv pip install --python .venv/bin/python -e '.[core,schemas]' --group dev
PYTHONPATH= .venv/bin/python -m pytest tests -q
```

`PYTHONPATH=` is not decoration: a sourced ROS 2 environment leaks
`/opt/ros/<distro>/lib/python3.*/site-packages` into the venv and pytest then
autoloads ROS's `launch_testing` plugin, which dies before collection. See
`docs/VERIFY.md`.

### The optional tiers

| Tier | Needs | Command |
|---|---|---|
| T0 unit | nothing but `usd-core` | `pytest tests/unit` |
| T1 converter | Linux/Windows (`usd-exchange` has no macOS wheel) | `python scripts/run_converter_matrix.py && pytest tests/converter` |
| T2 Isaac Sim | Linux + NVIDIA GPU + Isaac Sim 6.x | `<isaac>/python.sh scripts/verify_isaac_regression.py` |
| T3 simulation | the same, plus patience | `<isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts` |

Tests that need a tier you do not have **skip with a reason**. They never fail
silently and they never pass vacuously.

## House style

* **Line length 110**, `ruff` and `black` enforced. `ruff check . && black --check .`
* **Comments explain why, never what.** A comment that restates the code will be
  asked to go. A comment recording a measurement, a version-specific quirk, or
  a decision that looks wrong without context is exactly what we want — see
  `repair/drives.py` on the per-degree/per-radian split.
* **No new dependency** without a reason in the PR description. The core codes
  against `pxr` and `numpy` only, deliberately.
* **Units are never implicit.** Everything is computed in SI and converted once,
  in `model/units.py`. If you add a backend, add its row to the unit table in
  `docs/PHASE3_DESIGN.md` §3 and a case to
  `tests/unit/test_repair_units.py` — the test that fails on a 57.3× error.

## Things that will get a PR rejected

* A repair that overwrites a value the input authored, without `--force`.
* A number in a doc that is not reproducible from a command in the repo.
* Writing into `references/`. Those checkouts are read-only; see `CLAUDE.md`.
* A claim of "measured" for something that was reasoned about.
* Timestamps, hostnames or absolute paths in an authored USD layer — the output
  must stay byte-identical across runs.

## Where things live

```
src/urdf_usd_bridge/
  model/        articulation and units. Pure computation
  inspection/   read-only reporting. Never authors
  repair/       the rules. One module per gap
  sim/          the cross-backend harness
    metrics.py  pure: arrays in, numbers out. Testable without a GPU
    scene.py    the scene contract and its guards
docs/           one report per phase, plus the design that preceded it
tests/unit/     no GPU, no converter needed
tests/converter/ needs the converter matrix built
```

## Reporting a bug in something we depend on

`docs/UPSTREAM_ISSUES.md` is the format: a summary, the root cause with
file:line, a minimal reproduction, expected vs actual, and — importantly — a
section on **what you could not establish**. Three issues are written up there;
two are reproduced and one sub-issue is explicitly marked as a source reading
that was never observed running.
