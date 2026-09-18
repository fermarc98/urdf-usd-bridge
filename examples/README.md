# Examples

Three runnable scripts, in increasing order of what they need.

| Example | Needs | Shows |
|---|---|---|
| `01_inspect_report.py` | `usd-core` only | what a converted asset is missing, and how to read the three-state readings |
| `02_fix_and_verify.py` | `[convert]` extra (Linux/Windows) | convert, repair, and prove the gaps closed with `inspect` |
| `03_cross_backend.py` | Isaac Sim + NVIDIA GPU | the same asset in PhysX, Newton and MuJoCo, and how far apart they end up |

Run them from the repo root:

```bash
PYTHONPATH= .venv/bin/python examples/01_inspect_report.py
```

`PYTHONPATH=` matters if you have ROS sourced; see `CONTRIBUTING.md`.
