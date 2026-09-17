#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Convert the fixtures with several urdf-usd-converter versions and inspect each result.

One isolated virtualenv is built per converter version, because the versions
cannot coexist in one interpreter. Each env gets that converter plus this
package (editable, no extras -- ``pxr`` arrives via the converter's own
``usd-exchange`` dependency).

Artifacts land under ``--out`` (default ``tests/_artifacts``)::

    matrix.json                         environment + per-run status
    <version>/<fixture>.json            the inspect report
    <version>/<fixture>/                the converted USD asset

``tests/converter/test_converter_matrix.py`` asserts against those artifacts and
skips cleanly when they are absent.

Platform note
-------------
``urdf-usd-converter`` depends on ``usd-exchange``, which publishes manylinux and
win_amd64 wheels only. On macOS the install step fails and this script records
the failure in ``matrix.json`` rather than pretending to have run. See
``docs/VERIFY.md``.
"""

from __future__ import annotations

import argparse
import json
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"
DEFAULT_VERSIONS = ("0.3.2", "0.3.3")
DEFAULT_OUT = REPO_ROOT / "tests" / "_artifacts"


def _run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, **kwargs)


def _tail(text: str, lines: int = 12) -> str:
    return "\n".join(text.strip().splitlines()[-lines:])


def _cause(text: str) -> str:
    """The one line that explains a failure, for a readable pytest skip reason."""
    lines = [line.strip() for line in text.splitlines()]
    for prefix in ("cause:", "ERROR:", "error:"):
        for line in lines:
            if line.startswith(prefix):
                return line
    return _tail(text, 1)


def build_env(version: str, env_dir: Path, python: str) -> dict:
    """Create a virtualenv holding one converter version plus this package."""
    if env_dir.exists():
        shutil.rmtree(env_dir)

    uv = shutil.which("uv")
    if uv:
        created = _run([uv, "venv", "--python", python, str(env_dir)])
    else:
        created = _run([sys.executable, "-m", "venv", str(env_dir)])
    if created.returncode != 0:
        output = created.stderr or created.stdout
        return {"ok": False, "step": "venv", "error": _tail(output), "cause": _cause(output)}

    interpreter = env_dir / ("Scripts" if platform.system() == "Windows" else "bin") / "python"
    requirements = [f"urdf-usd-converter=={version}", "-e", str(REPO_ROOT)]
    if uv:
        installed = _run([uv, "pip", "install", "--python", str(interpreter), *requirements])
    else:
        installed = _run([str(interpreter), "-m", "pip", "install", *requirements])
    if installed.returncode != 0:
        output = installed.stderr or installed.stdout
        return {
            "ok": False,
            "step": "install",
            "error": _tail(output),
            "cause": _cause(output),
        }

    # A uv-created venv has no pip of its own, so ask uv first and only fall
    # back to `python -m pip`. Without this the recorded environment is empty,
    # which defeats the point of the artifact.
    frozen = _run([uv, "pip", "freeze", "--python", str(interpreter)]) if uv else None
    if frozen is None or frozen.returncode != 0:
        frozen = _run([str(interpreter), "-m", "pip", "freeze"])
    return {
        "ok": True,
        "python": str(interpreter),
        "frozen": frozen.stdout.splitlines() if frozen.returncode == 0 else [],
    }


def convert_and_inspect(interpreter: str, urdf: Path, out_dir: Path, report_path: Path) -> dict:
    """Run ``convert`` then ``inspect --json`` for one fixture."""
    out_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    converted = _run([interpreter, "-m", "urdf_usd_bridge", "convert", str(urdf), str(out_dir)])
    if converted.returncode != 0:
        return {
            "ok": False,
            "step": "convert",
            "error": _tail(converted.stderr or converted.stdout),
        }
    asset = converted.stdout.strip().splitlines()[-1]

    inspected = _run(
        [
            interpreter,
            "-m",
            "urdf_usd_bridge",
            "inspect",
            asset,
            "--json",
            "-o",
            str(report_path),
        ]
    )
    if inspected.returncode != 0:
        return {
            "ok": False,
            "step": "inspect",
            "asset": asset,
            "error": _tail(inspected.stderr or inspected.stdout),
        }
    return {
        "ok": True,
        "asset": asset,
        "report": str(report_path.relative_to(REPO_ROOT)),
        "seconds": round(time.time() - started, 3),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--versions", nargs="+", default=list(DEFAULT_VERSIONS))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--python", default="3.10", help="interpreter for the converter envs")
    parser.add_argument("--keep-envs", action="store_true", help="reuse existing venvs instead of rebuilding")
    args = parser.parse_args(argv)

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    fixtures = sorted(FIXTURES_DIR.glob("*.urdf"))
    if not fixtures:
        print(f"no fixtures found in {FIXTURES_DIR}", file=sys.stderr)
        return 1

    matrix: dict = {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": sys.version.split()[0],
        },
        "fixtures": [f.name for f in fixtures],
        "versions": {},
    }

    exit_code = 0
    for version in args.versions:
        env_dir = out_root / f"venv-{version}"
        print(f"==> urdf-usd-converter {version}")
        if args.keep_envs and env_dir.exists():
            interpreter = env_dir / ("Scripts" if platform.system() == "Windows" else "bin") / "python"
            env_info = {"ok": interpreter.exists(), "python": str(interpreter), "frozen": []}
        else:
            env_info = build_env(version, env_dir, args.python)

        entry: dict = {"env": env_info, "runs": {}}
        matrix["versions"][version] = entry

        if not env_info.get("ok"):
            print(f"    env FAILED at {env_info.get('step')}: {env_info.get('cause')}")
            print(f"      {env_info.get('error')}")
            exit_code = 1
            continue

        version_dir = out_root / version
        version_dir.mkdir(parents=True, exist_ok=True)
        for fixture in fixtures:
            stem = fixture.stem
            result = convert_and_inspect(
                env_info["python"],
                fixture,
                version_dir / stem,
                version_dir / f"{stem}.json",
            )
            entry["runs"][stem] = result
            status = "ok" if result["ok"] else f"FAILED at {result['step']}"
            print(f"    {stem}: {status}")
            if not result["ok"]:
                print(f"      {result['error']}")
                exit_code = 1

    matrix_path = out_root / "matrix.json"
    matrix_path.write_text(json.dumps(matrix, indent=2))
    print(f"\nwrote {matrix_path}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
