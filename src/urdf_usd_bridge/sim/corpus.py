# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The robots the matrix runs on. Fixtures plus public URDFs fetched at run time.

Nothing is vendored. Public robots are cloned shallow into a cache directory,
and their upstream URL and commit are recorded in the results file so a number
can be traced back to an exact input.

A fetch failure **skips with a reason**; it never fails the run and never
silently substitutes something else, matching how ``tests/converter`` behaves
when the converter matrix has not been built.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES = REPO_ROOT / "tests" / "fixtures"

#: Where public robots land. Overridable, never inside the repo by default.
CACHE_DIR = Path(os.environ.get("URDF_USD_BRIDGE_CORPUS", Path.home() / ".cache" / "urdf-usd-bridge-corpus"))


@dataclass
class RobotSpec:
    """One entry in the corpus."""

    label: str
    urdf: str | None = None
    #: Public robots only.
    git_url: str | None = None
    git_ref: str | None = None
    urdf_in_repo: str | None = None
    role: str = "fixture"
    #: Exploratory entries are reported separately from headline numbers.
    exploratory: bool = False
    licence: str = ""
    notes: str = ""
    resolved: dict[str, Any] = field(default_factory=dict)


#: The four fixtures, which are what the repairs were written for.
FIXTURE_ROBOTS = [
    RobotSpec("a_dynamics_damping", urdf=str(FIXTURES / "a_dynamics_damping.urdf"), role="fixture"),
    RobotSpec(
        "b_inertial_origin_mass_no_inertia",
        urdf=str(FIXTURES / "b_inertial_origin_mass_no_inertia.urdf"),
        role="fixture",
    ),
    RobotSpec("c_revolute_no_limit", urdf=str(FIXTURES / "c_revolute_no_limit.urdf"), role="fixture"),
    RobotSpec("d_mesh_collision", urdf=str(FIXTURES / "d_mesh_collision.urdf"), role="fixture"),
]

#: Decision N2: Panda and a gripper/arm are the primary corpus; the quadruped
#: is exploratory only, because its contact behaviour is governed by G4/G5
#: (collision approximation and physics materials) which Phase 3 does not touch.
PUBLIC_ROBOTS = [
    RobotSpec(
        "so101_arm",
        git_url="https://github.com/TheRobotStudio/SO-ARM100",
        git_ref="main",
        urdf_in_repo="Simulation/SO101/so101_new_calib.urdf",
        role="primary",
        licence="Apache-2.0 (SO-ARM100)",
        notes="5-DoF arm with a gripper; one of the corpora urdf-usd-converter benchmarks against",
    ),
    RobotSpec(
        "so100_arm",
        git_url="https://github.com/TheRobotStudio/SO-ARM100",
        git_ref="main",
        urdf_in_repo="Simulation/SO100/so100.urdf",
        role="primary",
        licence="Apache-2.0 (SO-ARM100)",
        notes="the earlier SO-ARM revision; a second real arm from the same source",
    ),
    RobotSpec(
        "unitree_go2",
        git_url="https://github.com/unitreerobotics/unitree_ros",
        git_ref="master",
        urdf_in_repo="robots/go2_description/urdf/go2_description.urdf",
        role="exploratory",
        exploratory=True,
        licence="BSD-3-Clause (unitree_ros)",
        notes="floating-base quadruped; drop test only, reported separately (decision N2)",
    ),
]

#: Robots considered and dropped, kept here so the choice is auditable.
#:
#: Franka Panda (``frankaemika/franka_description``) and UR5
#: (``ros-industrial/universal_robot``) were the design's first choices, but
#: both ship **xacro**, not URDF. Expanding xacro needs the ROS ``xacro`` tool
#: *and* an ament package index -- ROS Humble is installed on the test host and
#: still could not resolve ``package 'ur_description' not found`` from a bare
#: clone. Adding a ROS workspace build to a physics harness is more moving
#: parts than the measurement is worth, so the corpus uses arms that ship plain
#: URDF instead.
DROPPED_ROBOTS = {
    "franka_panda": "ships xacro only; needs a ROS ament package index to expand",
    "ur5": "ships xacro only; needs a ROS ament package index to expand",
}


def _run(cmd: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)


def fetch(spec: RobotSpec, *, cache: Path | None = None) -> RobotSpec:
    """Shallow-clone a public robot and resolve its URDF path.

    Sets ``spec.resolved`` with either ``{"urdf": ..., "commit": ...}`` or
    ``{"skipped": <reason>}``.
    """
    if spec.urdf:
        spec.resolved = {"urdf": spec.urdf, "commit": None}
        return spec
    if not spec.git_url:
        spec.resolved = {"skipped": "no source configured"}
        return spec

    cache = cache or CACHE_DIR
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / spec.label

    if not (target / ".git").exists():
        if shutil.which("git") is None:
            spec.resolved = {"skipped": "git is not installed"}
            return spec
        clone = _run(
            ["git", "clone", "--depth", "1", "--branch", spec.git_ref or "HEAD", spec.git_url, str(target)]
        )
        if clone.returncode != 0:
            spec.resolved = {
                "skipped": f"clone failed: {(clone.stderr or clone.stdout).strip().splitlines()[-1:]}"
            }
            return spec

    commit = _run(["git", "-C", str(target), "rev-parse", "HEAD"]).stdout.strip()
    urdf = target / (spec.urdf_in_repo or "")
    if not urdf.exists():
        found = sorted(target.rglob("*.urdf"))
        if not found:
            spec.resolved = {"skipped": f"no URDF at {spec.urdf_in_repo} and none found in the clone"}
            return spec
        urdf = found[0]
    spec.resolved = {
        "urdf": str(urdf),
        "commit": commit,
        "url": spec.git_url,
        "ref": spec.git_ref,
        "licence": spec.licence,
    }
    return spec


def corpus(*, include_public: bool = True, include_exploratory: bool = False) -> list[RobotSpec]:
    """The robot list, in report order."""
    robots = list(FIXTURE_ROBOTS)
    if include_public:
        for spec in PUBLIC_ROBOTS:
            if spec.exploratory and not include_exploratory:
                continue
            robots.append(spec)
    return robots
