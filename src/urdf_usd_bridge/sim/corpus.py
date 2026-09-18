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
    #: Robots from the same repository share one clone, so a twelve-robot
    #: corpus costs two downloads rather than twelve.
    share_clone: str = ""
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
_UNITREE = "https://github.com/unitreerobotics/unitree_ros"
_SOARM = "https://github.com/TheRobotStudio/SO-ARM100"


def _unitree(label: str, path: str, *, role: str = "primary", exploratory: bool = False, notes: str = ""):
    """One robot from ``unitree_ros``. All of them ship plain URDF.

    They share a clone, so adding the eleventh robot costs no extra download.
    """
    return RobotSpec(
        label=label,
        git_url=_UNITREE,
        git_ref="master",
        urdf_in_repo=path,
        role=role,
        exploratory=exploratory,
        licence="BSD-3-Clause (unitree_ros)",
        notes=notes,
        share_clone="unitree_ros",
    )


PUBLIC_ROBOTS = [
    # --- arms -------------------------------------------------------------
    RobotSpec(
        "so101_arm",
        git_url=_SOARM,
        git_ref="main",
        urdf_in_repo="Simulation/SO101/so101_new_calib.urdf",
        role="primary",
        licence="Apache-2.0 (SO-ARM100)",
        notes="5-DoF arm with a gripper; one of the corpora urdf-usd-converter benchmarks against",
        share_clone="so_arm",
    ),
    RobotSpec(
        "so100_arm",
        git_url=_SOARM,
        git_ref="main",
        urdf_in_repo="Simulation/SO100/so100.urdf",
        role="primary",
        licence="Apache-2.0 (SO-ARM100)",
        notes="the earlier SO-ARM revision",
        share_clone="so_arm",
    ),
    _unitree("unitree_z1", "robots/z1_description/xacro/z1.urdf", notes="6-DoF industrial arm"),
    # --- quadrupeds -------------------------------------------------------
    _unitree("unitree_a1", "robots/a1_description/urdf/a1.urdf", notes="12-DoF quadruped"),
    _unitree("unitree_go1", "robots/go1_description/urdf/go1.urdf", notes="12-DoF quadruped"),
    _unitree(
        "unitree_go2",
        "robots/go2_description/urdf/go2_description.urdf",
        role="exploratory",
        exploratory=True,
        notes="12-DoF quadruped; drop test only, reported separately (decision N2)",
    ),
    _unitree("unitree_aliengo", "robots/aliengo_description/urdf/aliengo.urdf", notes="12-DoF quadruped"),
    _unitree("unitree_b2", "robots/b2_description/urdf/b2_description.urdf", notes="larger quadruped"),
    _unitree("unitree_laikago", "robots/laikago_description/urdf/laikago.urdf", notes="earlier quadruped"),
    # --- humanoids and hands ---------------------------------------------
    _unitree("unitree_h1", "robots/h1_description/urdf/h1.urdf", notes="19-DoF humanoid"),
    _unitree("unitree_g1", "robots/g1_description/g1_23dof.urdf", notes="23-DoF humanoid"),
    _unitree(
        "unitree_dex_hand",
        "robots/dexterous_hand_description/dex2_5/Right_Hand_G1_5010_Wrist.urdf",
        notes="dexterous hand: the low-inertia DOFs the armature floor exists for",
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
    "franka_panda": (
        "frankaemika/franka_description ships xacro only; expanding it needs the ROS xacro tool "
        "and an ament package index. ROS Humble is installed on the test host and still could "
        "not resolve the package from a bare clone"
    ),
    "ur5": ("ros-industrial/universal_robot ships xacro only; same reason as franka_panda"),
    "unitree_h1_2": (
        "h1_2_description carries several variants and no obvious canonical one; h1 and g1 "
        "already cover the humanoid case"
    ),
}

#: How to add a xacro robot if one is ever needed. Not used by the corpus, and
#: written down so the option is a decision rather than a rediscovery.
XACRO_NOTE = """\
A xacro robot needs the package on an ament index, not merely on disk:

    mkdir -p ws/src && ln -s <clone> ws/src/<pkg>
    cd ws && colcon build --packages-select <pkg>
    source install/setup.bash
    xacro <pkg>/urdf/robot.xacro > robot.urdf

That is a ROS workspace build inside a physics harness, which is why the corpus
prefers robots that ship plain URDF.
"""


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
    target = cache / (spec.share_clone or spec.label)

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
        # Deliberately no fallback search: picking "some other URDF in the repo"
        # would silently measure a different robot than the one named.
        spec.resolved = {"skipped": f"no URDF at {spec.urdf_in_repo!r} in {spec.git_url}"}
        return spec
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
