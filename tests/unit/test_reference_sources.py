# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Static evidence for the G1 regression, read from the pinned reference trees.

This is *source* evidence, not *behavioural* evidence. It proves the attribute
names written by each converter version and the attribute names read by Isaac
Sim, and that those sets do not intersect. It does not prove what a running
Isaac Sim does; that is what ``scripts/verify_isaac_regression.py`` is for.

It runs anywhere ``git`` and the ``references/`` checkouts are available, which
includes macOS, where the converters themselves cannot be installed.
"""

from __future__ import annotations

import re
import subprocess

import pytest

from ..conftest import have_git_references

pytestmark = pytest.mark.skipif(not have_git_references(), reason="references/ checkouts are not present")

CONVERTER_LINK_PY = "urdf_usd_converter/_impl/link.py"
ISAAC_CONVERSION_PY = (
    "source/libraries/isaacsim/asset/importer/utils/python/impl/" "urdf_to_mjc_physx_conversion_utils.py"
)
ISAAC_IMPORTER_PYPROJECT = "source/extensions/isaacsim.asset.importer.urdf/pyproject.toml"


def _show(repo, ref: str, path: str) -> str:
    """``git show <ref>:<path>`` from a reference checkout."""
    result = subprocess.run(
        ["git", "-C", str(repo), "show", f"{ref}:{path}"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.skip(f"{ref}:{path} unavailable in this checkout: {result.stderr.strip()}")
    return result.stdout


def _authored_attribute_names(source: str) -> set[str]:
    """Attribute names the source passes to ``CreateAttribute``/``set_schema_attribute``."""
    names = set(re.findall(r'CreateAttribute\(\s*"([^"]+)"', source))
    names |= set(re.findall(r'set_schema_attribute\(\s*[^,]+,\s*"([^"]+)"', source))
    return names


def _read_attribute_names(source: str) -> set[str]:
    """Attribute names the source passes to ``GetAttribute``."""
    return set(re.findall(r'GetAttribute\(\s*"([^"]+)"', source))


@pytest.mark.parametrize("tag", ["v0.3.2", "v0.3.3"])
def test_modern_converter_authors_newton_namespace_not_urdf_dynamics(references_dir, tag):
    source = _show(references_dir / "urdf-usd-converter", tag, CONVERTER_LINK_PY)
    authored = _authored_attribute_names(source)

    assert "newton:damping" in authored
    assert "newton:friction" in authored
    assert "urdf:dynamics:damping" not in authored
    assert "urdf:dynamics:friction" not in authored


def test_old_converter_did_author_urdf_dynamics(references_dir):
    """0.1.3 is what Isaac Sim 6.0.1 pinned, and its spelling matched Isaac's reader."""
    source = _show(references_dir / "urdf-usd-converter", "v0.1.3", CONVERTER_LINK_PY)
    authored = _authored_attribute_names(source)

    assert "urdf:dynamics:damping" in authored
    assert "urdf:dynamics:friction" in authored


@pytest.mark.parametrize("tag", ["v0.3.2", "v0.3.3"])
def test_converter_authors_no_drive_api(references_dir, tag):
    """G1, first half: the converter never authors actuation of any kind."""
    repo = references_dir / "urdf-usd-converter"
    listing = subprocess.run(
        ["git", "-C", str(repo), "grep", "-l", "DriveAPI", tag, "--", "urdf_usd_converter"],
        capture_output=True,
        text=True,
    )
    assert listing.stdout.strip() == "", f"unexpected DriveAPI usage at {tag}:\n{listing.stdout}"


def test_isaac_61_reads_the_spelling_its_pinned_converter_no_longer_writes(references_dir):
    """G1, second half: the producer and the consumer disagree on the name."""
    isaac = references_dir / "IsaacSim"
    converter = references_dir / "urdf-usd-converter"

    pinned = _show(isaac, "v6.1.0", ISAAC_IMPORTER_PYPROJECT)
    match = re.search(r"urdf-usd-converter==([0-9.]+)", pinned)
    assert match, "could not find the urdf-usd-converter pin in Isaac Sim 6.1.0"
    pinned_version = match.group(1)
    assert pinned_version == "0.3.2"

    isaac_reads = _read_attribute_names(_show(isaac, "v6.1.0", ISAAC_CONVERSION_PY))
    converter_writes = _authored_attribute_names(_show(converter, f"v{pinned_version}", CONVERTER_LINK_PY))

    # Isaac looks for these two, and they are exactly what 0.3.2 stopped writing.
    assert "urdf:dynamics:damping" in isaac_reads
    assert "urdf:dynamics:friction" in isaac_reads
    assert not {"urdf:dynamics:damping", "urdf:dynamics:friction"} & converter_writes

    # The one URDF attribute that does still connect is the effort limit.
    assert "urdf:limit:effort" in isaac_reads & converter_writes


def test_isaac_60_pinned_a_converter_whose_spelling_matched(references_dir):
    """The same Isaac reader worked in 6.0.1, which makes 6.1.0 a regression."""
    isaac = references_dir / "IsaacSim"
    converter = references_dir / "urdf-usd-converter"

    pinned = _show(isaac, "v6.0.1", "deps/pip_urdf_usd.toml")
    match = re.search(r"urdf-usd-converter==([0-9.]+)", pinned)
    assert match
    assert match.group(1) == "0.1.3"

    converter_writes = _authored_attribute_names(_show(converter, "v0.1.3", CONVERTER_LINK_PY))
    isaac_reads = _read_attribute_names(
        _show(
            isaac,
            "v6.0.1",
            "source/extensions/isaacsim.asset.importer.utils/python/impl/"
            "urdf_to_mjc_physx_conversion_utils.py",
        )
    )
    assert {"urdf:dynamics:damping", "urdf:dynamics:friction"} <= (isaac_reads & converter_writes)


def test_isaac_61_applies_drive_api_without_authoring_gains(references_dir):
    """Drives exist on every actuatable joint, but carry no stiffness or damping."""
    source = _show(
        references_dir / "IsaacSim",
        "v6.1.0",
        "source/libraries/isaacsim/asset/importer/utils/python/impl/importer_utils.py",
    )
    body = source.split("def add_joint_schemas")[1].split("\ndef ")[0]
    assert "UsdPhysics.DriveAPI.Apply" in body
    assert "CreateStiffnessAttr" not in body
    assert "CreateDampingAttr" not in body


def test_converter_032_can_author_principal_axes_without_an_inertia_tensor(references_dir):
    """The 0.3.2 zero-quaternion defect, and its 0.3.3 fix, in source form."""
    converter = references_dir / "urdf-usd-converter"

    def apply_inertial(tag: str) -> str:
        source = _show(converter, tag, CONVERTER_LINK_PY)
        return source.split("def apply_inertial")[1].split("\ndef ")[0]

    old = apply_inertial("v0.3.2")
    new = apply_inertial("v0.3.3")

    # 0.3.2 writes principalAxes from inside the `origin` branch, which runs even
    # when no <inertia> was present.
    origin_branch_032 = old.split("if link.inertial.origin:")[1]
    assert "GetPrincipalAxesAttr().Set" in origin_branch_032

    # 0.3.3 writes it only from the branch guarded on an inertia tensor.
    origin_branch_033 = new.split("if link.inertial.origin:")[1]
    assert "GetPrincipalAxesAttr().Set" not in origin_branch_033
    inertia_branch_033 = new.split("if link.inertial and link.inertial.inertia:")[1]
    assert "GetPrincipalAxesAttr().Set" in inertia_branch_033


def test_converter_authors_only_convex_hull_for_mesh_colliders(references_dir):
    """G4: there is exactly one approximation in the converter's vocabulary."""
    source = _show(references_dir / "urdf-usd-converter", "v0.3.2", "urdf_usd_converter/_impl/geometry.py")
    approximations = set(re.findall(r"UsdPhysics\.Tokens\.(\w+)", source))
    assert "convexHull" in approximations
    assert not approximations & {"convexDecomposition", "boundingCube", "boundingSphere", "sdf"}
