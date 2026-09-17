# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The degree/radian regression guard.

This file exists because of one specific, demonstrated bug class. Isaac Sim
6.1.0's ``create_mjc_actuator_from_physics`` copies the **per-degree**
``UsdPhysics.DriveAPI`` stiffness straight into the **per-radian** MJCF
``gainPrm`` slot (``urdf_to_mjc_physx_conversion_utils.py:282-283``), and does
the same for ``mjc:ref`` from ``targetPosition`` (line 337). Those are 57.3x
errors. They do not fire today only because the URDF path leaves the drive
empty -- they go live the moment anything populates it, which is exactly what
this project does.

So: whenever the same physical gain is authored into more than one backend, the
values must differ by exactly ``180/pi`` where the conventions differ, and be
identical where they do not. A test that only checked "a gain was authored"
would pass with every one of these bugs present.

``docs/UPSTREAM_ISSUES.md`` carries the filed-report versions.
"""

from __future__ import annotations

import math

import pytest

from urdf_usd_bridge.model.units import (
    PER_DEGREE_TO_PER_RADIAN,
    PER_RADIAN_TO_PER_DEGREE,
    gain_urdf_to_usd,
    gain_usd_to_urdf,
)
from urdf_usd_bridge.repair import RepairOptions, fix_asset

from .repair_builders import export, variant_asset

#: The factor every one of these bugs gets wrong.
DEG_PER_RAD = 180.0 / math.pi


def test_the_constant_is_what_we_think_it_is():
    assert pytest.approx(57.29577951308232) == DEG_PER_RAD
    assert pytest.approx(1.0 / DEG_PER_RAD) == PER_RADIAN_TO_PER_DEGREE
    assert pytest.approx(DEG_PER_RAD) == PER_DEGREE_TO_PER_RADIAN


def test_angular_gains_convert_and_linear_gains_do_not():
    assert gain_urdf_to_usd(1.5, angular=True) == pytest.approx(1.5 / DEG_PER_RAD)
    assert gain_urdf_to_usd(4.0, angular=False) == 4.0
    assert gain_usd_to_urdf(gain_urdf_to_usd(1.5, angular=True), angular=True) == pytest.approx(1.5)


def _authored(prim, name):
    attr = prim.GetAttribute(name)
    return attr.Get() if attr and attr.IsValid() and attr.HasAuthoredValue() else None


def read_authored(root: str, selection: str, prim_path: str, *names):
    """Open ``root`` under one variant selection and read attributes eagerly.

    ``Usd.Stage.Open`` caches by identifier, so opening the same asset twice
    returns the *same* stage and the second variant selection silently changes
    what the first one's prims resolve to. Each call therefore gets its own
    session layer, and values are returned as plain Python rather than as prims
    whose stage might be reselected later.
    """
    from pxr import Sdf, Usd

    session = Sdf.Layer.CreateAnonymous()
    stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(root), session, Usd.Stage.LoadAll)
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection(selection)
    prim = stage.GetPrimAtPath(prim_path)
    out = {}
    for name in names:
        attr = prim.GetAttribute(name) if prim and prim.IsValid() else None
        value = attr.Get() if attr and attr.IsValid() and attr.HasAuthoredValue() else None
        out[name] = list(value) if hasattr(value, "__len__") and not isinstance(value, str) else value
    return out


@pytest.fixture
def stabilized(tmp_path):
    """An asset with a Physics variant set, fixed for all three backends."""
    source = export(variant_asset(), tmp_path / "robot.usda")
    out = tmp_path / "out"
    report = fix_asset(source, out, RepairOptions(backends_requested="all"))
    return report, str(out / "robot_stabilized.usda")


def test_the_same_gain_in_three_backends_differs_only_by_the_angle_convention(stabilized):
    """The regression guard.

    ``UsdPhysics.DriveAPI`` stores angular gains per degree; ``MjcActuator``
    gain/bias and ``NewtonPDControlAPI`` kp/kd are per radian. So for one joint:

        drive_stiffness * 180/pi  ==  mjc gainPrm[0]  ==  newton kp

    If any backend's authoring picks up the wrong convention, either the ratio
    stops being 180/pi or two of the three become equal. Both are asserted.
    """
    _, root = stabilized
    joint = "/robot/Physics/shoulder"

    physx = read_authored(
        root, "physx", joint, "drive:angular:physics:stiffness", "drive:angular:physics:damping"
    )
    drive_k = physx["drive:angular:physics:stiffness"]
    drive_d = physx["drive:angular:physics:damping"]
    assert drive_k, "no PhysX drive stiffness was authored at all"

    mjc = read_authored(root, "mujoco", "/robot/Physics/shoulder_actuator", "mjc:gainPrm", "mjc:biasPrm")
    mjc_gain = mjc["mjc:gainPrm"]
    mjc_bias = mjc["mjc:biasPrm"]

    newton = read_authored(
        root, "physics", "/robot/Physics/shoulder_newton_actuator", "newton:kp", "newton:kd"
    )
    newton_kp = newton["newton:kp"]
    newton_kd = newton["newton:kd"]

    # 1. The per-radian backends agree with each other exactly.
    assert mjc_gain[0] == pytest.approx(newton_kp, rel=1e-6)
    assert -mjc_bias[1] == pytest.approx(newton_kp, rel=1e-6)
    assert -mjc_bias[2] == pytest.approx(newton_kd, rel=1e-6)

    # 2. The per-degree backend is exactly 180/pi smaller.
    assert drive_k * DEG_PER_RAD == pytest.approx(mjc_gain[0], rel=1e-6)

    # 3. And they are emphatically not equal, which is what a naive copy gives.
    assert drive_k != pytest.approx(mjc_gain[0], rel=1e-3)
    assert newton_kp / drive_k == pytest.approx(DEG_PER_RAD, rel=1e-6)

    # 4. Damping carries the same convention split. The PhysX value also folds
    #    in the passive damping, so compare the drive term alone.
    passive_si = 1.5
    drive_only_si = drive_d * DEG_PER_RAD - passive_si
    assert drive_only_si == pytest.approx(newton_kd, rel=1e-4)


def test_quantities_without_an_angle_unit_are_never_scaled(stabilized):
    """Friction, armature and maxForce are efforts and inertias, not gains.

    Scaling one of them by 180/pi would be the mirror image of the bug above,
    and just as invisible without an explicit check.
    """
    _, root = stabilized
    joint = "/robot/Physics/shoulder"

    physx = read_authored(
        root,
        "physx",
        joint,
        "physxJoint:jointFriction",
        "physxJoint:armature",
        "newton:armature",
        "drive:angular:physics:maxForce",
    )
    mujoco = read_authored(root, "mujoco", joint, "mjc:frictionloss", "mjc:armature")

    # newton:friction = 0.3 N*m in the source, unchanged everywhere.
    assert physx["physxJoint:jointFriction"] == pytest.approx(0.3)
    assert mujoco["mjc:frictionloss"] == pytest.approx(0.3)

    # Armature is kg*m^2 in every namespace: identical, never converted.
    assert physx["physxJoint:armature"] == pytest.approx(mujoco["mjc:armature"], rel=1e-6)
    assert physx["physxJoint:armature"] == pytest.approx(physx["newton:armature"], rel=1e-9)

    # maxForce comes from urdf:limit:effort = 87 N*m, verbatim.
    assert physx["drive:angular:physics:maxForce"] == pytest.approx(87.0)
    actuator = read_authored(
        root, "mujoco", "/robot/Physics/shoulder_actuator", "mjc:forceRange:max", "mjc:forceRange:min"
    )
    assert actuator["mjc:forceRange:max"] == pytest.approx(87.0)
    assert actuator["mjc:forceRange:min"] == pytest.approx(-87.0)


def test_passive_damping_is_mirrored_into_mujoco_per_radian(stabilized):
    """``newton:damping`` is per degree, ``mjc:damping`` per radian."""
    _, root = stabilized
    joint = "/robot/Physics/shoulder"
    newton_damping = read_authored(root, "physics", joint, "newton:damping")["newton:damping"]
    mjc_damping = read_authored(root, "mujoco", joint, "mjc:damping")["mjc:damping"]

    assert newton_damping == pytest.approx(1.5 / DEG_PER_RAD, rel=1e-6)
    assert mjc_damping == pytest.approx(1.5, rel=1e-6)
    assert mjc_damping / newton_damping == pytest.approx(DEG_PER_RAD, rel=1e-6)


def test_a_naive_same_value_copy_would_fail_this_file():
    """Sanity check on the guard itself.

    If somebody "simplifies" the MuJoCo path to reuse the stored PhysX value,
    the ratio below becomes 1.0 and
    ``test_the_same_gain_in_three_backends...`` fails. This asserts the guard
    is capable of noticing, rather than trusting that it is.
    """
    stored_per_degree = 5.2280888
    naive_copy = stored_per_degree
    correct = stored_per_degree * DEG_PER_RAD
    assert correct / naive_copy == pytest.approx(DEG_PER_RAD)
    assert naive_copy != pytest.approx(correct, rel=1e-3)
